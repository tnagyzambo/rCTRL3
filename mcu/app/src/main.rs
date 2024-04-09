#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

mod net;

#[rtic::app(
    device = samv71_pac,
    dispatchers = [IXC]
)]
mod app {
    use crate::net::{NetworkComponents, NetworkManager, NetworkStack};
    use core::sync::atomic::{AtomicUsize, Ordering};
    use hal::{
        gmac::{
            mac_state, Mac, PhyInterrupt, RxBuf, RxBufData, RxBufDesc, TxBuf, TxBufData, TxBufDesc,
            D_BUF_COUNT, MTU, RX_BUF_COUNT, RX_BUF_DATA_DEFAULT, RX_BUF_DESC_DEFAULT, TX_BUF_COUNT,
            TX_BUF_DATA_DEFAULT, TX_BUF_DESC_DEFAULT,
        },
        rtt::Rtt,
        *,
    };
    use minimq::{ConfigBuilder, Minimq, Publication};
    use rtic_monotonics::systick::*;
    use samv71_pac as pac;
    use smoltcp::{
        iface::{Config, Interface, SocketHandle, SocketSet, SocketStorage},
        socket::{dhcpv4, tcp},
        time::Instant,
        wire::{EthernetAddress, IpCidr},
    };

    const F_SLCK: u32 = 32768; // 32.768 kHz
    const TCP_PORT: u16 = 8080;

    // Timestamp for DEFMT logging
    // TODO: Change to real timestamp
    static COUNT: AtomicUsize = AtomicUsize::new(0);
    defmt::timestamp!("{=usize}", COUNT.fetch_add(1, Ordering::Relaxed));

    #[shared]
    struct Shared {
        active: bool,
        net: NetworkComponents,
        //led: Pin<PA23, Output>,
    }

    #[local]
    struct Local {
        banka: pac::PIOA,
        dhcp_handle: SocketHandle,
        tcp_handle: SocketHandle,
    }

    #[init(local = [
        mac_rx_descs: [RxBufDesc; RX_BUF_COUNT] = [RX_BUF_DESC_DEFAULT; RX_BUF_COUNT],
        mac_rx_datas: [RxBufData; RX_BUF_COUNT] = [RX_BUF_DATA_DEFAULT; RX_BUF_COUNT],
        mac_tx_descs: [TxBufDesc; TX_BUF_COUNT] = [TX_BUF_DESC_DEFAULT; TX_BUF_COUNT],
        mac_tx_datas: [TxBufData; TX_BUF_COUNT] = [TX_BUF_DATA_DEFAULT; TX_BUF_COUNT],
        mac_drx_descs: [RxBufDesc; D_BUF_COUNT] = [RX_BUF_DESC_DEFAULT; D_BUF_COUNT],
        mac_drx_datas: [RxBufData; D_BUF_COUNT] = [RX_BUF_DATA_DEFAULT; D_BUF_COUNT],
        mac_dtx_descs: [TxBufDesc; D_BUF_COUNT] = [TX_BUF_DESC_DEFAULT; D_BUF_COUNT],
        mac_dtx_datas: [TxBufData; D_BUF_COUNT] = [TX_BUF_DATA_DEFAULT; D_BUF_COUNT],
        socket_storage: [SocketStorage<'static>; 2] = [SocketStorage::EMPTY; 2],
        tcp_rx_storage: [u8; MTU] = [0; MTU],
        tcp_tx_storage: [u8; MTU] = [0; MTU],
        net_manager: Option<NetworkManager> = None,
    ])]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        let init::LocalResources {
            socket_storage,
            tcp_rx_storage,
            tcp_tx_storage,
            ..
        } = ctx.local;
        // Init start
        defmt::info!("INIT");

        // Init device clocks
        ctx.device = init_clocks(ctx.device);

        // Init GPIO
        ctx.device = init_gpio(ctx.device);

        // TODO: Move SysTick cofig to function
        // Create SysTick monotonic
        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 12_000_000, systick_mono_token);

        // Disable SysTick counter
        // Otherwise SysTick will keep system from sleeping
        unsafe {
            (*pac::SYS_TICK::PTR)
                .csr()
                .write(|w| w.enable().clear_bit())
        };

        // Init RTT
        let rtt = Rtt::new(ctx.device.RTT, F_SLCK, 3);

        // Zip together GMAC buffers with their descriptors
        // Direct use of the descriptors and data storages after this point is UB
        // The RxBuf/TxBuf structures save pointers to the storages
        // Idealy they would also own their underlying storages, but we also need the
        // memory locations to have 'static lifetimes
        // Taking unique mutable references to the storage locations is also not an option
        // since that would require lifetime annotation that is incompatible with the
        // smoltcp Device trait
        let mac_rx_bufs: [RxBuf; RX_BUF_COUNT] = core::array::from_fn(|i| {
            RxBuf::new(
                &mut ctx.local.mac_rx_descs[i],
                &mut ctx.local.mac_rx_datas[i],
            )
        });
        let mac_tx_bufs: [TxBuf; TX_BUF_COUNT] = core::array::from_fn(|i| {
            TxBuf::new(
                &mut ctx.local.mac_tx_descs[i],
                &mut ctx.local.mac_tx_datas[i],
            )
        });
        let mac_drx_bufs: [RxBuf; D_BUF_COUNT] = core::array::from_fn(|i| {
            RxBuf::new(
                &mut ctx.local.mac_drx_descs[i],
                &mut ctx.local.mac_drx_datas[i],
            )
        });
        let mac_dtx_bufs: [TxBuf; D_BUF_COUNT] = core::array::from_fn(|i| {
            TxBuf::new(
                &mut ctx.local.mac_dtx_descs[i],
                &mut ctx.local.mac_dtx_datas[i],
            )
        });

        // Configure MAC
        let mac = Mac::new(
            ctx.device.GMAC,
            &ctx.device.PMC,
            &ctx.device.PIOD,
            [0x04, 0x91, 0x62, 0x01, 0x02, 0x03],
            mac_rx_bufs,
            mac_tx_bufs,
            mac_drx_bufs,
            mac_dtx_bufs,
        );
        mac.config_network_default().config_dma_default();

        // Enable MAC
        let mut mac = mac.enable();

        // smoltcp interface config
        let iface_config = Config::new(EthernetAddress(mac.mac_addr()).into());
        // TODO: Set random seed

        // smoltcp interferace
        let iface = Interface::new(
            iface_config,
            &mut mac,
            Instant::from_millis(rtt.millis()), // TODO: This is hella wrong
        );

        // DHCP socket
        let mut sockets = SocketSet::new(&mut socket_storage[..]);
        let dhcp_socket = dhcpv4::Socket::new();
        let dhcp_handle = sockets.add(dhcp_socket);

        // TCP socket
        let tcp_rx_buffer = tcp::SocketBuffer::new(&mut tcp_rx_storage[..]);
        let tcp_tx_buffer = tcp::SocketBuffer::new(&mut tcp_tx_storage[..]);
        let tcp_socket = tcp::Socket::new(tcp_rx_buffer, tcp_tx_buffer);
        let tcp_handle = sockets.add(tcp_socket);

        // NAL
        let net_stack = NetworkStack::new(iface, mac, sockets, rtt);
        let net_manager = NetworkManager::new(net_stack);
        ctx.local.net_manager.replace(net_manager);

        let net = NetworkComponents::new(ctx.local.net_manager.as_mut().unwrap());

        // MQTT
        //let mutxmqtt: Minimq<'_, _, _, minimq::broker::IpBroker> = Minimq::new(
        //    tcp_socket,
        //    std_embedded_time::StandardClock::default(),
        //    ConfigBuilder::new(localhost.into(), &mut buffer)
        //        .client_id("test")
        //        .unwrap(),
        //);

        (
            Shared { active: true, net },
            Local {
                banka: ctx.device.PIOA,
                dhcp_handle,
                tcp_handle,
            },
        )
    }

    // Handle RTT interrupts
    #[task(binds = RTT, shared = [])]
    fn rtt_interrupt_handler(ctx: rtt_interrupt_handler::Context) {
        //let rtt_interrupt_handler::SharedResources { mut rtt, .. } = ctx.shared;

        //rtt.lock(|rtt| {
        //    for interrupt in rtt.interrupts() {
        //        match interrupt {
        //            rtt::RttInterrupt::RTTINC => (),
        //            rtt::RttInterrupt::ALMS => (),
        //        }
        //    }
        //});
    }

    // Handle GMAC interrupts
    #[task(binds = GMAC, local = [dhcp_handle, tcp_handle, buffer: [u8; MTU] = [0; MTU]], shared = [net] )]
    fn gmac_interrupt_handler(ctx: gmac_interrupt_handler::Context) {
        let gmac_interrupt_handler::LocalResources {
            dhcp_handle,
            tcp_handle,
            buffer,
            ..
        } = ctx.local;

        let gmac_interrupt_handler::SharedResources { mut net, .. } = ctx.shared;

        for interrupt in mac.interrupts() {
            match interrupt {
                gmac::GmacInterrupt::PFTR => defmt::info!("PFTR"),
                gmac::GmacInterrupt::PTZ => defmt::info!("PTZ"),
                gmac::GmacInterrupt::PFNZ => defmt::info!("PFNZ"),
                gmac::GmacInterrupt::HRESP => {
                    defmt::info!(
                        "HRESP, RSR: {:#06b}, TSR: {:#010b}",
                        mac.gmac.rsr().read().bits(),
                        mac.gmac.tsr().read().bits()
                    );
                }
                gmac::GmacInterrupt::ROVR => defmt::info!("RX OVERUN"),
                gmac::GmacInterrupt::TCOMP => defmt::info!("TCOMP"),
                gmac::GmacInterrupt::TFC => defmt::info!("TFC"),
                gmac::GmacInterrupt::RLEX => defmt::info!("RLEX"),
                gmac::GmacInterrupt::TUR => defmt::info!("TUR"),
                //gmac::GmacInterrupt::TXUBR => {
                //    defmt::info!("TXUBR, TSR: {:#010b}", mac.gmac.tsr().read().bits());
                //    mac.gmac.tsr().write(|w| w.ubr().set_bit());
                //}
                gmac::GmacInterrupt::RXUBR => {
                    defmt::info!("RXUBR, RSR: {:#06b}", mac.gmac.rsr().read().bits())
                }
                gmac::GmacInterrupt::RCOMP => (),
                gmac::GmacInterrupt::MFS => defmt::info!("MFS"),
                _ => (),
            }
        }

        // Poll interface for new Rx/Tx events
        iface.poll(
            Instant::from_millis(rtt.lock(|rtt| rtt.millis())),
            mac,
            sockets,
        );

        // Handle DHCP events
        let event = sockets.get_mut::<dhcpv4::Socket>(*dhcp_handle).poll();
        match event {
            None => (),
            Some(dhcpv4::Event::Configured(config)) => {
                defmt::info!("DHCP config acquired!");

                defmt::info!("IP address: {}", config.address);
                iface.update_ip_addrs(|addrs| {
                    addrs.clear();
                    addrs.push(IpCidr::Ipv4(config.address)).unwrap();
                });

                if let Some(router) = config.router {
                    defmt::info!("Default gateway: {}", router);
                    iface.routes_mut().add_default_ipv4_route(router).unwrap();
                } else {
                    defmt::info!("Default gateway: None");
                    iface.routes_mut().remove_default_ipv4_route();
                }

                for (i, s) in config.dns_servers.iter().enumerate() {
                    defmt::info!("DNS server {}: {}", i, s);
                }

                // Open TCP socket
                let tcp = sockets.get_mut::<tcp::Socket>(*tcp_handle);
                tcp.listen(TCP_PORT).ok();
                defmt::info!("TCP OPEN: {}", TCP_PORT);
            }
            Some(dhcpv4::Event::Deconfigured) => {
                defmt::info!("DHCP lost config!");
                iface.update_ip_addrs(|addrs| addrs.clear());
                iface.routes_mut().remove_default_ipv4_route();

                // Close TCP socket
                let tcp = sockets.get_mut::<tcp::Socket>(*tcp_handle);
                tcp.close();
                defmt::info!("TCP CLOSE: {}", TCP_PORT);
            }
        }

        // TCP echo
        let tcp = sockets.get_mut::<tcp::Socket>(*tcp_handle);
        if let Ok(recv_bytes) = tcp.recv_slice(buffer) {
            if recv_bytes > 0 {
                defmt::info!(
                    "TCP RX: {}. @{}ms",
                    recv_bytes,
                    rtt.lock(|rtt| rtt.millis())
                );
                tcp.send_slice(&buffer[..recv_bytes]).ok();
            }
        }

        if !tcp.is_listening() && !tcp.is_open() || tcp.state() == tcp::State::CloseWait {
            tcp.abort();
            tcp.listen(TCP_PORT).ok();
            defmt::info!("TCP DISCONNECT, REOPEN: {}", TCP_PORT);
        }
    }

    fn init_clocks(device: pac::Peripherals) -> pac::Peripherals {
        // Setup clocks on SAMV71
        device.PMC.ckgr_mor().write(|w| {
            // Enable main crystal osc
            w.moscxten().set_bit();
            unsafe {
                // Set main crystal osc wake up time to 62ms
                w.moscxtst().bits(0xFF)
            }
        });

        // Wait for PMC to be correctly configured
        while device.PMC.sr().read().moscxts().bit() {}

        // Switch MAINCK to main crystal osc
        device.PMC.ckgr_mor().write(|w| w.moscsel().set_bit());

        // Wait for MAINCK to complete the switch
        while device.PMC.sr().read().moscsels().bit() {}

        // Wait for MAINCK frequency to stabilize
        while device.PMC.ckgr_mcfr().read().mainfrdy().bit_is_clear() {}

        // Read MAINCK frequency
        let main_f = device.PMC.ckgr_mcfr().read().mainf().bits() as u32;
        if main_f == 0 {
            defmt::error!("MAINCK NOT RUNNING");
            panic!()
        }
        defmt::info!(
            "MAINCK FREQUENCY: {} Hz ({} MAINCK CYCLES)",
            main_f * F_SLCK / 16,
            main_f
        );

        // Select MCK
        device.PMC.mckr().write(|w| w.css().main_clk());

        // Wait for MCK to be ready
        while device.PMC.sr().read().mckrdy().bit_is_clear() {}

        defmt::info!("MCK SET");
        device
    }

    fn init_gpio(device: pac::Peripherals) -> pac::Peripherals {
        // Enable peripheral clock
        device.PMC.pcer0().write(|w| {
            w.pid10().set_bit(); // PIOA
            w.pid18().set_bit() // PIOD
        });

        // Clear PIO interupts
        unsafe {
            pac::NVIC::unmask(pac::Interrupt::PIOA); // Enable PIO interrupt in the NVIC
            pac::NVIC::unmask(pac::Interrupt::PIOD);
            pac::NVIC::unpend(pac::Interrupt::PIOA); // Unpend PIOA interrupts
            pac::NVIC::unpend(pac::Interrupt::PIOD);
            (*pac::PIOA::PTR).isr().read().bits(); // Clear ISRs
            (*pac::PIOD::PTR).isr().read().bits();
        }

        // Enable pull-up resistor
        device.PIOA.puer().write(|w| {
            w.p9().set_bit();
            w.p19().set_bit()
        });

        // Enable input filter slow clock (debouncing filter)
        device.PIOA.ifscer().write(|w| {
            w.p9().set_bit();
            w.p19().set_bit()
        });

        // Set debounce duration
        device.PIOA.scdr().write(|w| unsafe { w.bits(0x50) });

        // Enable input filter
        device.PIOA.ifer().write(|w| {
            w.p9().set_bit();
            w.p19().set_bit()
        });

        // Edge event detection
        device.PIOA.esr().write(|w| {
            w.p9().set_bit();
            w.p19().set_bit()
        });

        // Enable single edge detection
        device.PIOA.aimer().write(|w| {
            w.p9().set_bit();
            w.p19().set_bit()
        });

        // Rising edge event detection
        device.PIOA.rehlsr().write(|w| w.p9().set_bit());

        // Low level detection
        device.PIOA.fellsr().write(|w| w.p19().set_bit());

        // Enable input interrupt
        device.PIOA.ier().write(|w| {
            w.p9().set_bit();
            w.p19().set_bit()
        });

        device
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            rtic::export::wfi();
        }
    }

    // Handle button press
    #[task(binds = PIOA, local = [banka], shared = [net])]
    fn button(ctx: button::Context) {
        let button::SharedResources { mut net, .. } = ctx.shared;

        match ctx.local.banka.isr().read().bits() {
            0x80000 => {
                // Ethernet PHY event
                net.lock(|n| n.processor.handle_mdio_interrupt());
            }
            0x200 => {
                // button
                //defmt::info!("RTT: {}ms", rtt.lock(|rtt| rtt.millis()));
            }
            _ => (),
        }

        //ctx.shared.active.lock(|a| *a = !*a);

        //if ctx.shared.active.lock(|a| *a) {
        //    unsafe {
        //        // Write to current SysTick counter value (resets value to zero)
        //        // Also clears SYST_CSR_COUNTFLAG
        //        (*pac::SYS_TICK::PTR)
        //            .cvr()
        //            .write(|w| w.current().bits(0x00));
        //        // Simultaneously write to enable SysTick counter and TICKINT interrupts
        //        // For some reason does not work if this is done with sequential writes
        //        (*pac::SYS_TICK::PTR)
        //            .csr()
        //            .modify(|r, w| w.bits(r.bits() | 0x03));
        //    }
        //} else {
        //    // Disable SysTick counter
        //    unsafe {
        //        (*pac::SYS_TICK::PTR)
        //            .csr()
        //            .write(|w| w.enable().clear_bit());
        //    }
        //}
    }

    //// Flash the LED rapidly using a Systick
    //#[task(shared = [led, tc_timer], priority = 1)]
    //async fn blinky(ctx: blinky::Context) {
    //    defmt::info!("blinky");
    //    (ctx.shared.tc_timer, ctx.shared.led).lock(|timer, led| {
    //        for _ in 0..8 {
    //            led.toggle().unwrap();
    //            timer.delay_ms(50);
    //        }
    //        _ = timer.cancel();
    //    })
    //}

    //#[task(binds = TC1)]
    //fn help(_ctx: help::Context) {
    //    defmt::info!("help!")
    //}
}
