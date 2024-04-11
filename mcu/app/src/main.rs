#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(
    device = samv71_pac,
    dispatchers = [IXC]
)]
mod app {
    use core::sync::atomic::{AtomicUsize, Ordering};
    use hal::uart;
    use hal::{
        dma::Dma,
        rtt::Rtt,
        uart::{Uart0, UartInterrupt},
    };
    use rtic_monotonics::systick::*;
    use samv71_pac as pac;

    const F_SLCK: u32 = 32768; // 32.768 kHz

    // Timestamp for DEFMT logging
    // TODO: Change to real timestamp
    static COUNT: AtomicUsize = AtomicUsize::new(0);
    defmt::timestamp!("{=usize}", COUNT.fetch_add(1, Ordering::Relaxed));

    #[shared]
    struct Shared {
        active: bool,
        uart: Uart0<2, 1>,
        //led: Pin<PA23, Output>,
    }

    #[local]
    struct Local {
        data: u8,
        banka: pac::PIOA,
    }

    #[init(local = [uart_storage: uart::Storage<2, 1> = uart::Storage::default()])]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        let init::LocalResources { uart_storage, .. } = ctx.local;

        // Init start
        defmt::info!("INIT");

        // Init device clocks
        ctx.device = init_clocks(ctx.device);

        // Init GPIO
        ctx.device = init_gpio(ctx.device);

        // Init UART
        ctx.device = init_uart(ctx.device);

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

        // DMA
        let dma = Dma::new(ctx.device.XDMAC);

        // UART
        let uart = Uart0::new(ctx.device.UART2, uart_storage, &dma);

        (
            Shared { active: true, uart },
            Local {
                banka: ctx.device.PIOA,
                data: 0u8,
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
            pac::NVIC::unpend(pac::Interrupt::PIOA); // Unpend PIOA interrupts
            (*pac::PIOA::PTR).isr().read().bits(); // Clear ISRs
        }

        // Enable pull-up resistor
        device.PIOA.puer().write(|w| w.p9().set_bit());

        // Enable input filter slow clock (debouncing filter)
        device.PIOA.ifscer().write(|w| w.p9().set_bit());

        // Set debounce duration
        device.PIOA.scdr().write(|w| unsafe { w.bits(0x50) });

        // Enable input filter
        device.PIOA.ifer().write(|w| w.p9().set_bit());

        // Edge event detection
        device.PIOA.esr().write(|w| w.p9().set_bit());

        // Enable single edge detection
        device.PIOA.aimer().write(|w| w.p9().set_bit());

        // Rising edge event detection
        device.PIOA.rehlsr().write(|w| w.p9().set_bit());

        // Enable input interrupt
        device.PIOA.ier().write(|w| w.p9().set_bit());

        // TODO: Set UART pin device modes
        device.PIOD.abcdsr(0).write(|w| {
            w.p25().clear_bit();
            w.p26().clear_bit()
        });
        device.PIOD.abcdsr(1).write(|w| {
            w.p25().set_bit();
            w.p26().set_bit()
        });
        device.PIOD.pdr().write(|w| {
            w.p25().set_bit();
            w.p26().set_bit()
        });

        device
    }

    fn init_uart(device: pac::Peripherals) -> pac::Peripherals {
        // Enable peripheral clock
        device.PMC.pcer1().write(|w| {
            w.pid44().set_bit() // UART2
        });

        unsafe {
            pac::NVIC::unmask(pac::Interrupt::UART2); // Enable PIO interrupt in the NVIC
        }

        device
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            rtic::export::wfi();
        }
    }

    #[task(binds = UART2, shared = [uart])]
    fn handle_uart(ctx: handle_uart::Context) {
        let handle_uart::SharedResources { mut uart, .. } = ctx.shared;

        let interrupts = uart.lock(|u| u.interrupts());

        for interrupt in interrupts {
            match interrupt {
                //UartInterrupt::CMP => defmt::info!("CMP"),
                //UartInterrupt::TXEMTPY => defmt::info!("TXEMPTY"),
                UartInterrupt::PARE => defmt::info!("PARE"),
                UartInterrupt::FRAME => defmt::info!("FRAME"),
                UartInterrupt::OVRE => defmt::info!("OVRE"),
                // UartInterrupt::TXRDY => defmt::info!("TXRDY"),
                // UartInterrupt::RXRDY => defmt::info!("RXRDY"),
                _ => (),
            }
        }
    }

    // Handle button press
    #[task(binds = PIOA, local = [banka, data], shared = [uart])]
    fn button(ctx: button::Context) {
        let button::LocalResources { banka, data, .. } = ctx.local;
        let button::SharedResources { mut uart, .. } = ctx.shared;

        match banka.isr().read().bits() {
            0x200 => {
                // button
                uart.lock(|uart| {
                    uart.write(*data);
                    defmt::info!("WRITE: {:#x}", data);
                    defmt::info!("READ: {:#x}", uart.read());
                    *data += 1;
                });
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
