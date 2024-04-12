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
        rtt::{Rtt, RttInterrupt},
        uart::{Uart0, UartInterrupt},
    };
    use rtic_monotonics::systick::*;
    use samv71_pac as pac;
    use you_must_enable_the_rt_feature_for_the_pac_in_your_cargo_toml::TWIHS0;

    const F_SLCK: u32 = 32768; // 32.768 kHz

    // Timestamp for DEFMT logging
    // TODO: Change to real timestamp
    static COUNT: AtomicUsize = AtomicUsize::new(0);
    defmt::timestamp!("{=usize}", COUNT.fetch_add(1, Ordering::Relaxed));

    #[shared]
    struct Shared {
        active: bool,
        uart: Uart0<2, 1>,
    }

    #[local]
    struct Local {
        data: u8,
        banka: pac::PIOA,
        rtt: Rtt,
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

        // Init I2C
        ctx.device = init_i2c(ctx.device);

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

        rtt.rtt.ar().write(|w| unsafe { w.almv().bits(5000) });

        (
            Shared { active: true, uart },
            Local {
                banka: ctx.device.PIOA,
                data: 0u8,
                rtt,
            },
        )
    }

    // Handle RTT interrupts
    #[task(binds = RTT, local = [rtt])]
    fn rtt_interrupt_handler(ctx: rtt_interrupt_handler::Context) {
        let rtt_interrupt_handler::LocalResources { mut rtt, .. } = ctx.local;

        for interrupt in rtt.interrupts() {
            match interrupt {
                RttInterrupt::RTTINC => (),
                RttInterrupt::ALMS => {
                    // Flash LED
                    let pioa = unsafe { &*pac::PIOA::PTR };
                    if pioa.odsr().read().p23().bit_is_clear() {
                        pioa.sodr().write(|w| w.p23().set_bit());
                    } else {
                        pioa.codr().write(|w| w.p23().set_bit());
                    }

                    // I2C
                    let i2c = unsafe { &*TWIHS0::PTR };
                    i2c.mmr().modify(|_, w| w.mread().set_bit());
                    i2c.iadr().write(|w| unsafe { w.iadr().bits(0x1A) });
                    i2c.cr().write(|w| w.start().set_bit());
                    while i2c.sr().read().rxrdy().bit_is_clear() {}
                    let l = i2c.rhr().read().rxdata().bits() as u16;
                    i2c.iadr().write(|w| unsafe { w.iadr().bits(0x1B) });
                    i2c.cr().write(|w| w.start().set_bit());
                    while i2c.sr().read().rxrdy().bit_is_clear() {}
                    let h = i2c.rhr().read().rxdata().bits() as u16;
                    let gyro: u16 = (h << 8) | l;
                    let gyro = if gyro & (1 << 15) != 0 {
                        // Negative number, invert and add 1
                        !(gyro ^ 0xFFFF) + 1
                    } else {
                        // Positive number, return as is
                        gyro
                    };
                    defmt::info!("I2C GYRO: {:#x}", gyro);

                    // Restart timer
                    rtt.rtt.mr().modify(|_, w| w.rttrst().set_bit());
                    rtt.rtt.ar().write(|w| unsafe { w.almv().bits(5000) });
                }
            }
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

        // LED
        device.PIOA.oer().write(|w| w.p23().set_bit());

        device
    }

    fn init_uart(device: pac::Peripherals) -> pac::Peripherals {
        // Enable peripheral clock
        device.PMC.pcer1().write(|w| {
            w.pid44().set_bit() // UART2
        });

        unsafe {
            pac::NVIC::unmask(pac::Interrupt::UART2); // Enable interrupt in the NVIC
        }

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

    fn init_i2c(device: pac::Peripherals) -> pac::Peripherals {
        // Enable peripheral clock
        device.PMC.pcer0().write(|w| {
            w.pid19().set_bit() // TWIHS0
        });

        unsafe {
            //pac::NVIC::unmask(pac::Interrupt::TWIHS0); // Enable interrupt in the NVIC
        }

        // Pull up res
        device.PIOA.puer().write(|w| w.p3().set_bit());
        device.PIOA.puer().write(|w| w.p4().set_bit());

        // Peripheral mode A
        device.PIOA.abcdsr(0).write(|w| {
            w.p3().clear_bit();
            w.p4().clear_bit()
        });
        device.PIOA.abcdsr(1).write(|w| {
            w.p3().clear_bit();
            w.p4().clear_bit()
        });
        device.PIOA.pdr().write(|w| {
            w.p3().set_bit();
            w.p4().set_bit()
        });

        let i2c = unsafe { &*TWIHS0::PTR };
        i2c.mmr().write(|w| {
            unsafe { w.dadr().bits(0x6B) };
            w.mread().clear_bit();
            w.iadrsz()._1_byte()
        });

        // Try to find a valid clock configuration. From §43.8.5 we
        // have
        //
        //    DIV * 2^CKDIV = (f_pid / f_twi / 2) - 3
        //
        // where DIV = CHDIV = CLDIV.
        //
        // We thus iterate over possible values of CKDIV and use the
        // first valid permutation of (CKIV, DIV), unless options are
        // exhausted.

        i2c.cwgr().write(|w| {
            unsafe { w.ckdiv().bits(6) };
            unsafe { w.chdiv().bits(1) };
            unsafe { w.cldiv().bits(1) }
        });

        i2c.cr().write(|w| {
            w.svdis().set_bit();
            w.msen().set_bit()
        });

        i2c.iadr().write(|w| unsafe { w.iadr().bits(0x10) });
        i2c.thr().write(|w| unsafe { w.bits(0b00100000) });
        i2c.cr().write(|w| w.stop().set_bit());
        while i2c.sr().read().txrdy().bit_is_clear() {}
        while i2c.sr().read().txcomp().bit_is_clear() {}

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
                defmt::info!("BUTTON");

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
