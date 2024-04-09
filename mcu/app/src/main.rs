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
    use hal::{rtt::Rtt, *};
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
        //led: Pin<PA23, Output>,
    }

    #[local]
    struct Local {
        banka: pac::PIOA,
    }

    #[init()]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
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

        (
            Shared { active: true },
            Local {
                banka: ctx.device.PIOA,
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
    #[task(binds = PIOA, local = [banka], shared = [])]
    fn button(ctx: button::Context) {
        match ctx.local.banka.isr().read().bits() {
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
