#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(
    device = hal::pac,
    peripherals = true,
    dispatchers = [IXC]
)]
mod app {
    use core::sync::atomic::{AtomicUsize, Ordering};
    use hal::pio::{PeripheralA, PeripheralC};
    use hal::uart;
    use hal::{
        clock::*,
        i2c::*,
        pio::*,
        rtt::{Rtt, RttInterrupt},
        uart::{Uart, Uart2, UartInterrupt},
    };
    use rtic_monotonics::systick::*;

    const F_SLCK: u32 = 32768; // 32.768 kHz

    // Timestamp for DEFMT logging
    // TODO: Change to real timestamp
    static COUNT: AtomicUsize = AtomicUsize::new(0);
    defmt::timestamp!("{=usize}", COUNT.fetch_add(1, Ordering::Relaxed));

    #[shared]
    struct Shared {
        active: bool,
        uart: Uart<Uart2>,
    }

    #[local]
    struct Local {
        i2c: I2C<I2C0, Host>,
        led: Pin<PA23, Output>,
        button: Pin<PA9, Input>,
        data: u8,
        banka: Bank<BankA>,
        rtt: Rtt,
    }

    #[init()]
    fn init(mut ctx: init::Context) -> (Shared, Local) {
        // Init start
        defmt::info!("INIT");

        // Init device clocks
        let slck = Slck::default();
        let mainck = Mainck::default().set_source_xtal_internal();
        let mck = Mck::<_, mck::Div<1>, mck::Div<1>>::from(mainck);

        // Init GPIO
        let banka = Bank::<BankA>::init();
        banka.set_debounce_period(0x50);
        banka.enable_interrupts();

        let led = Pin::<PA23, Output>::init();
        let button = Pin::<PA9, Input>::init();
        button.enable_pullup();
        button.enable_debounce_filter();
        button.enable_rising_edge_interrupt();

        // Init UART
        let rx = Pin::<PD25, PeripheralC>::init();
        let tx = Pin::<PD26, PeripheralC>::init();
        let uart = Uart::<Uart2>::init(rx, tx, mck, 10000);
        uart.enable_interrupts();

        // Init I2C
        let sda = Pin::<PA3, PeripheralA>::init();
        let scl = Pin::<PA4, PeripheralA>::init();
        let mut i2c = I2C::<I2C0, Host>::init(sda, scl);
        let accl = i2c.as_device::<LSM9DS1Accelerometer<_>>(0x6B);
        accl.gyro_enable();

        // TODO: Move SysTick cofig to function
        // Create SysTick monotonic
        let systick_mono_token = rtic_monotonics::create_systick_token!();
        Systick::start(ctx.core.SYST, 12_000_000, systick_mono_token);

        // Disable SysTick counter
        // Otherwise SysTick will keep system from sleeping
        unsafe {
            (*hal::pac::SYS_TICK::PTR)
                .csr()
                .write(|w| w.enable().clear_bit())
        };

        // Init RTT
        let rtt = Rtt::new(ctx.device.RTT, F_SLCK, 3);
        rtt.rtt.ar().write(|w| unsafe { w.almv().bits(5000) });

        // Set NVIC priorities
        let mut per = unsafe { hal::pac::CorePeripherals::steal() };
        let nvic = &mut per.NVIC;
        unsafe { hal::pac::NVIC::set_priority(nvic, hal::pac::Interrupt::PIOA, 1) };
        unsafe { hal::pac::NVIC::set_priority(nvic, hal::pac::Interrupt::RTT, 2) };

        (
            Shared { active: true, uart },
            Local {
                i2c,
                led,
                button,
                banka,
                data: 0u8,
                rtt,
            },
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            rtic::export::wfi();
        }
    }

    // Handle RTT interrupts
    #[task(binds = RTT, local = [rtt, led, i2c])]
    fn rtt_interrupt_handler(ctx: rtt_interrupt_handler::Context) {
        let rtt_interrupt_handler::LocalResources { rtt, led, i2c, .. } = ctx.local;

        for interrupt in rtt.interrupts() {
            match interrupt {
                RttInterrupt::RTTINC => (),
                RttInterrupt::ALMS => {
                    // Flash LED
                    led.toggle();

                    // I2C
                    let accl = i2c.as_device::<LSM9DS1Accelerometer<_>>(0x6B);
                    defmt::info!(
                        "X: {}deg/s, Y: {}deg/s, Z: {}deg/s",
                        accl.gyro_x(),
                        accl.gyro_y(),
                        accl.gyro_z()
                    );

                    // Restart timer
                    rtt.rtt.mr().modify(|_, w| w.rttrst().set_bit());
                    rtt.rtt.ar().write(|w| unsafe { w.almv().bits(2500) });
                }
            }
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
    #[task(binds = PIOA, local = [button, banka, data], shared = [uart])]
    fn button(ctx: button::Context) {
        let button::LocalResources { banka, data, .. } = ctx.local;
        let button::SharedResources { mut uart, .. } = ctx.shared;

        match banka.interrupts() {
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
}
