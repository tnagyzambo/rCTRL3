#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(
    device = pac,
    peripherals = true,
    dispatchers = [IXC]
)]
mod app {
    use core::sync::atomic::{AtomicUsize, Ordering};
    use hal::{
        board::{bmi088, SpiBus},
        core::{
            clock::*,
            pac,
            pio::*,
            rtt::{Rtt, RttInterrupt},
            spi::{Spi0, SpiInterrupt},
            uart::{Uart, Uart1},
        },
    };
    use postcard::to_slice_cobs;
    use rctrl_lib::SerialPacket;
    use rtic_monotonics::systick::*;

    const F_SLCK: u32 = 32768; // 32.768 kHz

    // Timestamp for DEFMT logging
    // TODO: Change to real timestamp
    static COUNT: AtomicUsize = AtomicUsize::new(0);

    defmt::timestamp!("{=usize}", COUNT.fetch_add(1, Ordering::Relaxed));

    #[shared]
    struct Shared {
        active: bool,
        uart: Uart<Uart1>,
        spi: SpiBus<Spi0>,
    }

    #[local]
    struct Local {
        led0: Pin<PA23, Output>,
        led1: Pin<PC9, Output>,
        button: Pin<PA9, Input>,
        banka: Bank<BankA>,
        rtt: Rtt,
    }

    #[init()]
    fn init(ctx: init::Context) -> (Shared, Local) {
        // Init start
        defmt::info!("INIT");

        // Set NVIC priorities
        let mut per = unsafe { pac::CorePeripherals::steal() };
        let nvic = &mut per.NVIC;
        unsafe { nvic.set_priority(pac::Interrupt::PIOA, 16) };
        unsafe { nvic.set_priority(pac::Interrupt::RTT, 32) };

        // Init device clocks
        let _slck = Slck::default();
        let mainck = Mainck::default().set_source_xtal_internal();
        let mck = Mck::<_, mck::Div<1>, mck::Div<1>>::from(mainck);

        // Init GPIO
        let banka = Bank::<BankA>::init();
        let _ = Bank::<BankB>::init();
        let _ = Bank::<BankC>::init();
        let _ = Bank::<BankD>::init();
        banka.set_debounce_period(0x50);
        banka.enable_interrupts();

        let led0 = Pin::<PA23, Output>::init();
        let led1 = Pin::<PC9, Output>::init();
        let button = Pin::<PA9, Input>::init();
        button.enable_pullup();
        button.enable_debounce_filter();
        button.enable_rising_edge_interrupt();

        // Init UART
        let rx = Pin::<PA5, PeripheralC>::init();
        let tx = Pin::<PA4, PeripheralC>::init();
        let uart = Uart::<Uart1>::init(rx, tx, mck, 57600);
        uart.enable_interrupts();

        // Init SPI
        let mut spi = SpiBus::<Spi0>::init();
        let mut gyro = spi.gyro();
        defmt::assert!(gyro.id_ok(), "Gyroscope ID match failed");
        gyro.set_range(bmi088::gyro::Range::_250DegSec);
        gyro.set_bw(bmi088::gyro::Bandwidth::Odr400Filter47);
        let mut accl = spi.accelerometer();
        accl.init_spi();
        accl.enable();
        defmt::assert!(accl.id_ok(), "Accelerometer ID match failed");

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
        rtt.rtt.ar().write(|w| unsafe { w.almv().bits(5000) });

        (
            Shared {
                active: true,
                uart,
                spi,
            },
            Local {
                led0,
                led1,
                button,
                banka,
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
    #[task(binds = RTT, local = [rtt, led0, uart_buf: [u8; 50] = [0u8; 50]], shared = [uart, spi])]
    fn rtt_interrupt_handler(ctx: rtt_interrupt_handler::Context) {
        let rtt_interrupt_handler::LocalResources {
            rtt,
            led0,
            uart_buf,
            ..
        } = ctx.local;
        let rtt_interrupt_handler::SharedResources {
            mut uart, mut spi, ..
        } = ctx.shared;

        for interrupt in rtt.interrupts() {
            match interrupt {
                RttInterrupt::RTTINC => (),
                RttInterrupt::ALMS => {
                    // Flash LED
                    led0.toggle();

                    let packet = spi.lock(|spi| {
                        let gyro = spi.gyro();
                        let gyro_x = gyro.x();
                        let gyro_y = gyro.y();
                        let gyro_z = gyro.z();

                        let accl = spi.accelerometer();
                        let accl_x = -accl.x();
                        let accl_y = -accl.y();
                        let accl_z = -accl.z();

                        SerialPacket {
                            accl_x,
                            accl_y,
                            accl_z,
                            gyro_x,
                            gyro_y,
                            gyro_z,
                        }
                    });

                    match to_slice_cobs(&packet, uart_buf) {
                        Ok(data) => {
                            // TODO: Replace with DMA transfer
                            for byte in data {
                                uart.lock(|uart| uart.write(*byte));
                            }
                        }
                        Err(e) => defmt::error!("Error encoding serial data: {}", e),
                    };

                    // Restart timer
                    rtt.rtt.mr().modify(|_, w| w.almien().clear_bit());
                    rtt.rtt.ar().write(|w| unsafe { w.almv().bits(500) });
                    rtt.rtt.mr().modify(|_, w| w.almien().set_bit());
                    rtt.rtt.mr().modify(|_, w| w.rttrst().set_bit());
                }
            }
        }
    }

    #[task(binds = SPI0, shared = [spi])]
    fn handle_spi(ctx: handle_spi::Context) {
        let handle_spi::SharedResources { mut spi, .. } = ctx.shared;

        let interrupts = spi.lock(|s| s.spi.interrupts());

        for interrupt in interrupts {
            match interrupt {
                SpiInterrupt::UNDES => defmt::info!("SPI_UNDES"),
                SpiInterrupt::TXEMPTY => defmt::info!("SPI_TXEMPTY"),
                SpiInterrupt::NSSR => defmt::info!("SPI_NSSR"),
                SpiInterrupt::OVRES => defmt::info!("SPI_OVRES"),
                SpiInterrupt::MODF => defmt::info!("SPI_MODF"),
                SpiInterrupt::TDRE => defmt::info!("SPI_TDRE"),
                SpiInterrupt::RDRF => defmt::info!("SPI_RDRF"),
            }
        }
    }

    //#[task(binds = UART2, shared = [uart])]
    //fn handle_uart(ctx: handle_uart::Context) {
    //    let handle_uart::SharedResources { mut uart, .. } = ctx.shared;

    //    let interrupts = uart.lock(|u| u.interrupts());

    //    for interrupt in interrupts {
    //        match interrupt {
    //            //UartInterrupt::CMP => defmt::info!("CMP"),
    //            //UartInterrupt::TXEMTPY => defmt::info!("TXEMPTY"),
    //            UartInterrupt::PARE => defmt::info!("PARE"),
    //            UartInterrupt::FRAME => defmt::info!("FRAME"),
    //            UartInterrupt::OVRE => defmt::info!("OVRE"),
    //            // UartInterrupt::TXRDY => defmt::info!("TXRDY"),
    //            // UartInterrupt::RXRDY => defmt::info!("RXRDY"),
    //            _ => (),
    //        }
    //    }
    //}

    // Handle button press
    #[task(binds = PIOA, local = [button, banka, led1] )]
    fn button(ctx: button::Context) {
        let button::LocalResources { banka, led1, .. } = ctx.local;

        for interrupt in banka.interrupts() {
            match interrupt {
                PioInterrupt::P9 => {
                    // Button
                    led1.toggle();
                }
                _ => (),
            }
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
