#![no_std]
#![no_main]

// TODO: Improve testing
// run with: cargo test --test integration

use defmt_rtt as _;
use panic_probe as _;

#[cfg(test)]
#[defmt_test::tests]
mod tests {
    use core::cell::UnsafeCell;
    use defmt::assert;
    use hal_core::gmac::*;

    #[test]
    fn rx_buf_desc_owned() {
        let desc = RxBufDesc {
            data: UnsafeCell::new([0xFFFFFFFF, 0x0]),
        };
        assert!(
            desc.owned(),
            "assertion `owned == true` failed\n   desc: {:#x}",
            desc.word(0),
        );

        // Verify clear_owned() works
        desc.clear_owned();
        assert!(
            !desc.owned(),
            "assertion `owned == false` failed\n   desc: {:#x}",
            desc.word(0),
        );

        // Verify that other bits were not changed
        let w = 0xFFFFFFFE;
        assert!(
            desc.word(0) == w,
            "assertion `word[0] == {:#x}` failed\n   desc: {:#x}",
            w,
            desc.word(0),
        );
    }

    #[test]
    fn rx_buf_desc_wrap() {
        let desc = RxBufDesc {
            data: UnsafeCell::new([0xFFFFFFFD, 0x0]),
        };
        assert!(
            !desc.wrap(),
            "assertion `wrap == false` failed\n   desc: {:#x}",
            desc.word(0),
        );

        // Verify set_wrap() works
        desc.set_wrap(true);
        assert!(
            desc.wrap(),
            "assertion `wrap == true` failed\n   desc: {:#x}",
            desc.word(0),
        );

        // Verify that other bits were not changed
        let w = 0xFFFFFFFF;
        assert!(
            desc.word(0) == w,
            "assertion `word[0] == {:#x}` failed\n   desc: {:#x}",
            w,
            desc.word(0),
        );
    }

    #[test]
    fn tx_buf_desc_used() {
        let desc = TxBufDesc {
            data: UnsafeCell::new([0x0, 0xFFFFFFFF]),
        };
        assert!(
            desc.used(),
            "assertion `used == true` failed\n   desc: {:#x}",
            desc.word(1),
        );

        // Verify clear_used() works
        desc.clear_used();
        assert!(
            !desc.used(),
            "assertion `used == false` failed\n   desc: {:#x}",
            desc.word(1),
        );

        // Verify that other bits were not changed
        let w = 0x5FFFFFFF;
        assert!(
            desc.word(1) == w,
            "assertion `word[0] == {:#x}` failed\n   desc: {:#x}",
            w,
            desc.word(1),
        );
    }

    #[test]
    fn tx_buf_desc_wrap() {
        let desc = TxBufDesc {
            data: UnsafeCell::new([0x0, 0xBFFFFFFF]),
        };
        assert!(
            !desc.wrap(),
            "assertion `wrap == false` failed\n   desc: {:#x}",
            desc.word(1),
        );

        // Verify set_wrap() works
        desc.set_wrap(true);
        assert!(
            desc.wrap(),
            "assertion `wrap == true` failed\n   desc: {:#x}",
            desc.word(1),
        );

        // Verify that other bits were not changed
        let w = 0xFFFFFFFF;
        assert!(
            desc.word(1) == w,
            "assertion `word[0] == {:#x}` failed\n   desc: {:#x}",
            w,
            desc.word(1),
        );
    }
}
