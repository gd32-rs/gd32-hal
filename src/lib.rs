//! This library provides high-level access to STM32 peripherals.
//!
//! **Current family support**: F3, Others need your help
//!
//! Please see the [Readme](https://github.com/gd32-rs/gd32-hal/blob/main/README.md) for a detailed overview,


#![no_std]
// Some reg modifications are marked `unsafe` in some PAC crates, but not others.
// Disable these warnings.
#![allow(unused_unsafe)]
// The `doc_cfg` feature allows us to show functionality that is feature-gated on `docs.rs`.
// todo: Re-implement the doc_cfg feature and the relevant tags (From all modules that impl EH traits)
// todo oncoe this is in stable.
// #![feature(doc_cfg)]

// todo: H7B3 has too many changes in v14 PAC; not supporting at this time. (2021-10-07)

#[cfg(not(any(
    feature = "f303",
)))]
compile_error!("This crate requires an MCU-specifying feature to be enabled. eg `l552`.");

// Re-export of the [svd2rust](https://crates.io/crates/svd2rust) auto-generated API for
// stm32 peripherals.

#[cfg(feature = "f303")]
pub use gd32f3::gd32f303 as pac;

// todo: U5 once SVD is out.

// #[cfg(not(any(feature = "f301", feature = "f302")))]
// pub mod adc;

// bxCAN families: F3, F4, L4,
// fdCAN families: L5, U5, G4, H7
// H7 suppords fd and can_ccu. (What's that?)
// WB and WL?
#[cfg(all(
    feature = "can",
    not(any(feature = "f301", feature = "f401", feature = "f410", feature = "f411"))
))]
pub mod can;

pub mod clocks;
// todo: You could get CRC working on most of these with some effort.
// #[cfg(not(any(
//     feature = "f4",
//     feature = "g0",
//     feature = "g4",
//     feature = "l5",
//     feature = "wb",
//     feature = "wl"
// )))]
// pub mod crc;
// #[cfg(not(any(
//     feature = "f401",
//     feature = "f411",
//     feature = "f412",
//     feature = "wb",
//     feature = "g0"
// )))]
// // WB doesn't have a DAC. Some G0 variants do - add it! Most F4 variants have it, some don't
// pub mod dac;

#[cfg(not(any(
    feature = "f3",
    feature = "f4",
    feature = "l4x1",
    feature = "l4x2",
    feature = "l412",
    feature = "l4x3",
    feature = "l4x6",
    feature = "g0",
    feature = "g4",
    feature = "wb",
    feature = "wl",
    feature = "l5", // todo: L5 just doesn't have the same PAC layout; doesn't use clusters.
)))]
pub mod dfsdm;

// // todo: G0 missing many DMA registers like CCR?
// // todo: F4 needs some mods. So, only working on L4 and G4.
// // todo: L5 has a PAC bug on CCR registers past 1.
// // https://github.com/stm32-rs/stm32-rs/issues/551
// #[cfg(not(any(feature = "f4", feature = "l5")))]
// pub mod dma;

// // #[cfg(not(any(feature = "h747cm4", feature = "h747cm7")))]
// // PAC error on bank 2 accessor for H747cmx.
// pub mod flash;

// // todo: PAC doesn't yet support these newer H7 MCUs that use FMAC.
// // #[cfg(any(feature = "h723", feature = "h725", feature = "h733", feature = "h735"))]
// // todo: Also G4.
// // pub mod fmac;

pub mod gpio;

// // #[cfg(feature = "wb")]
// // pub mod bluetooth;
// // #[cfg(feature = "wb")]
// // pub mod tl_mbox; // Mailbox for communicating with the RF core.

// #[cfg(feature = "wb")]
// pub mod hsem;

// #[cfg(not(feature = "f4"))]
// pub mod i2c;
// #[cfg(feature = "f4")]
// pub mod i2c_f4;
// #[cfg(feature = "f4")]
// pub use i2c_f4 as i2c;

// #[cfg(feature = "wb")]
// pub mod ipcc;

// pub mod low_power;

// #[cfg(any(feature = "h747cm4", feature = "h747cm7"))]
// pub mod power;

// // F3, F4, L5, G0, and WL don't have Quad SPI.
// #[cfg(not(any(
// feature = "f3",
// feature = "f4",
// feature = "l4x3", // todo: PAC bug?
// feature = "l5",
// feature = "g0",
// feature = "g431",
// feature = "g441",
// feature = "g471",
// feature = "g491",
// feature = "g4a1",
// feature = "h7b3",
// feature = "wl",
// )))]
// pub mod qspi;

// // Note: Some F4 variants support RNG, but we haven't figured out the details yet. Send a PR if interested.
// #[cfg(not(any(
//     feature = "f3",
//     feature = "f4",
//     feature = "g030",
//     feature = "g031",
//     feature = "g070",
//     feature = "g071"
// )))]
// pub mod rng;

// pub mod rtc;

// #[cfg(not(any(
//     feature = "f3",
//     feature = "f4",
//     feature = "g0",
//     feature = "g4", // todo: G4 PAC issue re getting channel-specific reg blocks.
//     feature = "h7b3",
//     feature = "wl"
// )))]
// pub mod sai;

// pub mod spi;

// pub mod timer;
// pub mod usart;

// // See note at top of `usb` module for info on G0; not avail on modules the PAC has avail.
// cfg_if::cfg_if! {
//     if #[cfg(all(
//         feature = "usb",
//         all(
//             any(
//                 feature = "f303",
//                 feature = "l4x2",
//                 feature = "l412",
//                 feature = "l4x3",
//                 feature = "l5",
//                 feature = "g4",
//                 feature = "wb",
//             ),
//         not(feature = "g4a1"))
//     ))] {
//         pub mod usb;
//     } else if #[cfg(all(
//     // todo: I think only H7 has hs (high-speed), while F4 and L4 have FS only.
//         any(feature = "usbotg_fs", feature = "usbotg_hs"),
//         any(feature = "f4", feature = "l4x5", feature = "l4x6", feature = "h7")
//     ))] {
//         pub mod usb_otg;
//     }
// }

mod util;

// todo: should these helper macros be removed from this library? It has nothing to do with STM32.

/// Syntax helper for getting global variables of the form `Mutex<RefCell<Option>>>` from an interrupt-free
/// context - eg in interrupt handlers.
///
/// Example: `access_global!(DELAY, delay, cs)`
#[macro_export]
macro_rules! access_global {
    ($NAME_GLOBAL:ident, $name_local:ident, $cs:expr) => {
        let mut part1 = $NAME_GLOBAL.borrow($cs).borrow_mut();
        let $name_local = part1.as_mut().unwrap();
    };
}

/// Syntax helper for setting global variables of the form `Mutex<RefCell<Option>>>`.
/// eg in interrupt handlers. Ideal for non-copy-type variables that can't be initialized
/// immediatiately.
///
/// Example: `make_globals!(
///     (USB_SERIAL, SerialPort<UsbBusType>),
///     (DELAY, Delay),
/// )`
#[macro_export]
macro_rules! make_globals {
    ($(($NAME:ident, $type:ty)),+) => {
        $(
            static $NAME: Mutex<RefCell<Option<$type>>> = Mutex::new(RefCell::new(None));
        )+
    };
}

/// Syntax helper for setting global variables of the form `Mutex<Cell<>>>`.
/// eg in interrupt handlers. Ideal for copy-type variables.
///
/// Example: `make_simple_globals!(
///     (VALUE, f32, 2.),
///     (SETTING, Setting, Setting::A),
/// )`
#[macro_export]
macro_rules! make_simple_globals {
    ($(($NAME:ident, $type:ty, $val:expr)),+) => {
        $(
            static $NAME: Mutex<Cell<$type>> = Mutex::new(Cell::new($val));
        )+
    };
}

// todo: Remove this debug_workaroudn function on MCUs that don't require it. Ie, is this required on G4? G0?
use cortex_m::interrupt::free;

// #[cfg(not(any(feature = "g0")))]
// /// Workaround due to debugger disconnecting in WFI (and low-power) modes.
// /// This affects most (all?) STM32 devices. In production on battery-powered
// /// devices that don't use DMA, consider removing this, to prevent power
// /// use by the DMA clock.
// /// For why we enable the DMA clock, see STM32F446 errata, section 2.1.1.
// pub fn debug_workaround() {
//     free(|_| {
//         let dbgmcu = unsafe { &(*pac::DBGMCU::ptr()) };

//         cfg_if::cfg_if! {
//             if #[cfg(all(feature = "h7", not(any(feature = "h747cm4", feature = "h747cm7"))))] {
//                 dbgmcu.cr.modify(|_, w| w.dbgsleep_d1().set_bit());
//                 dbgmcu.cr.modify(|_, w| w.dbgstop_d1().set_bit());
//                 dbgmcu.cr.modify(|_, w| w.dbgstby_d1().set_bit());
//             } else if #[cfg(feature = "h7")] {
//                 dbgmcu.cr.modify(|_, w| w.dbgslpd1().set_bit());
//                 dbgmcu.cr.modify(|_, w| w.dbgstpd1().set_bit());
//                 dbgmcu.cr.modify(|_, w| w.dbgstbd1().set_bit());
//             } else {
//                 #[cfg(not(feature = "l5"))]
//                 dbgmcu.cr.modify(|_, w| w.dbg_sleep().set_bit());
//                 dbgmcu.cr.modify(|_, w| w.dbg_stop().set_bit());
//                 dbgmcu.cr.modify(|_, w| w.dbg_standby().set_bit());
//             }
//         }
//     });

//     free(|_| {
//         let rcc = unsafe { &(*pac::RCC::ptr()) };

//         // todo Some MCUs may need the dbgmcu lines, but not DMA enabled.
//         // todo: Remove this part on MCUs not affected. F4 and L4 are confirmed affected.

//         #[cfg(feature = "f3")]
//         rcc.ahbenr.modify(|_, w| w.dma1en().set_bit());

//         #[cfg(not(feature = "f3"))]
//         rcc.ahb1enr.modify(|_, w| w.dma1en().set_bit());
//     })
// }

/// In the prelude, we export helper macros.
pub mod prelude {
    pub use access_global;
    pub use make_globals;
    pub use make_simple_globals;
}
