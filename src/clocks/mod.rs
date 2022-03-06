//! This module contains clock configurations for various MCUs. They tend to be significantly
//! different from one another, so we've feature-gated these files, rather than
//! code within the files, to differentiate families.
//!
//! See the Reference Manuals for seeing what settings are available, and validating them.

cfg_if::cfg_if! {
    if #[cfg(any(feature = "f3", feature = "f4"))] {
        mod f;
        pub use f::*;
    }
}

// todo: Consider merging the modules into a single file: There's more similar than different.
// todo: You have a good deal of DRY atm between modules.

// Dat structures and functions that are shared between clock modules go here.

// todo: Continue working through DRY between the clock modules.

/// Speed out of limits.
#[derive(Debug)]
pub enum SpeedError {
    SysclkOutOfLimits,
    HclkOutOfLimits,
    Apb1OutOfLimits,
    Apb2OutOfLimits,
}


// #[derive(Clone, Copy)]
// #[repr(u8)]
// pub enum ClocksValid {
//     Valid,
//     NotValid,
// }
