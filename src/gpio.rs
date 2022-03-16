//! This module provides functionality for General Purpose Input and Output (GPIO) pins,
//! including all GPIOx register functions. It also configures GPIO interrupts using SYSCFG and EXTI
//! registers as appropriate.

// todo: WL is missing port C here due to some pins being missing, and this being tough
// todo to change with our current model. Note sure if PAC, or MCU limitation
// todo: WL is also missing interrupt support.

#[cfg(feature = "embedded-hal")]
use core::convert::Infallible;

use cortex_m::interrupt::free;

use crate::{
    pac::{self, RCU},
    rcu_en_reset, // todo?
};

#[cfg(feature = "embedded-hal")]
use embedded_hal::digital::v2::{InputPin, OutputPin, ToggleableOutputPin};

use cfg_if::cfg_if;
use paste::paste;

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for `GPIOx_MODER`
pub enum PinMode {
    Input,
    Output,
    Alt(u8),
    Analog,
}

impl PinMode {
    /// We use this function to find the value bits due to being unable to repr(u8) with
    /// the wrapped `AltFn` value.
    fn val(&self) -> u8 {
        match self {
            Self::Input => 0b01_00,  // default to float input
            Self::Output => 0b00_11, // default to PushPull 50MHz
            Self::Alt(_) => 0b10_11, // default to PushPull 50MHz
            Self::Analog => 0b00_00,
        }
    }
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for `GPIOx_OTYPER`
pub enum OutputType {
    PushPull = 0,
    OpenDrain = 1,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for `GPIOx_OSPEEDR`. This configures I/O output speed. See the user manual
/// for your MCU for what speeds these are. Note that Fast speed (0b10) is not
/// available on all STM32 families.
pub enum OutputSpeed {
    Low = 0b00,
    Medium = 0b01,
    High = 0b11, 
    VeryHigh = 0b111,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for `GPIOx_PUPDR`
pub enum Pull {
    Floating = 0b00,
    Up = 0b01,
    Dn = 0b10,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for `GPIOx_ISTAT` and `GPIOx_ODR`.
pub enum PinState {
    High = 1,
    Low = 0,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for `GPIOx_LCKR`.
pub enum CfgLock {
    NotLocked = 0,
    Locked = 1,
}

#[derive(Copy, Clone)]
#[repr(u8)]
/// Values for `GPIOx_BRR`.
pub enum ResetState {
    NoAction = 0,
    Reset = 1,
}

// todo: If you get rid of Port struct, rename this enum Port
#[derive(Copy, Clone)]
/// GPIO port letter
pub enum Port {
    A,
    B,
    C,
    D,
    E,
    F,
    G,
}

impl Port {
    /// See F303 RM section 12.1.3: each reg has an associated value
    fn cr_val(&self) -> u8 {
        match self {
            Self::A => 0,
            Self::B => 1,
            Self::C => 2,
            Self::D => 3,
            Self::E => 4,
            Self::F => 5,
            Self::G => 6,
        }
    }
}

#[derive(Copy, Clone, Debug)]
/// The pulse edge used to trigger interrupts.
pub enum Edge {
    Rising,
    Falling,
    Edge,
}

// These macros are used to interate over pin number, for use with PAC fields.
macro_rules! set_field {
    ($regs: expr, $pin:expr, $reg:ident, $field_bit_val:tt, [$($num:expr),+]) => {
        paste! {
            unsafe {
                match $pin {
                    $(
                        $num => (*$regs).$reg.modify(|_, w| set_field!(@expand_fields_op w, $field_bit_val, $num)),
                    )+
                    _ => panic!("GPIO pins must be 0 - 15."),
                }
            }
        }
    };

    (@expand_fields_op $w:ident, [$(($field:ident, $bit:ident, $val:expr)),+] , $num:expr) => {
        paste!{
            $w.$([<$field $num>]().$bit($val)).+
        }
    };
}


macro_rules! set_mode_and_ctl_field {
    ($regs: expr, $pin:expr, $val:expr, [$(($num:expr, $regid: expr)),+]) => {
        paste! {
            unsafe {
                match $pin {
                    $(
                        $num => {
                            (*$regs).[<ctl $regid>].modify(|_, w| w.[<md $num>]().bits($val));
                            (*$regs).[<ctl $regid>].modify(|_, w| w.[<ctl $num>]().bits($val >> 2));
                        },
                    )+
                    _ => panic!("GPIO pins must be 0 - 15."),
                }
            }
        }
    }
}

macro_rules! set_mode_or_ctl_field {
    ($regs: expr, $pin:expr, $field:expr, $val:expr, [$(($num:expr, $regid: expr)),+]) => {
        paste! {
            unsafe {
                match $pin {
                    $(
                        $num => {
                            (*$regs).[<ctl $regid>].modify(|_, w| w.[<$field $num>]().bits($val));
                        },
                    )+
                    _ => panic!("GPIO pins must be 0 - 15."),
                }
            }
        }
    }
}

macro_rules! set_alt {
    ($regs: expr, $pin:expr, $field_af:ident, $val:expr, [$(($num:expr, $zo:ident)),+]) => {
        paste! {
            unsafe {
                match $pin {
                    $(
                        $num => {
                            (*$regs).[<ctl $zo>].modify(|_, w| w.[<moder $num>]().bits(PinMode::Alt(0).val()));
                        }
                    )+
                    _ => panic!("GPIO pins must be 0 - 15."),
                }
            }
        }
    }
}

macro_rules! get_input_data {
    ($regs: expr, $pin:expr, [$($num:expr),+]) => {
        paste! {
            unsafe {
                match $pin {
                    $(
                        $num => (*$regs).istat.read().[<istat $num>]().bit_is_set(),
                    )+
                    _ => panic!("GPIO pins must be 0 - 15."),
                }
            }
        }
    }
}

macro_rules! set_state {
    ($regs: expr, $pin:expr, $offset: expr, [$($num:expr),+]) => {
        paste! {
            unsafe {
                match $pin {
                    $(
                        $num => (*$regs).bop.write(|w| w.bits(1 << ($offset + $num))),
                    )+
                    _ => panic!("GPIO pins must be 0 - 15."),
                }
            }
        }
    }
}

// todo: Consolidate these exti macros

// Reduce DRY for setting up interrupts.
macro_rules! set_exti {
    ($pin:expr, $trigger:expr, $val:expr, [$(($num:expr, $crnum:expr)),+]) => {
        let exti = unsafe { &(*pac::EXTI::ptr()) };
        let afio = unsafe { &(*pac::AFIO::ptr() )};
        paste! {
            match $pin {
                $(
                    $num => {
                    // todo: Core 2 interrupts for wb. (?)
                        cfg_if! {
                            if #[cfg(feature = "f3")] {
                                exti.inten.modify(|_, w| w.[<inten $num>]().set_bit());
                            }
                        }

                        cfg_if! {
                            if #[cfg(feature = "f3")] {
                                exti.rten.modify(|_, w| w.[<rten $num>]().bit($trigger.0));
                                exti.ften.modify(|_, w| w.[<ften $num>]().bit(!$trigger.1));
                            }
                        }
                        afio
                            .[<extiss $crnum>]
                            .modify(|_, w| unsafe { w.[<exti $num _ss>]().bits($val) });
                    }
                )+
                _ => panic!("GPIO pins must be 0 - 15."),
            }
        }
    }
}


/// Represents a single GPIO pin. Allows configuration, and reading/setting state.
pub struct Pin {
    /// The GPIO Port letter. Eg A, B, C.
    pub port: Port,
    /// The pin number: 1 - 15.
    pub pin: u8,
}

impl Pin {
    /// Internal function to get the appropriate GPIO block pointer.
    const fn regs(&self) -> *const pac::gpioa::RegisterBlock {
        // Note that we use this `const` fn and pointer casting since not all ports actually
        // deref to GPIOA in PAC.
        regs(self.port)
    }

    /// Create a new pin, with a specific mode. Enables the RCU peripheral clock to the port,
    /// if not already enabled. Example: `let pa1 = Pin::new(Port::A, 1);`
    pub fn new(port: Port, pin: u8, mode: PinMode) -> Self {
        assert!(pin <= 15, "Pin must be 0 - 15.");

        free(|_| {
            let rcu = unsafe { &(*RCU::ptr()) };

            match port {
                Port::A => {
                    cfg_if! {
                        if #[cfg(feature = "f3")] {
                            if rcu.apb2en.read().paen().bit_is_clear() {
                                rcu_en_reset!(apb2, pa, rcu);
                            }
                        }
                    }
                }
                Port::B => {
                    cfg_if! {
                        if #[cfg(feature = "f3")] {
                            if rcu.apb2en.read().pben().bit_is_clear() {
                                rcu_en_reset!(apb2, pb, rcu);
                            }
                        }
                    }
                }
                #[cfg(not(feature = "wl"))]
                Port::C => {
                    cfg_if! {
                        if #[cfg(feature = "f3")] {
                            if rcu.apb2en.read().pcen().bit_is_clear() {
                                rcu_en_reset!(apb2, pc, rcu);
                            }
                        }
                    }
                }
                #[cfg(not(any(feature = "f410", feature = "wl")))]
                Port::D => {
                    cfg_if! {
                        if #[cfg(feature = "f3")] {
                            if rcu.apb2en.read().pden().bit_is_clear() {
                                rcu_en_reset!(apb2, pd, rcu);
                            }
                        }
                    }
                }
                Port::E => {
                    cfg_if! {
                        if #[cfg(feature = "f3")] {
                            if rcu.apb2en.read().peen().bit_is_clear() {
                                rcu_en_reset!(apb2, pe, rcu);
                            }
                        }
                    }
                }
                Port::F => {
                    cfg_if! {
                        if #[cfg(feature = "f3")] {
                            if rcu.apb2en.read().pfen().bit_is_clear() {
                                rcu_en_reset!(apb2, pf, rcu);
                            }
                        }
                    }
                }
                Port::G => {
                    cfg_if! {
                        if #[cfg(feature = "f3")] {
                            if rcu.apb2en.read().pgen().bit_is_clear() {
                                rcu_en_reset!(apb2, pg, rcu);
                            }
                        }
                    }
                }
            }
        });

        let mut result = Self { port, pin };
        result.mode(mode);

        result
    }

    /// Set pin mode. Eg, Output, Input, Analog, or Alt. Sets the `MODER` register.
    pub fn mode(&mut self, value: PinMode) {
        set_mode_and_ctl_field!(
            self.regs(),
            self.pin,
            value.val(),
            [(0,0), (1,0), (2,0), (3,0), (4,0), (5,0), (6,0), (7,0), (8,1), (9,1), (10,1), (11,1), (12,1), (13,1), (14,1), (15,1)]
        );

        #[cfg(any(feature = "f310", feature = "f350", feature = "f370"))]
        {
            if let PinMode::Alt(alt) = value {
                self.alt_fn(alt);
            }
        }
    }

    /// Set output type. Sets the `OTYPER` register.
    pub fn output_type(&mut self, value: OutputType, mode:PinMode) {
        let value = match (value, mode) {
            (OutputType::PushPull, PinMode::Output) => 00,
            (OutputType::OpenDrain, PinMode::Output) => 01,
            (OutputType::PushPull, PinMode::Alt(_)) => 10,
            (OutputType::OpenDrain, PinMode::Alt(_)) => 11,
            _ => {panic!("not supported")},
        };
        set_mode_or_ctl_field!(
            self.regs(),
            self.pin,
            ctl,
            value as u8,
            [(0,0), (1,0), (2,0), (3,0), (4,0), (5,0), (6,0), (7,0), (8,1), (9,1), (10,1), (11,1), (12,1), (13,1), (14,1), (15,1)]
        );
    }

    /// Set output speed to Low, Medium, or High. Sets the `OSPEEDR` register.
    pub fn output_speed(&mut self, value: OutputSpeed) {

        let spd_val = if let OutputSpeed::VeryHigh = value {
            true
        } else {
            false
        };

        let value = match value {
            OutputSpeed::Low => 0b10,
            OutputSpeed::Medium => 0b01,
            OutputSpeed::High => 0b11,
            OutputSpeed::VeryHigh => 0b11,
        };

        set_mode_or_ctl_field!(
            self.regs(),
            self.pin,
            md,
            value as u8,
            [(0,0), (1,0), (2,0), (3,0), (4,0), (5,0), (6,0), (7,0), (8,1), (9,1), (10,1), (11,1), (12,1), (13,1), (14,1), (15,1)]
        );

        set_field!(
            self.regs(),
            self.pin,
            spd,
            [(spd,bit,spd_val)],
            [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        );

        
    }

    /// Set internal pull resistor: Pull up, pull down, or floating. Sets the `PUPDR` register.
    pub fn pull(&mut self, value: Pull) {
        if let Pull::Floating = value {
            set_mode_or_ctl_field!(
                self.regs(),
                self.pin,
                ctl,
                0b01,
                [(0,0), (1,0), (2,0), (3,0), (4,0), (5,0), (6,0), (7,0), (8,1), (9,1), (10,1), (11,1), (12,1), (13,1), (14,1), (15,1)]
            );
        } else {
            let v = if let Pull::Up = value {
                true
            } else {
                false
            };

            set_mode_or_ctl_field!(
                self.regs(),
                self.pin,
                ctl,
                0b10,
                [(0,0), (1,0), (2,0), (3,0), (4,0), (5,0), (6,0), (7,0), (8,1), (9,1), (10,1), (11,1), (12,1), (13,1), (14,1), (15,1)]
            );

            set_field!(
                self.regs(),
                self.pin,
                octl,
                [(octl,bit,v)],
                [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
            );
        }
        
    }

    /// Lock or unlock a port configuration. Sets the `LCKR` register.
    pub fn cfg_lock(&mut self, value: CfgLock) {
        set_field!(
            self.regs(),
            self.pin,
            lock,
            [(lk,
            bit,
            value as u8 != 0)],
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        );
    }

    /// Read the input data register. Eg determine if the pin is high or low. See also `is_high()`
    /// and `is_low()`. Reads from the `ISTAT` register.
    pub fn get_state(&mut self) -> PinState {
        let val = get_input_data!(
            self.regs(),
            self.pin,
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        );
        if val {
            PinState::High
        } else {
            PinState::Low
        }
    }

    /// Set a pin state (ie set high or low output voltage level). See also `set_high()` and
    /// `set_low()`. Sets the `BSRR` register. Atomic.
    pub fn set_state(&mut self, value: PinState) {
        let offset = match value {
            PinState::Low => 16,
            PinState::High => 0,
        };

        set_state!(
            self.regs(),
            self.pin,
            offset,
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        );
    }

    /// Set up a pin's alternate function. We set this up initially using `mode()`.
    /// TODO and Note: GD32F30x and GD32F3x0 has different ways to handle alternative functions, GD32F3x0 use AF Registers and each pin has a 4-bit config field, while GD32F30x use remap
    /// So, this function is keep here for GD32F3x0 series and similar devices.
    #[cfg(any(feature = "f310", feature = "f350", feature = "f370"))]
    fn alt_fn(&mut self, value: u8) {
        assert!(value <= 15, "Alt function must be 0 to 15.");

        cfg_if! {
            if #[cfg(any(feature = "f3"))] {
                // TODO
            }
        }
    }

    #[cfg(any(feature = "f303"))]
    /// Configure this pin as an interrupt source. Set the edge as Rising or Falling.
    pub fn enable_interrupt(&mut self, edge: Edge) {
        let rise_trigger = match edge {
            Edge::Rising => {
                // configure EXTI line to trigger on rising edge, disable trigger on falling edge.
                (true,false)
            }
            Edge::Falling => {
                // configure EXTI line to trigger on falling edge, disable trigger on rising edge.
                (false, true)
            }
            Edge::Edge => {
                (true, true)
            }
        };

        cfg_if! {
            if #[cfg(feature = "f3")] {
                set_exti!(self.pin, rise_trigger, self.port.cr_val(), [(0, 0), (1, 0), (2, 0),
                    (3, 0), (4, 1), (5, 1), (6, 1), (7, 1), (8, 2), (9, 2), (10, 2), (11, 2), (12, 3),
                    (13, 3), (14, 3), (15, 3)]
                );
            }
        }
    }

    /// Check if the pin's input voltage is high. Reads from the `ISTAT` register.
    pub fn is_high(&self) -> bool {
        get_input_data!(
            self.regs(),
            self.pin,
            [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
        )
    }

    /// Check if the pin's input voltage is low. Reads from the `ISTAT` register.
    pub fn is_low(&self) -> bool {
        !self.is_high()
    }

    /// Set the pin's output voltage to high. Sets the `BSRR` register. Atomic.
    pub fn set_high(&mut self) {
        self.set_state(PinState::High);
    }

    /// Set the pin's output voltage to low. Sets the `BSRR` register. Atomic.
    pub fn set_low(&mut self) {
        self.set_state(PinState::Low);
    }
}
//
#[cfg(feature = "embedded-hal")]
// #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
impl InputPin for Pin {
    type Error = Infallible;

    fn is_high(&self) -> Result<bool, Self::Error> {
        Ok(Pin::is_high(self))
    }

    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(Pin::is_low(self))
    }
}

#[cfg(feature = "embedded-hal")]
// #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
impl OutputPin for Pin {
    type Error = Infallible;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        Pin::set_low(self);
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Pin::set_high(self);
        Ok(())
    }
}

#[cfg(feature = "embedded-hal")]
// #[cfg_attr(docsrs, doc(cfg(feature = "embedded-hal")))]
impl ToggleableOutputPin for Pin {
    type Error = Infallible;

    fn toggle(&mut self) -> Result<(), Self::Error> {
        if self.is_high() {
            Pin::set_low(self);
        } else {
            Pin::set_high(self);
        }
        Ok(())
    }
}

/// Check if a pin's input voltage is high. Reads from the `ISTAT` register.
/// Does not require a `Pin` struct.
pub fn is_high(port: Port, pin: u8) -> bool {
    get_input_data!(
        regs(port),
        pin,
        [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
    )
}

/// Check if a pin's input voltage is low. Reads from the `ISTAT` register.
/// Does not require a `Pin` struct.
pub fn is_low(port: Port, pin: u8) -> bool {
    !is_high(port, pin)
}

/// Set a pin's output voltage to high. Sets the `BSRR` register. Atomic.
/// Does not require a `Pin` struct.
pub fn set_high(port: Port, pin: u8) {
    set_state(port, pin, PinState::High);
}

/// Set a pin's output voltage to low. Sets the `BSRR` register. Atomic.
/// Does not require a `Pin` struct.
pub fn set_low(port: Port, pin: u8) {
    set_state(port, pin, PinState::Low);
}

/// Set a pin state (ie set high or low output voltage level). See also `set_high()` and
/// `set_low()`. Sets the `BSRR` register. Atomic.
/// Does not require a `Pin` struct.
fn set_state(port: Port, pin: u8, value: PinState) {
    let offset = match value {
        PinState::Low => 16,
        PinState::High => 0,
    };

    set_state!(
        regs(port),
        pin,
        offset,
        [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]
    );
}

const fn regs(port: Port) -> *const pac::gpioa::RegisterBlock {
    // Note that we use this `const` fn and pointer casting since not all ports actually
    // deref to GPIOA in PAC.
    match port {
        Port::A => crate::pac::GPIOA::ptr(),
        Port::B => crate::pac::GPIOB::ptr() as _,
        Port::C => crate::pac::GPIOC::ptr() as _,
        Port::D => crate::pac::GPIOD::ptr() as _,
        Port::E => crate::pac::GPIOE::ptr() as _,
        Port::F => crate::pac::GPIOF::ptr() as _,
        Port::G => crate::pac::GPIOG::ptr() as _,
    }
}
