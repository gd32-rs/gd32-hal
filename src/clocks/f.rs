use crate::{
    clocks::SpeedError,
    pac::RCU,
    rcc_en_reset,
};

use cfg_if::cfg_if;

cfg_if! {
   if #[cfg(feature = "f3")] {
       #[derive(Clone, Copy)]
        /// The clocks source input used by the PLL.
        /// Note that this corresponds to Bits 16:15: Applicable only to some models,
        ///303xB/C etc use only bit 16, with bit 15 at reset value (0?) but it's equiv. 303xD/E and xE use bits 16:15.
        pub enum PllSrc {
            IRC8MDiv2,
            IRC48M,
            HXTAL(u32),      // Freq in Hz
        }

   }
}

#[derive(Clone, Copy)]
pub enum InputSrc {
    IRC8M,
    HXTAL(u32), // freq in Mhz
    Pll(PllSrc),
}

impl InputSrc {
    /// Required due to numerical value on non-uniform discrim being experimental.
    /// (ie, can't set on `Pll(Pllsrc)`.
    pub fn bits(&self) -> u8 {
        match self {
            Self::IRC8M => 0b00,
            Self::HXTAL(_) => 0b01,
            Self::Pll(_) => 0b10,
        }
    }
}

#[cfg(feature = "f3")]
#[derive(Clone, Copy)]
#[repr(u8)]
/// RCC_cfgr2. Scales the input source before the PLL.
pub enum Prediv {
    Div1 = 0b0000,
    Div2 = 0b0001,
}

#[cfg(feature = "f3")]
impl Prediv {
    pub fn value(&self) -> u8 {
        match self {
            Self::Div1 => 1,
            Self::Div2 => 2,
        }
    }
}

#[cfg(feature = "f3")]
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum PllMF {
    MF2  = 0x00 | 0x00,
    MF3  = 0x00 | 0x01,
    MF4  = 0x00 | 0x02,
    MF5  = 0x00 | 0x03,
    MF6  = 0x00 | 0x04,
    MF7  = 0x00 | 0x05,
    MF8  = 0x00 | 0x06,
    MF9  = 0x00 | 0x07,
    MF10 = 0x00 | 0x08,
    MF11 = 0x00 | 0x09,
    MF12 = 0x00 | 0x0A,
    MF13 = 0x00 | 0x0B,
    MF14 = 0x00 | 0x0C,
    MF15 = 0x00 | 0x0D,
    MF16 = 0x00 | 0x0E,

    MF17 = 0x10 | 0x00,
    MF18 = 0x10 | 0x01,
    MF19 = 0x10 | 0x02,
    MF20 = 0x10 | 0x03,
    MF21 = 0x10 | 0x04,
    MF22 = 0x10 | 0x05,
    MF23 = 0x10 | 0x06,
    MF24 = 0x10 | 0x07,
    MF25 = 0x10 | 0x08,
    MF26 = 0x10 | 0x09,
    MF27 = 0x10 | 0x0A,
    MF28 = 0x10 | 0x0B,
    MF29 = 0x10 | 0x0C,
    MF30 = 0x10 | 0x0D,
    MF31 = 0x10 | 0x0E,
    MF32 = 0x10 | 0x0F,

    MF33 = 0x20 | 0x00,
    MF34 = 0x20 | 0x01,
    MF35 = 0x20 | 0x02,
    MF36 = 0x20 | 0x03,
    MF37 = 0x20 | 0x04,
    MF38 = 0x20 | 0x05,
    MF39 = 0x20 | 0x06,
    MF40 = 0x20 | 0x07,
    MF41 = 0x20 | 0x08,
    MF42 = 0x20 | 0x09,
    MF43 = 0x20 | 0x0A,
    MF44 = 0x20 | 0x0B,
    MF45 = 0x20 | 0x0C,
    MF46 = 0x20 | 0x0D,
    MF47 = 0x20 | 0x0E,
    MF48 = 0x20 | 0x0F,

    MF49 = 0x30 | 0x00,
    MF50 = 0x30 | 0x01,
    MF51 = 0x30 | 0x02,
    MF52 = 0x30 | 0x03,
    MF53 = 0x30 | 0x04,
    MF54 = 0x30 | 0x05,
    MF55 = 0x30 | 0x06,
    MF56 = 0x30 | 0x07,
    MF57 = 0x30 | 0x08,
    MF58 = 0x30 | 0x09,
    MF59 = 0x30 | 0x0A,
    MF60 = 0x30 | 0x0B,
    MF61 = 0x30 | 0x0C,
    MF62 = 0x30 | 0x0D,
    MF63 = 0x30 | 0x0E,
}

#[cfg(feature = "f3")]
impl PllMF {
    pub fn value(&self) -> u8 {
        match self {
            Self::MF2 => 2,
            Self::MF3 => 3,
            Self::MF4 => 4,
            Self::MF5 => 5,
            Self::MF6 => 6,
            Self::MF7 => 7,
            Self::MF8 => 8,
            Self::MF9 => 9,
            Self::MF10 => 10,
            Self::MF11 => 11,
            Self::MF12 => 12,
            Self::MF13 => 13,
            Self::MF14 => 14,
            Self::MF15 => 15,
            Self::MF16 => 16,
            Self::MF17 => 17,
            Self::MF18 => 18,
            Self::MF19 => 19,
            Self::MF20 => 20,
            Self::MF21 => 21,
            Self::MF22 => 22,
            Self::MF23 => 23,
            Self::MF24 => 24,
            Self::MF25 => 25,
            Self::MF26 => 26,
            Self::MF27 => 27,
            Self::MF28 => 28,
            Self::MF29 => 29,
            Self::MF30 => 30,
            Self::MF31 => 31,
            Self::MF32 => 32,
            Self::MF33 => 33,
            Self::MF34 => 34,
            Self::MF35 => 35,
            Self::MF36 => 36,
            Self::MF37 => 37,
            Self::MF38 => 38,
            Self::MF39 => 39,
            Self::MF40 => 40,
            Self::MF41 => 41,
            Self::MF42 => 42,
            Self::MF43 => 43,
            Self::MF44 => 44,
            Self::MF45 => 45,
            Self::MF46 => 46,
            Self::MF47 => 47,
            Self::MF48 => 48,
            Self::MF49 => 49,
            Self::MF50 => 50,
            Self::MF51 => 51,
            Self::MF52 => 52,
            Self::MF53 => 53,
            Self::MF54 => 54,
            Self::MF55 => 55,
            Self::MF56 => 56,
            Self::MF57 => 57,
            Self::MF58 => 58,
            Self::MF59 => 59,
            Self::MF60 => 60,
            Self::MF61 => 61,
            Self::MF62 => 62,
            Self::MF63 => 63,
        }
    }
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// Division factor for the AHB clock. Also known as AHB Prescaler.
pub enum AHBclkPrescaler {
    Div1 = 0b0000,
    Div2 = 0b1000,
    Div4 = 0b1001,
    Div8 = 0b1010,
    Div16 = 0b1011,
    Div64 = 0b1100,
    Div128 = 0b1101,
    Div256 = 0b1110,
    Div512 = 0b1111,
}

impl AHBclkPrescaler {
    pub fn value(&self) -> u16 {
        match self {
            Self::Div1 => 1,
            Self::Div2 => 2,
            Self::Div4 => 4,
            Self::Div8 => 8,
            Self::Div16 => 16,
            Self::Div64 => 64,
            Self::Div128 => 128,
            Self::Div256 => 256,
            Self::Div512 => 512,
        }
    }
}

// f3 uses 0 - 2 only. F4 uses up to 7.
#[derive(Clone, Copy)]
#[repr(u8)]
/// Represents Flash wait states in the FLASH_ACR register.
enum WaitState {
    W0 = 0,
    W1 = 1,
    W2 = 2,
    #[cfg(feature = "f4")]
    W3 = 3,
    #[cfg(feature = "f4")]
    W4 = 4,
    #[cfg(feature = "f4")]
    W5 = 5,
    #[cfg(feature = "f4")]
    W6 = 6,
    #[cfg(feature = "f4")]
    W7 = 7,
}

#[derive(Clone, Copy)]
#[repr(u8)]
/// For use with `RCC_APBPPRE1`, and `RCC_APBPPRE2`. Ie, low-speed and high-speed prescalers respectively.
pub enum ApbPrescaler {
    Div1 = 0b000,
    Div2 = 0b100,
    Div4 = 0b101,
    Div8 = 0b110,
    Div16 = 0b111,
}

impl ApbPrescaler {
    pub fn value(&self) -> u8 {
        match self {
            Self::Div1 => 1,
            Self::Div2 => 2,
            Self::Div4 => 4,
            Self::Div8 => 8,
            Self::Div16 => 16,
        }
    }
}

#[cfg(feature = "f3")]
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum UsbPrescaler {
    Div1_5 = 0,
    Div1 = 1,
    Div2_5 = 2,
    Div2 = 3,
    Div3 = 4,
    Div3_5 = 5,
    Div4 = 6,
}

#[cfg(feature = "f3")]
impl UsbPrescaler {
    pub fn value(&self) -> f32 {
        match self {
            Self::Div1_5 => 1.5,
            Self::Div1 => 1.,
            Self::Div2_5 => 2.5,
            Self::Div2 => 2.,
            Self::Div3 => 3.,
            Self::Div3_5 => 3.5,
            Self::Div4 => 4.,
        }
    }
}



#[cfg(feature = "f3")]
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum UsbClockSource {
    PLL = 0,
    IRC48M = 1,
}


/// Settings used to configure clocks. Create this struct by using its `Default::default()`
/// implementation, then modify as required, referencing your RM's clock tree,
/// or Stm32Cube IDE's interactive clock manager. Apply settings by running `.setup()`.
pub struct Clocks {
    /// The input source for the system and peripheral clocks. Eg HSE, HSI, PLL etc
    pub input_src: InputSrc,

    #[cfg(feature = "f3")]
    /// Input source predivision, for PLL
    pub prediv0: Prediv,
    #[cfg(feature = "f3")]
    pub pll_mul: PllMF,


    #[cfg(feature = "f3")]
    pub usb_pre: UsbPrescaler, // USB prescaler, for target of 48Mhz.
    #[cfg(feature = "f3")]
    pub usb_clock_source: UsbClockSource, 
    /// The value to divide SYSCLK by, to get systick and peripheral clocks. Also known as AHB divider
    pub ahbclk_prescaler: AHBclkPrescaler,
    /// The divider of HCLK to get the APB1 peripheral clock
    pub apb1_prescaler: ApbPrescaler,
    /// The divider of HCLK to get the APB2 peripheral clock
    pub apb2_prescaler: ApbPrescaler,
    /// Bypass the HSE output, for use with oscillators that don't need it. Saves power, and
    /// frees up the pin for use as GPIO.
    pub hse_bypass: bool,
    pub clock_monitor: bool,
}

impl Clocks {
    /// Setup common and return a `Valid` status if the config is valid. Return
    /// `Invalid`, and don't setup if not.
    pub fn setup(&self) -> Result<(), SpeedError> {
        if let Err(e) = self.validate_speeds() {
            return Err(e);
        }

        let rcu = unsafe { &(*RCU::ptr()) };


        let sysclk = self.sysclk();

        let hclk = sysclk / self.ahbclk_prescaler.value() as u32;

        // 303 RM, 5.2.1:
        // The internal PLL can be used to multiply the HSI or HSE output clock frequency. Refer to
        // Figure 5-2 and Clock Control register ((RCU_CTL).
        // The PLL configuration (selection of the input clock, and multiplication factor) must be done
        // before enabling the PLL. Once the PLL is enabled, these parameters cannot be changed.
        // To modify the PLL configuration, proceed as follows:
        // 1. Disable the PLL by setting PLLEN to 0.
        // 2. Wait until PLLSTB is cleared. The PLL is now fully stopped.
        // 3. Change the desired parameter.
        // 4. Enable the PLL again by setting PLLEN to 1.
        // An interrupt can be generated when the PLL is ready, if enabled in the Clock interrupt
        // register (RCU_INT).
        // The PLL output frequency must not exceed 120 MHz.
        // Set up the HSE if required.

        // Enable oscillators, and wait until ready.
        match self.input_src {
            InputSrc::HXTAL(_) => {
                rcu.ctl.modify(|_, w| w.hxtalen().bit(true));
                // Wait for the HSE to be ready.
                while rcu.ctl.read().hxtalstb().bit_is_clear() {}
            }
            InputSrc::IRC8M => {
                rcu.ctl.modify(|_, w| w.irc8men().bit(true));
                while rcu.ctl.read().irc8mstb().bit_is_clear() {}
            }
            InputSrc::Pll(pll_src) => {
                match pll_src {
                    PllSrc::HXTAL(_) => {
                        // DRY
                        rcu.ctl.modify(|_, w| w.hxtalen().bit(true));
                        while rcu.ctl.read().hxtalstb().bit_is_clear() {}
                    }
                    PllSrc::IRC8MDiv2 => {
                        rcu.ctl.modify(|_, w| w.irc8men().bit(true));
                        while rcu.ctl.read().irc8mstb().bit_is_clear() {}
                    }
                    PllSrc::IRC48M => {
                        rcu.addctl.modify(|_, w| w.irc48men().bit(true));
                        while rcu.addctl.read().irc48mstb().bit_is_clear() {}
                    }
                }
            }
        }

        if let UsbClockSource::IRC48M = self.usb_clock_source {
            // enable IRC48 if USB need it
            rcu.addctl.modify(|_, w| w.irc48men().bit(true));
            while rcu.addctl.read().irc48mstb().bit_is_clear() {}

            // set USB to use IRC48M as clock
            rcu.addctl.modify(|_, w|w.ck48msel().bit(true));
        }

        rcu.ctl.modify(|_, w| {
            // Enable bypass mode on HSE, since we're using a ceramic oscillator.
            w.hxtalbps().bit(self.hse_bypass)
        });

        if let InputSrc::Pll(pll_src) = self.input_src {
            // Turn off the PLL: Required for modifying some of the settings below.
            rcu.ctl.modify(|_, w| w.pllen().clear_bit());
            // Wait for the PLL to no longer be ready before executing certain writes.
            while rcu.ctl.read().pllstb().bit_is_set() {}

            cfg_if! {
                if #[cfg(feature = "f3")] {
                    rcu.cfg0.modify(|_, w| {
                        cfg_if! {
                            if #[cfg(any(feature = "f305", feature = "f307"))] {
                                // TODO not supported yet
                            } else {
                                unsafe {
                                    w
                                    .pllmf_3_0().bits(self.pll_mul as u8 & 0x0F)
                                    .pllmf_4().bit(self.pll_mul as u8 & 0x10 != 0)
                                    .pllmf_5().bit(self.pll_mul as u8 & 0x20 != 0);

                                    match pll_src {
                                        PllSrc::IRC8MDiv2 => w.pllsel().clear_bit(),
                                        _ => w.pllsel().set_bit(),
                                    };
                                    w
                                } 
                            }
                        }
                    });

                    if let PllSrc::IRC48M = pll_src {
                        rcu.cfg1.modify(|_,w| w.pllpresel().set_bit());
                    }
                } 
            }

            #[cfg(feature = "f3")]
            rcu.cfg0.modify(|_, w| w.predv0().bit(self.prediv0 as u8 == 1));

            // Now turn PLL back on, once we're configured things that can only be set with it off.
            rcu.ctl.modify(|_, w| w.pllen().set_bit());

            while rcu.ctl.read().pllstb().bit_is_clear() {}
        }

        rcu.cfg0.modify(|_, w| unsafe {
            #[cfg(not(any(feature = "f301", feature = "f3x4", feature = "f4")))]
            w.usbdpsc_1_0().bits(self.usb_pre as u8 & 0x03).usbdpsc_2().bit(self.usb_pre as u8 & 0x04 != 0); // eg: Divide by 1.5: 72/1.5 = 48Mhz, required by USB clock.

            w.scs().bits(self.input_src.bits());
            w.ahbpsc().bits(self.ahbclk_prescaler as u8); // eg: Divide SYSCLK by 2 to get HCLK of 36Mhz.
            w.apb2psc().bits(self.apb2_prescaler as u8); // HCLK division for APB2.
            w.apb1psc().bits(self.apb1_prescaler as u8) // HCLK division for APB1
        });

        rcu.ctl.modify(|_, w| w.ckmen().bit(self.clock_monitor));

        // If we're not using the default clock source as input source or for PLL, turn it off.
        match self.input_src {
            InputSrc::IRC8M => (),
            InputSrc::Pll(pll_src) => match pll_src {
                #[cfg(feature = "f3")]
                PllSrc::IRC8MDiv2 => (),
                #[cfg(feature = "f4")]
                PllSrc::Hsi => (),
                _ => {
                    rcu.ctl.modify(|_, w| w.irc8men().clear_bit());
                }
            },
            InputSrc::HXTAL(_) => {
                rcu.ctl.modify(|_, w| w.irc8men().clear_bit());
            }
        }

        Ok(())
    }

    /// Re-select innput source; used on Stop and Standby modes, where the system reverts
    /// to HSI after wake.
    pub fn reselect_input(&self) {
        let rcu = unsafe { &(*RCU::ptr()) };
        // Re-select the input source; it will revert to HSI during `Stop` or `Standby` mode.

        // Note: It would save code repetition to pass the `Clocks` struct in and re-run setup
        // todo: But this saves a few reg writes.
        match self.input_src {
            InputSrc::HXTAL(_) => {
                rcu.ctl.modify(|_, w| w.hxtalen().set_bit());
                while rcu.ctl.read().hxtalstb().bit_is_clear() {}

                rcu.cfg0
                    .modify(|_, w| unsafe { w.scs().bits(self.input_src.bits()) });
            }
            InputSrc::Pll(_) => {
                // todo: DRY with above.
                rcu.ctl.modify(|_, w| w.hxtalen().set_bit());
                while rcu.ctl.read().hxtalstb().bit_is_clear() {}

                rcu.ctl.modify(|_, w| w.pllen().clear_bit());
                while rcu.ctl.read().pllstb().bit_is_set() {}

                rcu.cfg0
                    .modify(|_, w| unsafe { w.scs().bits(self.input_src.bits()) });

                rcu.ctl.modify(|_, w| w.pllen().set_bit());
                while rcu.ctl.read().pllstb().bit_is_clear() {}
            }
            InputSrc::IRC8M => (), // Already reset to this.
        }
    }

    #[cfg(feature = "f3")]
    /// Calculate the sysclock frequency, in  Hz.
    /// 
    /// TODO 适配内部函数
    pub fn sysclk(&self) -> u32 {
        match self.input_src {
            InputSrc::Pll(pll_src) => match pll_src {
                PllSrc::HXTAL(freq) => {
                    freq / self.prediv0.value() as u32 * self.pll_mul.value() as u32
                },
                PllSrc::IRC48M => 48_000_000,
                PllSrc::IRC8MDiv2 => 4_000_000 * self.pll_mul.value() as u32,
            },
            InputSrc::IRC8M => 8_000_000,
            InputSrc::HXTAL(freq) => freq,
        }
    }


    /// Check if the PLL is enabled. This is useful if checking wheather to re-enable the PLL
    /// after exiting Stop or Standby modes, eg so you don't re-enable if it was already re-enabled
    /// in a different context. eg:
    /// ```
    /// if !clock_cfg.pll_is_enabled() {
    ///     clock_cfg.reselect_input();
    ///}
    ///```
    pub fn pll_is_enabled(&self) -> bool {
        let rcu = unsafe { &(*RCU::ptr()) };
        rcu.ctl.read().pllen().bit_is_set()
    }

    pub fn abhclk(&self) -> u32 {
        self.sysclk() / self.ahbclk_prescaler.value() as u32
    }

    pub fn systick(&self) -> u32 {
        self.abhclk() / 8
    }

    pub fn usb(&self) -> u32 {
        #[cfg(feature = "f3")]
        return self.sysclk() / self.usb_pre.value() as u32;
        #[cfg(feature = "f4")]
        return 0; // todo
    }

    pub fn apb1(&self) -> u32 {
        self.abhclk() / self.apb1_prescaler.value() as u32
    }

    pub fn apb1_timer(&self) -> u32 {
        if let ApbPrescaler::Div1 = self.apb1_prescaler {
            self.apb1()
        } else {
            self.apb1() * 2
        }
    }

    pub fn apb2(&self) -> u32 {
        self.abhclk() / self.apb2_prescaler.value() as u32
    }

    pub fn apb2_timer(&self) -> u32 {
        if let ApbPrescaler::Div1 = self.apb2_prescaler {
            self.apb2()
        } else {
            self.apb2() * 2
        }
    }

    pub fn validate_speeds(&self) -> Result<(), SpeedError> {
        #[cfg(feature = "f303")]
        let (max_sys, max_ahb, max_apb1, max_apb2) = (120_000_000, 120_000_000, 60_000_000, 120_000_000);



        // todo: min clock? eg for apxb?
        if self.sysclk() > max_sys {
            return Err(SpeedError::SysclkOutOfLimits);
        }

        if self.abhclk() > max_ahb {
            return Err(SpeedError::HclkOutOfLimits);
        }

        if self.apb1() > max_apb1 {
            return Err(SpeedError::Apb1OutOfLimits);
        }

        if self.apb2() > max_apb2 {
            return Err(SpeedError::Apb2OutOfLimits);
        }

        Ok(())
    }
}

impl Default for Clocks {
    #[cfg(feature = "f3")]
    /// This default configures common with a HSI, a 64Mhz sysclck. All peripheral common are at
    /// 64Mhz, except for APB1, which is at and 32Mhz. Not valid for USB.
    fn default() -> Self {
        Self {
            input_src: InputSrc::Pll(PllSrc::IRC8MDiv2),
            prediv0: Prediv::Div1,
            pll_mul: PllMF::MF18,
            usb_pre: UsbPrescaler::Div1,
            usb_clock_source: UsbClockSource::PLL,
            ahbclk_prescaler: AHBclkPrescaler::Div1,
            apb1_prescaler: ApbPrescaler::Div2,
            apb2_prescaler: ApbPrescaler::Div1,
            hse_bypass: false,
            clock_monitor: false,
        }
    }
}
