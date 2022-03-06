# GD32-HAL

<!-- [![Crate](https://img.shields.io/crates/v/gd32-hal.svg)](https://crates.io/crates/gd32-hal)
[![Docs](https://docs.rs/gd32-hal/badge.svg)](https://docs.rs/stm32-hal2) -->


> This repo is a fork from https://github.com/David-OConnor/stm32-hal and ported for GD32 series MCU. 
> So, you should read stm32-hal's `Readme` file first.


This library provides high-level access to GD32 peripherals.


### About this repo

At the time I setup this repo, I found there are two different style of HAL-Impl crates for STM32:
* one is the older crate per series style, there are different crate for different series of STM32, like the [stm32f0xx-hal](https://crates.io/crates/stm32f0xx-hal), [stm32f1xx-hal](https://github.com/stm32-rs/stm32f1xx-hal), [stm32f3xx-hal](https://crates.io/crates/stm32f3xx-hal) and so on. To my surprise, although those crates impl traits defined in `embedded-hal` project, the impl detail and other API beyond `embedded-hal` differs a lot.

* another one is the younger all-in-one crate like [stm32-hal](https://github.com/David-OConnor/stm32-hal)


Since the all-in-one version:
* api is more easy to learn 
* more easy to port from one device to another
* source code is more easy to read and thus it easier for others to contribute

This repo is a forked from [stm32-hal](https://github.com/David-OConnor/stm32-hal) project instead of [stm32f3xx-hal](https://crates.io/crates/stm32f3xx-hal) project.

All kinds of contribute is welcomed since a single person of myself doesn't have enough energy. Now I will only port some basic peripheral and only target for GD32F303 series.

You can find the contribute guide in the [stm32-hal](https://github.com/David-OConnor/stm32-hal) project