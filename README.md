# is31fl3733-rs
Rust interface for the Lumissil IS31FL3733 16x12 LED Driver.

This crate currently only contains the minimal functionality required to bring up the chip and control the leds.

Control of the LEDs on/off state is through the `set_leds` function, which writes directly to the internal chip state.
Each bit in the 24 byte buffer sequentially maps to a LED. 

Similarly the brightness is controlled through `set_brightness` which writes to the chips PWM state.
In this case each LED has its own byte determining brightness.

The overall maximum brightness limit can be adjusted using `set_global_control_current`.

See the [datasheet](https://www.lumissil.com/assets/pdf/core/IS31FL3733_DS.pdf) for more information.


An internal state is held and diffed against to keep the amount of bus writes to a minimum.

# Example
```rust
let is31fl3733 = IS31FL3733::is31fl3733::new();

is31fl3733.initialize()?;
is31fl3733.set_global_control_current(0xff)?; // Maximum brightness limit
is31fl3733.set_leds(&[0xffu8; 24])?; // Turn on all LEDs

// Set each LED's brightness to its index value
let brightness: [u8; 192] = core::array::from_fn(|i| i);
is31fl3733.set_brightness(&brightness);
```

# TBD
* LED open/short state detection
* Interrupt support
* Auto breathing mode

Disclaimer: This library is not an official product, use freely at your own risk.
