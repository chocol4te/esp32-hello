#![feature(never_type)]
#![feature(const_cstr_unchecked)]
#![warn(missing_debug_implementations)]

use std::ffi::CStr;
use std::str;

#[macro_use]
extern crate alloc;

#[macro_use]
mod esp_error;
pub use esp_error::EspError;

mod heap;
pub mod interface;
pub use heap::Heap;
pub mod i2c;
pub mod nvs;
pub mod wifi;
