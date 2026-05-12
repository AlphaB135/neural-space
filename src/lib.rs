#![no_std]
#![no_main]

pub mod afferent;
pub mod central;
pub mod efferent;
pub mod reflex;

// Use libm for no_std math functions (sqrt, exp, etc.)
extern crate libm;

#[cfg(not(test))]
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
