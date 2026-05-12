#![no_std]
#![no_main]
#![feature(asm_const)]
#![feature(naked_functions)]

pub mod afferent;
pub mod central;
pub mod efferent;
pub mod reflex;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
