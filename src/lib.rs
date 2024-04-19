#![no_std]

extern crate alloc;



pub mod serials;
mod task;
mod waker;

pub const TEST: usize = 1;

#[cfg(test)]
mod tests {

}