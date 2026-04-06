#![macro_use]

#[cfg(feature = "defmt")]
macro_rules! info {
    ($($arg:tt)*) => { defmt::info!($($arg)*) };
}

#[cfg(not(feature = "defmt"))]
macro_rules! info {
    ($($arg:tt)*) => {{}};
}

#[cfg(feature = "defmt")]
macro_rules! warn {
    ($($arg:tt)*) => { defmt::warn!($($arg)*) };
}

#[cfg(not(feature = "defmt"))]
macro_rules! warn {
    ($($arg:tt)*) => {{}};
}
