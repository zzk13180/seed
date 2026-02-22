#![cfg_attr(
    all(not(debug_assertions), target_os = "windows"),
    windows_subsystem = "windows"
)]

pub fn main() {
    zzk13180_seed_lib::AppBuilder::new().run();
}
