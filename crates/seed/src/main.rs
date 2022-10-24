#![cfg_attr(
    all(not(debug_assertions), target_os = "windows"),
    windows_subsystem = "windows"
)]

mod tray;

// Learn more about Tauri commands at https://tauri.app/v1/guides/features/command
#[tauri::command]
fn greet(name: &str) -> String {
    format!("Hello, {}! You've been greeted from Rust!", name)
}

#[tauri::command]
fn is_win_11() -> bool {
    common::is_win_11()
}

fn main() {
    tauri::Builder::default()
        .system_tray(tray::SystemTrayBuilder::build())
        .on_system_tray_event(tray::SystemTrayBuilder::handle_tray_event)
        .invoke_handler(tauri::generate_handler![greet, is_win_11])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
