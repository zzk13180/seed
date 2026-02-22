use tauri::{App, Manager};

#[cfg(mobile)]
mod mobile;

#[cfg(mobile)]
pub use mobile::*;

pub type SetupHook = Box<dyn FnOnce(&mut App) -> Result<(), Box<dyn std::error::Error>> + Send>;

#[tauri::command]
fn greet(name: &str) -> String {
    format!("Hello, {}! You've been greeted from Rust!", name)
}

#[tauri::command]
fn is_win_11() -> bool {
    common::is_win_11()
}

#[derive(Default)]
pub struct AppBuilder {
    setup: Option<SetupHook>,
}

impl AppBuilder {
    pub fn new() -> Self {
        Self::default()
    }

    #[must_use]
    pub fn setup<F>(mut self, setup: F) -> Self
    where
        F: FnOnce(&mut App) -> Result<(), Box<dyn std::error::Error>> + Send + 'static,
    {
        self.setup.replace(Box::new(setup));
        self
    }

    pub fn run(self) {
        let setup = self.setup;
        tauri::Builder::default()
            .plugin(tauri_plugin_os::init())
            .plugin(tauri_plugin_process::init())
            .plugin(tauri_plugin_opener::init())
            .plugin(tauri_plugin_updater::Builder::new().build())
            .plugin(tauri_plugin_window_state::Builder::new().build())
            .plugin(tauri_plugin_single_instance::init(|app, _args, _cwd| {
                // Focus main window when another instance is launched
                if let Some(window) =
                    app.get_webview_window("main")
                {
                    let _ = window.set_focus();
                }
            }))
            .plugin(
                tauri_plugin_log::Builder::new()
                    .level(log::LevelFilter::Info)
                    .build(),
            )
            .setup(move |app| {
                if let Some(setup) = setup {
                    (setup)(app)?;
                }

                // Show main window after setup (prevents white flash)
                if let Some(window) = app.get_webview_window("main") {
                    let _ = window.show();
                }

                Ok(())
            })
            .invoke_handler(tauri::generate_handler![greet, is_win_11])
            .run(tauri::generate_context!())
            .expect("error while running tauri application");
    }
}
