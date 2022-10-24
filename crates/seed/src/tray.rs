use serde::Serialize;
use tauri::CustomMenuItem;
use tauri::{AppHandle, Manager, SystemTray, SystemTrayEvent, Wry};
use tauri::{SystemTrayMenu, SystemTrayMenuItem};

pub struct SystemTrayBuilder {}

impl SystemTrayBuilder {
    pub fn build() -> SystemTray {
        Self::new_tray()
    }

    fn new_tray() -> SystemTray {
        SystemTray::new().with_menu(Self::build_menu())
    }

    fn build_menu() -> SystemTrayMenu {
        let mut menu = SystemTrayMenu::new();
        let action_names = vec!["Open", "Settings", "-", "Quit"];
        for tray_menu_name in action_names.into_iter() {
            let id = tray_menu_name.to_string();
            if id.as_str() == "-" {
                menu = menu.add_native_item(SystemTrayMenuItem::Separator);
            } else {
                let item = CustomMenuItem::new(id, tray_menu_name);
                menu = menu.add_item(item)
            };
        }
        menu
    }

    pub fn handle_tray_event(app_handler: &AppHandle<Wry>, e: SystemTrayEvent) {
        match e {
            SystemTrayEvent::MenuItemClick { id, .. } => match id.as_str() {
                "Open" => show_app(app_handler),
                "Settings" => open_settings(app_handler),
                "Quit" => quit_app(app_handler),
                _ => {}
            },
            SystemTrayEvent::LeftClick { .. } => {}
            SystemTrayEvent::RightClick { .. } => {}
            SystemTrayEvent::DoubleClick { .. } => {}
            _ => {}
        }
    }
}

fn emit_to_main<S>(app_handler: &AppHandle<Wry>, event: &str, payload: S)
where
    S: Serialize + Clone,
{
    let _ = app_handler.get_window("main").map(|win| {
        let _ = win.emit(event, payload);
    });
}

fn show_app(a: &AppHandle<Wry>) {
    let _ = a.get_window("main").map(|win| {
        let _ = win.unminimize();
        let _ = win.show();
        let _ = win.set_focus();
    });
}

fn open_settings(a: &AppHandle<Wry>) {
    emit_to_main(a, "open-settings", ());
}

fn quit_app(app_handler: &AppHandle<Wry>) {
    app_handler.exit(0)
}
