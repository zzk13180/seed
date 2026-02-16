use sysinfo::System;

pub fn is_win_11() -> bool {
    let version = System::os_version().unwrap_or_default();
    // Windows 11 build numbers start at 22000
    version
        .split('(')
        .nth(1)
        .and_then(|s| s.split(')').next())
        .and_then(|s| s.split('.').next())
        .and_then(|s| s.parse::<u32>().ok())
        .map_or(false, |build| build >= 22000)
}
