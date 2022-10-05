use sysinfo::{System, SystemExt};

pub fn is_win_11() -> bool {
    let sys = System::new_all();
    let version = sys.os_version().unwrap();
    let version = version.split('(').collect::<Vec<&str>>()[1]
        .split(')')
        .collect::<Vec<&str>>()[0];
    let version: u32 = version.split('.').collect::<Vec<&str>>()[0]
        .parse()
        .unwrap();
    version >= 22000
}
