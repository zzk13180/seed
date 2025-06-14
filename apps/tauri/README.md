# @seed/tauri

```json
"rust-analyzer.linkedProjects": [
  ".\\apps\\tauri\\crates\\seed\\Cargo.toml"
]
// "rust-analyzer.linkedProjects": ["./apps/tauri/crates/seed/Cargo.toml"]
```


```bash
rustup update

pnpm tauri info
pnpm tauri dev
```

对于 Android：

```bash
# 初始化 Android 开发环境（仅需运行一次）
pnpm tauri android init

pnpm tauri android dev
# 或在真机上进行开发
pnpm tauri android dev --host
```

对于 iOS：

```bash
# 设置 iOS 开发环境（仅需运行一次）
pnpm tauri ios init

pnpm tauri ios dev
# 或在真机上进行开发
pnpm tauri ios dev --host
```

```bash
pnpm tauri build
pnpm tauri android build
pnpm tauri ios build
```



```sh
# https://crates.io/crates/tauri-cli
# cargo install tauri-cli
xcodebuild -importPlatform ./iOS_18.2_Simulator_Runtime.dmg 
```
