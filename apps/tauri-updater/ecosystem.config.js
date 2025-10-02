// eslint-disable-next-line unicorn/prefer-module
module.exports = {
  apps: [
    {
      name: 'tauri-updater', // 应用名称，用于 pm2 管理和显示
      script: 'dist/main.js', // 启动脚本路径，pm2 会运行此文件
      instances: '1', // 实例数量，'max' 表示根据 CPU 核心数自动启动多个进程(max 要配合 cluster 模式使用)
      args: '',
      cwd: '.',
      watch: false,
      exec_mode: 'fork', // 'fork' 单进程模式 'cluster' 支持负载均衡和多进程
      env: {
        NODE_ENV: 'production', // 环境变量，指定为生产环境
      },
      error_file: './logs/err.log', // 错误日志文件路径，记录标准错误输出
      out_file: './logs/out.log', // 普通日志文件路径，记录标准输出
      log_date_format: 'YYYY-MM-DD HH:mm:ss', // 日志时间格式，便于日志追踪
    },
  ],
}
