module.exports = {
  apps: [
    {
      name: 'seed-server',
      script: 'dist/main.js',
      instances: 'max',
      exec_mode: 'cluster',
      cwd: '.',
      watch: false,
      env: {
        NODE_ENV: 'production',
      },
      error_file: '../../logs/server/err.log',
      out_file: '../../logs/server/out.log',
      log_date_format: 'YYYY-MM-DD HH:mm:ss',
      merge_logs: true,
      // 平滑重启配置
      wait_ready: true,
      listen_timeout: 10000,
      kill_timeout: 5000,
      // 内存限制自动重启
      max_memory_restart: '1G',
    },
  ],
}
