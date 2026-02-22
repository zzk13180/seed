module.exports = {
  apps: [
    {
      name: 'seed-mobile',
      script: 'pnpm',
      args: 'production', // 生产模式：vite serve --mode production
      // 如果已构建静态文件，可改为: args: 'preview'
      instances: '1',
      cwd: '.',
      watch: false,
      exec_mode: 'fork',
      env: {
        NODE_ENV: 'production',
      },
      error_file: '../../logs/mobile/err.log',
      out_file: '../../logs/mobile/out.log',
      log_date_format: 'YYYY-MM-DD HH:mm:ss',
      merge_logs: true,
    },
  ],
}
