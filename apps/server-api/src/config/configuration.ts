export default () => {
  const isProd = process.env.NODE_ENV === 'production'
  const jwtSecret = process.env.JWT_SECRET

  // 生产环境必须配置 JWT_SECRET
  if (isProd && !jwtSecret) {
    throw new Error('JWT_SECRET environment variable is required in production')
  }

  // 生产环境禁止使用 synchronize
  const dbSync = process.env.DB_SYNC === 'true'
  if (isProd && dbSync) {
    throw new Error('DB_SYNC=true is forbidden in production! Use migrations instead.')
  }

  return {
    port: Number.parseInt(process.env.PORT || '3003', 10),
    apiPrefix: process.env.API_PREFIX || 'api',
    isDev: process.env.NODE_ENV === 'development',
    isProd,

    // JWT 配置
    jwt: {
      // 生产环境必须配置，开发环境可使用默认值
      secret: jwtSecret || (isProd ? '' : 'dev-secret-key-not-for-production'),
      accessTokenExpiry: process.env.JWT_ACCESS_TOKEN_EXPIRY || '1h',
      refreshTokenExpiry: process.env.JWT_REFRESH_TOKEN_EXPIRY || '7d',
    },

    // 数据库配置
    database: {
      type: 'mysql',
      host: process.env.DB_HOST || 'localhost',
      port: Number.parseInt(process.env.DB_PORT || '3306', 10),
      username: process.env.DB_USERNAME || 'root',
      password: process.env.DB_PASSWORD || 'password',
      database: process.env.DB_DATABASE || 'seed',
      synchronize: isProd ? false : dbSync, // 生产环境强制禁用
      logging: process.env.DB_LOGGING === 'true',
    },

    // Redis 配置
    redis: {
      host: process.env.REDIS_HOST || 'localhost',
      port: Number.parseInt(process.env.REDIS_PORT || '6379', 10),
      password: process.env.REDIS_PASSWORD || '',
      db: Number.parseInt(process.env.REDIS_DB || '0', 10),
    },
  }
}
