/**
 * @file 数据库 Seed 脚本
 * @description 幂等地创建初始管理员账号，确保系统开箱即用
 *
 * 用法：
 *   pnpm nx run @seed/server:db:seed
 *   # 或直接
 *   bun run src/seed.ts
 *
 * 设计原则：
 * - 幂等：重复执行不会创建重复数据
 * - 安全：密码通过 Better Auth 内置的 bcrypt 哈希
 * - 可配置：通过环境变量覆盖默认值
 *
 * 默认管理员：
 *   email:    admin@seed.dev
 *   password: admin123
 *   role:     admin
 */

import { eq } from 'drizzle-orm'
import * as schema from '@seed/db/schema'
import { auth } from './config/auth.config'
import { createDatabase } from './database/database.provider'
import { getPool } from './database/database.provider'

// ── Seed 配置（可通过环境变量覆盖）──

const SEED_ADMIN_EMAIL = process.env.SEED_ADMIN_EMAIL ?? 'admin@seed.dev'
const SEED_ADMIN_PASSWORD = process.env.SEED_ADMIN_PASSWORD ?? 'admin123'
const SEED_ADMIN_NAME = process.env.SEED_ADMIN_NAME ?? 'Admin'

async function seed() {
  console.log('🌱 Seed: 开始初始化数据...\n')

  const db = createDatabase()

  // ── 1. 创建管理员账号 ──

  const existing = await db
    .select({ id: schema.user.id, email: schema.user.email, role: schema.user.role })
    .from(schema.user)
    .where(eq(schema.user.email, SEED_ADMIN_EMAIL))
    .limit(1)

  if (existing.length > 0) {
    const admin = existing[0]
    if (!admin) {
      throw new Error('管理员记录读取失败')
    }
    console.log(`  ✅ 管理员已存在: ${admin.email} (role=${admin.role})`)

    // 确保角色是 admin
    if (admin.role !== 'admin') {
      await db.update(schema.user).set({ role: 'admin' }).where(eq(schema.user.id, admin.id))
      console.log(`  🔄 已将角色从 "${admin.role}" 升级为 "admin"`)
    }
  } else {
    // 通过 Better Auth API 创建（自动处理密码哈希 + account 记录）
    const result = await auth.api.signUpEmail({
      body: {
        email: SEED_ADMIN_EMAIL,
        password: SEED_ADMIN_PASSWORD,
        name: SEED_ADMIN_NAME,
      },
    })

    if (!result?.user) {
      throw new Error('创建管理员失败')
    }

    // Better Auth 创建的用户默认 role='user'，升级为 admin
    await db.update(schema.user).set({ role: 'admin' }).where(eq(schema.user.id, result.user.id))

    console.log(`  ✅ 管理员已创建: ${SEED_ADMIN_EMAIL} (role=admin)`)
  }

  // ── 完成 ──

  console.log('\n🌱 Seed: 初始化完成!')
  console.log('  📧 邮箱:  ', SEED_ADMIN_EMAIL)
  console.log('  🔑 密码:  ', SEED_ADMIN_PASSWORD)
  console.log('  👤 角色:   admin\n')

  // 关闭数据库连接
  const pool = getPool()
  if (pool) {
    await pool.end()
  }
}

seed().catch(error => {
  console.error('❌ Seed 失败:', error)
  process.exit(1)
})
