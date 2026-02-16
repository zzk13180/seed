import type { UserVO } from '@seed/contracts'
import type { UserState, UserDeps, LoginParams } from './user.types'

/**
 * UserController - 纯 TypeScript 类，不依赖任何框架
 *
 * 职责：
 * - 管理用户认证状态
 * - 处理登录/登出逻辑
 * - 通过 Deps 注入执行副作用（API 调用、存储等）
 *
 * Better Auth 使用 cookie-session 模式:
 * - 登录后浏览器自动管理 session cookie
 * - 通过 getSession() 获取当前用户状态
 */
export class UserController {
  private readonly state: UserState
  private readonly deps: UserDeps

  constructor(state: UserState, deps: UserDeps) {
    this.state = state
    this.deps = deps
  }

  /**
   * 检查是否已登录
   */
  get isLoggedIn(): boolean {
    return !!this.state.user
  }

  /**
   * 获取用户名（Better Auth 使用 name 字段）
   */
  get username(): string {
    return this.state.user?.name || ''
  }

  /**
   * 获取昵称
   */
  get nickname(): string {
    return this.state.user?.name || ''
  }

  /**
   * 获取头像
   */
  get avatar(): string {
    return this.state.user?.image || ''
  }

  /**
   * 获取角色
   */
  get role(): string {
    return this.state.user?.role || ''
  }

  /**
   * 是否为管理员
   */
  get isAdmin(): boolean {
    return this.state.user?.role === 'admin'
  }

  /**
   * 用户登录
   */
  async login(params: LoginParams): Promise<UserVO> {
    this.state.loading = true
    this.deps.logger.info('User login attempt', { email: params.email })

    try {
      const user = await this.deps.authService.login(params)
      this.state.user = user
      this.deps.logger.info('User login successful', { userId: user.id })
      return user
    } catch (error) {
      this.deps.logger.error('User login failed', error)
      throw error
    } finally {
      this.state.loading = false
    }
  }

  /**
   * 用户登出
   */
  async logout(): Promise<void> {
    this.state.loading = true
    this.deps.logger.info('User logout')

    try {
      await this.deps.authService.logout()
    } catch (error) {
      this.deps.logger.warn('Logout API failed, clearing local state anyway', error)
    } finally {
      this.state.user = null
      this.state.loading = false
    }
  }

  /**
   * 获取当前用户信息
   *
   * 通过 Better Auth session 获取当前用户
   */
  async fetchUser(): Promise<UserVO | null> {
    this.state.loading = true
    this.deps.logger.debug('Fetching current user from session')

    try {
      const user = await this.deps.authService.getCurrentUser()
      this.state.user = user
      return user
    } catch (error) {
      this.deps.logger.error('Failed to fetch user', error)
      this.state.user = null
      return null
    } finally {
      this.state.loading = false
    }
  }

  /**
   * 初始化用户状态
   *
   * 在应用启动时调用：
   * - Better Auth 使用 cookie-session，检查当前 session 是否有效
   * - 如果有有效 session，自动获取用户信息
   */
  async init(): Promise<void> {
    this.deps.logger.debug('Initializing user state from session')
    await this.fetchUser()
  }

  /**
   * 设置用户信息
   */
  setUser(user: UserVO | null): void {
    this.state.user = user
  }

  /**
   * 保存登录凭据到本地存储
   */
  saveCredentials(email: string, password: string): void {
    this.deps.storageService.set('login_email', email)
    this.deps.storageService.set('login_password', password)
    this.deps.storageService.set('rememberMe', 'true')
    this.deps.logger.debug('Credentials saved')
  }

  /**
   * 清除保存的登录凭据
   */
  clearCredentials(): void {
    this.deps.storageService.remove('login_email')
    this.deps.storageService.remove('login_password')
    this.deps.storageService.remove('rememberMe')
    this.deps.logger.debug('Credentials cleared')
  }

  /**
   * 获取保存的登录凭据
   */
  getSavedCredentials(): { email: string; password: string } | null {
    const email = this.deps.storageService.get('login_email')
    const password = this.deps.storageService.get('login_password')
    const rememberMe = this.deps.storageService.get('rememberMe')

    if (email && password && rememberMe) {
      return { email, password }
    }
    return null
  }

  /**
   * 导航到登录页
   */
  async navigateToLogin(): Promise<void> {
    await this.deps.navigation.push('/login')
  }

  /**
   * 导航到首页
   */
  async navigateToHome(): Promise<void> {
    await this.deps.navigation.push('/')
  }
}
