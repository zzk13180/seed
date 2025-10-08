import type { UserState, UserEnv, LoginParams, UserInfo } from './user.types'

/**
 * UserController - 纯 TypeScript 类，不依赖任何框架
 *
 * 职责：
 * - 管理用户认证状态
 * - 处理登录/登出逻辑
 * - 通过 Env 注入执行副作用（API 调用、存储等）
 */
export class UserController {
  private readonly state: UserState
  private readonly env: UserEnv

  constructor(state: UserState, env: UserEnv) {
    this.state = state
    this.env = env
  }

  /**
   * 检查是否已登录
   */
  get isLoggedIn(): boolean {
    return !!this.state.user && this.env.authService.isAuthenticated()
  }

  /**
   * 获取用户名
   */
  get username(): string {
    return this.state.user?.username || ''
  }

  /**
   * 获取昵称
   */
  get nickname(): string {
    return this.state.user?.nickname || this.state.user?.username || ''
  }

  /**
   * 获取头像
   */
  get avatar(): string {
    return this.state.user?.avatar || ''
  }

  /**
   * 用户登录
   */
  async login(params: LoginParams): Promise<UserInfo> {
    this.state.loading = true
    this.env.logger.info('User login attempt', { username: params.username })

    try {
      const result = await this.env.authService.login(params)
      this.state.user = result.user
      this.env.logger.info('User login successful', { userId: result.user.id })
      return result.user
    } catch (error) {
      this.env.logger.error('User login failed', error)
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
    this.env.logger.info('User logout')

    try {
      await this.env.authService.logout()
    } catch (error) {
      this.env.logger.warn('Logout API failed, clearing local state anyway', error)
    } finally {
      this.state.user = null
      this.state.loading = false
    }
  }

  /**
   * 获取当前用户信息
   */
  async fetchUser(): Promise<UserInfo | null> {
    if (!this.env.authService.isAuthenticated()) {
      return null
    }

    this.state.loading = true
    this.env.logger.debug('Fetching current user')

    try {
      const user = await this.env.authService.getCurrentUser()
      this.state.user = user
      return user
    } catch (error) {
      this.env.logger.error('Failed to fetch user', error)
      this.state.user = null
      return null
    } finally {
      this.state.loading = false
    }
  }

  /**
   * 初始化用户状态
   * 在应用启动时调用
   */
  async init(): Promise<void> {
    this.env.logger.debug('Initializing user state')
    if (this.env.authService.initAuth()) {
      await this.fetchUser()
    }
  }

  /**
   * 设置用户信息
   */
  setUser(user: UserInfo | null): void {
    this.state.user = user
  }

  /**
   * 保存登录凭据到本地存储
   */
  saveCredentials(username: string, password: string): void {
    this.env.storageService.set('username', username)
    this.env.storageService.set('password', password)
    this.env.storageService.set('rememberMe', 'true')
    this.env.logger.debug('Credentials saved')
  }

  /**
   * 清除保存的登录凭据
   */
  clearCredentials(): void {
    this.env.storageService.remove('username')
    this.env.storageService.remove('password')
    this.env.storageService.remove('rememberMe')
    this.env.logger.debug('Credentials cleared')
  }

  /**
   * 获取保存的登录凭据
   */
  getSavedCredentials(): { username: string; password: string } | null {
    const username = this.env.storageService.get('username')
    const password = this.env.storageService.get('password')
    const rememberMe = this.env.storageService.get('rememberMe')

    if (username && password && rememberMe) {
      return { username, password }
    }
    return null
  }

  /**
   * 导航到登录页
   */
  async navigateToLogin(): Promise<void> {
    await this.env.navigation.push('/login')
  }

  /**
   * 导航到首页
   */
  async navigateToHome(): Promise<void> {
    await this.env.navigation.push('/')
  }
}
