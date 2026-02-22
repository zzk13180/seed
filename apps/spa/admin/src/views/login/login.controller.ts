import type { LoginState, LoginDeps, ValidationResult, UserVO } from './login.types'

/**
 * LoginController - 纯 TypeScript 类，不依赖任何框架
 *
 * 职责：
 * - 管理登录表单状态
 * - 处理登录逻辑
 * - 表单验证
 * - 通过 Deps 注入执行副作用
 *
 * Better Auth 使用 email + password 登录:
 * - 登录成功后浏览器自动管理 session cookie
 * - 无需手动管理 token
 */
export class LoginController {
  constructor(
    private readonly state: LoginState,
    private readonly deps: LoginDeps,
  ) {}

  /**
   * 获取是否正在加载
   */
  get isLoading(): boolean {
    return this.state.loading
  }

  /**
   * 初始化 - 加载保存的凭据
   */
  initialize(): void {
    this.deps.logger.debug('LoginController initialized')
    this.loadSavedCredentials()
  }

  /**
   * 销毁
   */
  dispose(): void {
    this.deps.logger.debug('LoginController disposed')
    this.clearError()
  }

  /**
   * 验证表单
   */
  validate(): ValidationResult {
    const errors: ValidationResult['errors'] = {}

    if (!this.state.form.email.trim()) {
      errors.email = '请输入您的邮箱'
    } else if (!/\S[^\s@]*@\S+\.\S+/.test(this.state.form.email)) {
      errors.email = '请输入有效的邮箱地址'
    }

    if (!this.state.form.password.trim()) {
      errors.password = '请输入您的密码'
    }

    return {
      valid: Object.keys(errors).length === 0,
      errors,
    }
  }

  /**
   * 执行登录
   */
  async login(): Promise<UserVO | null> {
    // 验证表单
    const validation = this.validate()
    if (!validation.valid) {
      const firstError = validation.errors.email || validation.errors.password
      if (firstError) {
        this.state.errorMessage = firstError
      }
      return null
    }

    this.state.loading = true
    this.clearError()
    this.deps.logger.info('Login attempt', { email: this.state.form.email })

    try {
      const user = await this.deps.authService.login({
        email: this.state.form.email,
        password: this.state.form.password,
      })

      // Admin 后台仅允许管理员登录
      if (user.role !== 'admin') {
        // 登出 session，阻止非管理员进入
        await this.deps.authService.logout()
        this.state.errorMessage = '权限不足：仅管理员可登录后台'
        this.deps.logger.warn('Non-admin login rejected', { userId: user.id, role: user.role })
        return null
      }

      // 处理记住密码
      if (this.state.form.rememberMe) {
        this.saveCredentials()
      } else {
        this.clearCredentials()
      }

      this.deps.messageService.success('登录成功')
      this.deps.logger.info('Login successful', { userId: user.id })

      // 导航到首页
      await this.deps.navigation.push('/')

      return user
    } catch (error) {
      // 使用统一的错误处理服务获取用户友好的错误消息
      const errorMessage = this.deps.errorHandler.getUserMessage(error)
      this.state.errorMessage = errorMessage
      this.deps.logger.error('Login failed', error)
      return null
    } finally {
      this.state.loading = false
    }
  }

  /**
   * 更新邮箱
   */
  setEmail(email: string): void {
    this.state.form.email = email
  }

  /**
   * 更新密码
   */
  setPassword(password: string): void {
    this.state.form.password = password
  }

  /**
   * 更新记住密码
   */
  setRememberMe(rememberMe: boolean): void {
    this.state.form.rememberMe = rememberMe
  }

  /**
   * 清除错误信息
   */
  clearError(): void {
    this.state.errorMessage = null
  }

  /**
   * 加载保存的登录凭据
   */
  private loadSavedCredentials(): void {
    const email = this.deps.storageService.get('login_email')
    const password = this.deps.storageService.get('login_password')
    const rememberMe = this.deps.storageService.get('rememberMe')

    if (email && password && rememberMe === 'true') {
      this.state.form.email = email
      this.state.form.password = password
      this.state.form.rememberMe = true
      this.deps.logger.debug('Loaded saved credentials')
    }
  }

  /**
   * 保存凭据
   */
  private saveCredentials(): void {
    this.deps.storageService.set('login_email', this.state.form.email)
    this.deps.storageService.set('login_password', this.state.form.password)
    this.deps.storageService.set('rememberMe', 'true')
    this.deps.logger.debug('Credentials saved')
  }

  /**
   * 清除凭据
   */
  private clearCredentials(): void {
    this.deps.storageService.remove('login_email')
    this.deps.storageService.remove('login_password')
    this.deps.storageService.remove('rememberMe')
    this.deps.logger.debug('Credentials cleared')
  }
}
