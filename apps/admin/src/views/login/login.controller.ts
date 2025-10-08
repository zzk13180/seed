import type { LoginState, LoginEnv, ValidationResult, UserInfo } from './login.types'

/**
 * LoginController - 纯 TypeScript 类，不依赖任何框架
 *
 * 职责：
 * - 管理登录表单状态
 * - 处理登录逻辑
 * - 表单验证
 * - 通过 Env 注入执行副作用
 */
export class LoginController {
  constructor(
    private readonly state: LoginState,
    private readonly env: LoginEnv,
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
    this.env.logger.debug('LoginController initialized')
    this.loadSavedCredentials()
  }

  /**
   * 销毁
   */
  dispose(): void {
    this.env.logger.debug('LoginController disposed')
    this.clearError()
  }

  /**
   * 验证表单
   */
  validate(): ValidationResult {
    const errors: ValidationResult['errors'] = {}

    if (!this.state.form.username.trim()) {
      errors.username = '请输入您的账号'
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
  async login(): Promise<UserInfo | null> {
    // 验证表单
    const validation = this.validate()
    if (!validation.valid) {
      const firstError = validation.errors.username || validation.errors.password
      if (firstError) {
        this.state.errorMessage = firstError
      }
      return null
    }

    this.state.loading = true
    this.clearError()
    this.env.logger.info('Login attempt', { username: this.state.form.username })

    try {
      const result = await this.env.authService.login({
        username: this.state.form.username,
        password: this.state.form.password,
      })

      // 处理记住密码
      if (this.state.form.rememberMe) {
        this.saveCredentials()
      } else {
        this.clearCredentials()
      }

      this.env.messageService.success('登录成功')
      this.env.logger.info('Login successful', { userId: result.user.id })

      // 导航到首页
      await this.env.navigation.push('/')

      return result.user
    } catch (error) {
      // 使用统一的错误处理服务获取用户友好的错误消息
      const errorMessage = this.env.errorHandler.getUserMessage(error)
      this.state.errorMessage = errorMessage
      // 注意：全局错误拦截器已经显示了错误通知，这里不需要再调用 messageService
      this.env.logger.error('Login failed', error)
      return null
    } finally {
      this.state.loading = false
    }
  }

  /**
   * 更新用户名
   */
  setUsername(username: string): void {
    this.state.form.username = username
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
    const username = this.env.storageService.get('username')
    const password = this.env.storageService.get('password')
    const rememberMe = this.env.storageService.get('rememberMe')

    if (username && password && rememberMe === 'true') {
      this.state.form.username = username
      this.state.form.password = password
      this.state.form.rememberMe = true
      this.env.logger.debug('Loaded saved credentials')
    }
  }

  /**
   * 保存凭据
   */
  private saveCredentials(): void {
    this.env.storageService.set('username', this.state.form.username)
    this.env.storageService.set('password', this.state.form.password)
    this.env.storageService.set('rememberMe', 'true')
    this.env.logger.debug('Credentials saved')
  }

  /**
   * 清除凭据
   */
  private clearCredentials(): void {
    this.env.storageService.remove('username')
    this.env.storageService.remove('password')
    this.env.storageService.remove('rememberMe')
    this.env.logger.debug('Credentials cleared')
  }
}
