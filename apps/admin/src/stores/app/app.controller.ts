import type { AppState, AppEnv, Theme } from './app.types'

/**
 * AppController - 纯 TypeScript 类，不依赖任何框架
 *
 * 职责：
 * - 管理应用全局状态（侧边栏、主题等）
 * - 通过 Env 注入执行副作用（DOM 操作等）
 */
export class AppController {
  constructor(
    private readonly state: AppState,
    private readonly env: AppEnv,
  ) {}

  /**
   * 获取当前主题
   */
  get currentTheme(): Theme {
    return this.state.theme
  }

  /**
   * 获取侧边栏是否隐藏
   */
  get isSidebarHidden(): boolean {
    return this.state.sidebarIsHidden
  }

  /**
   * 获取侧边栏是否折叠
   */
  get isSidebarCollapsed(): boolean {
    return this.state.sidebarCollapsed
  }

  /**
   * 设置侧边栏显示/隐藏
   */
  setSidebarIsHidden(isHidden: boolean): void {
    this.state.sidebarIsHidden = isHidden
    this.env.logger.debug('Sidebar visibility changed', { isHidden })
  }

  /**
   * 切换侧边栏显示/隐藏
   */
  toggleSidebar(): void {
    this.state.sidebarIsHidden = !this.state.sidebarIsHidden
    this.env.logger.debug('Sidebar toggled', { isHidden: this.state.sidebarIsHidden })
  }

  /**
   * 设置侧边栏折叠状态
   */
  setSidebarCollapsed(collapsed: boolean): void {
    this.state.sidebarCollapsed = collapsed
    this.env.logger.debug('Sidebar collapsed changed', { collapsed })
  }

  /**
   * 切换侧边栏折叠状态
   */
  toggleSidebarCollapsed(): void {
    this.state.sidebarCollapsed = !this.state.sidebarCollapsed
    this.env.logger.debug('Sidebar collapsed toggled', { collapsed: this.state.sidebarCollapsed })
  }

  /**
   * 设置页面标题
   */
  setTitle(title: string): void {
    this.state.title = title
    this.env.themeService.setDocumentTitle(title)
    this.env.logger.debug('Title changed', { title })
  }

  /**
   * 设置主题
   */
  setTheme(theme: Theme): void {
    this.state.theme = theme
    this.env.themeService.applyTheme(theme)
    this.env.logger.debug('Theme changed', { theme })
  }
}
