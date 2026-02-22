import type { Theme, ThemeService } from './app.types'

/**
 * 浏览器主题服务实现
 * 处理 DOM 相关的主题操作
 */
export class BrowserThemeService implements ThemeService {
  private readonly appName: string

  constructor(appName: string = 'Seed Admin') {
    this.appName = appName
  }

  /**
   * 应用主题到 HTML 元素
   */
  applyTheme(theme: Theme): void {
    const root = document.documentElement

    // 移除现有主题类
    root.classList.remove('light', 'dark')

    if (theme === 'system') {
      // 跟随系统主题
      const prefersDark = globalThis.matchMedia('(prefers-color-scheme: dark)').matches
      root.classList.add(prefersDark ? 'dark' : 'light')
    } else {
      root.classList.add(theme)
    }
  }

  /**
   * 设置文档标题
   */
  setDocumentTitle(title: string): void {
    document.title = title ? `${title} - ${this.appName}` : this.appName
  }
}
