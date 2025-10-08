/**
 * @seed/tailwind-config - 共享 Tailwind CSS 预设
 *
 * 使用方式：
 * import seedPreset from '@seed/tailwind-config/preset'
 * export default {
 *   presets: [seedPreset],
 *   content: [...],
 * }
 */

export default {
  theme: {
    extend: {
      // 统一的颜色系统
      colors: {
        primary: 'var(--seed-color-primary, #3b82f6)',
        secondary: 'var(--seed-color-secondary, #64748b)',
        accent: 'var(--seed-color-accent, #f59e0b)',
        success: 'var(--seed-color-success, #22c55e)',
        warning: 'var(--seed-color-warning, #eab308)',
        error: 'var(--seed-color-error, #ef4444)',
      },
      // 统一的字体系统
      fontFamily: {
        sans: [
          'Inter',
          'ui-sans-serif',
          'system-ui',
          '-apple-system',
          'BlinkMacSystemFont',
          'Segoe UI',
          'Roboto',
          'Helvetica Neue',
          'Arial',
          'sans-serif',
        ],
        mono: [
          'JetBrains Mono',
          'ui-monospace',
          'SFMono-Regular',
          'Menlo',
          'Monaco',
          'Consolas',
          'Liberation Mono',
          'Courier New',
          'monospace',
        ],
      },
      // 统一的断点（与 Tailwind 默认一致，可根据需要调整）
      screens: {
        xs: '475px',
        sm: '640px',
        md: '768px',
        lg: '1024px',
        xl: '1280px',
        '2xl': '1536px',
      },
      // 统一的动画
      animation: {
        'fade-in': 'fadeIn 0.3s ease-in-out',
        'fade-out': 'fadeOut 0.3s ease-in-out',
        'slide-in': 'slideIn 0.3s ease-out',
        'spin-slow': 'spin 3s linear infinite',
      },
      keyframes: {
        fadeIn: {
          '0%': { opacity: '0' },
          '100%': { opacity: '1' },
        },
        fadeOut: {
          '0%': { opacity: '1' },
          '100%': { opacity: '0' },
        },
        slideIn: {
          '0%': { opacity: '0', transform: 'translateY(-10px)' },
          '100%': { opacity: '1', transform: 'translateY(0)' },
        },
      },
      // 统一的间距扩展
      spacing: {
        18: '4.5rem',
        88: '22rem',
        128: '32rem',
      },
      // 统一的圆角
      borderRadius: {
        '4xl': '2rem',
      },
    },
  },
  // 暗色模式使用 class 策略
  darkMode: 'class',
}
