/** @jsxImportSource preact */
/**
 * 搜索组件
 *
 * 使用 Preact + Algolia DocSearch
 * Preact 的 compat 模式提供 React API 兼容性
 */
import { useState, useCallback, useRef } from 'preact/hooks'
import '@docsearch/css'
import './Search.css'
import { ALGOLIA } from '../../config'

// 简化版搜索组件（当 Algolia 未配置时使用）
export default function Search() {
  const [isOpen, setIsOpen] = useState(false)
  const searchButtonRef = useRef<HTMLButtonElement>(null)

  const onOpen = useCallback(() => {
    // TODO: 当配置了 Algolia 后，这里加载 DocSearch Modal
    // 目前显示提示信息
    if (ALGOLIA.appId === 'XXXXXXXXXX') {
      alert('请先配置 Algolia DocSearch。参考文档：https://docsearch.algolia.com/')
      return
    }
    setIsOpen(true)
  }, [])

  // 键盘快捷键支持
  if (typeof window !== 'undefined') {
    document.addEventListener('keydown', e => {
      if (e.key === '/' && !isOpen) {
        e.preventDefault()
        onOpen()
      }
    })
  }

  return (
    <button
      type="button"
      ref={searchButtonRef}
      onClick={onOpen}
      className="search-input"
      aria-label="Search documentation"
    >
      <svg width="24" height="24" fill="none" aria-hidden="true">
        <path
          d="M21 21l-6-6m2-5a7 7 0 11-14 0 7 7 0 0114 0z"
          stroke="currentColor"
          strokeWidth="2"
          strokeLinecap="round"
          strokeLinejoin="round"
        />
      </svg>

      <span>搜索</span>

      <span className="search-hint">
        <span className="sr-only">Press </span>
        <kbd>/</kbd>
        <span className="sr-only"> to search</span>
      </span>
    </button>
  )
}
