/**
 * @file 错误处理服务
 * @description 提供统一的错误处理和用户友好提示
 * @module core/error.service
 */

import { HttpError } from '@seed/http'

/**
 * 错误类型枚举
 */
export enum ErrorType {
  /** 网络错误 - 无法连接到服务器 */
  NETWORK_ERROR = 'NETWORK_ERROR',
  /** 超时错误 */
  TIMEOUT_ERROR = 'TIMEOUT_ERROR',
  /** 服务不可用 */
  SERVICE_UNAVAILABLE = 'SERVICE_UNAVAILABLE',
  /** 未授权 */
  UNAUTHORIZED = 'UNAUTHORIZED',
  /** 禁止访问 */
  FORBIDDEN = 'FORBIDDEN',
  /** 资源未找到 */
  NOT_FOUND = 'NOT_FOUND',
  /** 客户端错误 */
  CLIENT_ERROR = 'CLIENT_ERROR',
  /** 服务器错误 */
  SERVER_ERROR = 'SERVER_ERROR',
  /** 未知错误 */
  UNKNOWN_ERROR = 'UNKNOWN_ERROR',
}

/**
 * 错误信息接口
 */
export interface ErrorInfo {
  type: ErrorType
  message: string
  title: string
  status: number
  originalError?: unknown
  /** 是否应该显示给用户 */
  shouldNotify: boolean
  /** 是否需要重新登录 */
  requiresReLogin: boolean
}

/**
 * 错误消息映射表 - 用户友好的中文提示
 */
export const ERROR_MESSAGES: Record<ErrorType, { title: string; message: string }> = {
  [ErrorType.NETWORK_ERROR]: {
    title: '网络连接失败',
    message: '无法连接到服务器，请检查您的网络连接或稍后重试',
  },
  [ErrorType.TIMEOUT_ERROR]: {
    title: '请求超时',
    message: '服务器响应超时，请稍后重试',
  },
  [ErrorType.SERVICE_UNAVAILABLE]: {
    title: '服务暂不可用',
    message: '服务器正在维护中，请稍后重试',
  },
  [ErrorType.UNAUTHORIZED]: {
    title: '登录已过期',
    message: '您的登录状态已过期，请重新登录',
  },
  [ErrorType.FORBIDDEN]: {
    title: '无权访问',
    message: '您没有权限执行此操作',
  },
  [ErrorType.NOT_FOUND]: {
    title: '资源不存在',
    message: '请求的资源不存在或已被删除',
  },
  [ErrorType.CLIENT_ERROR]: {
    title: '请求错误',
    message: '请求参数错误，请检查后重试',
  },
  [ErrorType.SERVER_ERROR]: {
    title: '服务器错误',
    message: '服务器内部错误，请稍后重试',
  },
  [ErrorType.UNKNOWN_ERROR]: {
    title: '未知错误',
    message: '发生未知错误，请稍后重试',
  },
}

/**
 * 错误处理服务接口
 */
export interface ErrorHandler {
  /**
   * 解析错误并返回用户友好的错误信息
   */
  parseError(error: unknown): ErrorInfo

  /**
   * 获取错误类型
   */
  getErrorType(error: unknown): ErrorType

  /**
   * 获取用户友好的错误消息
   */
  getUserMessage(error: unknown): string
}

/**
 * 默认错误处理服务实现
 */
export class DefaultErrorHandler implements ErrorHandler {
  /**
   * 解析错误并返回用户友好的错误信息
   */
  parseError(error: unknown): ErrorInfo {
    const type = this.getErrorType(error)
    const { title, message } = this.getDefaultMessage(type)
    const status = this.getStatusCode(error)

    // 尝试从后端响应中提取更具体的错误信息
    const serverMessage = this.extractServerMessage(error)

    return {
      type,
      title,
      message: serverMessage || message,
      status,
      originalError: error,
      shouldNotify: this.shouldNotifyUser(type),
      requiresReLogin: type === ErrorType.UNAUTHORIZED,
    }
  }

  /**
   * 获取错误类型
   */
  getErrorType(error: unknown): ErrorType {
    if (error instanceof HttpError) {
      return this.mapStatusToErrorType(error.status, error.message)
    }

    if (error instanceof Error) {
      // 检查网络错误
      if (this.isNetworkError(error)) {
        return ErrorType.NETWORK_ERROR
      }
      // 检查超时错误
      if (this.isTimeoutError(error)) {
        return ErrorType.TIMEOUT_ERROR
      }
    }

    return ErrorType.UNKNOWN_ERROR
  }

  /**
   * 获取用户友好的错误消息
   */
  getUserMessage(error: unknown): string {
    const errorInfo = this.parseError(error)
    return errorInfo.message
  }

  /**
   * 根据 HTTP 状态码映射错误类型
   */
  private mapStatusToErrorType(status: number, message?: string): ErrorType {
    // 状态码为 0 通常表示网络错误或超时
    if (status === 0) {
      if (message?.toLowerCase().includes('timeout')) {
        return ErrorType.TIMEOUT_ERROR
      }
      return ErrorType.NETWORK_ERROR
    }

    // 4xx 客户端错误
    if (status === 401) {
      return ErrorType.UNAUTHORIZED
    }
    if (status === 403) {
      return ErrorType.FORBIDDEN
    }
    if (status === 404) {
      return ErrorType.NOT_FOUND
    }
    if (status >= 400 && status < 500) {
      return ErrorType.CLIENT_ERROR
    }

    // 5xx 服务器错误
    if (status === 502 || status === 503 || status === 504) {
      return ErrorType.SERVICE_UNAVAILABLE
    }
    if (status >= 500) {
      return ErrorType.SERVER_ERROR
    }

    return ErrorType.UNKNOWN_ERROR
  }

  /**
   * 检查是否为网络错误
   */
  private isNetworkError(error: Error): boolean {
    const message = error.message.toLowerCase()
    return (
      message.includes('network') ||
      message.includes('failed to fetch') ||
      message.includes('net::err') ||
      message.includes('networkerror') ||
      message.includes('connection refused') ||
      error.name === 'TypeError' // fetch 网络错误通常是 TypeError
    )
  }

  /**
   * 检查是否为超时错误
   */
  private isTimeoutError(error: Error): boolean {
    const message = error.message.toLowerCase()
    return (
      message.includes('timeout') || message.includes('timed out') || error.name === 'AbortError'
    )
  }

  /**
   * 从后端响应中提取错误信息
   */
  private extractServerMessage(error: unknown): string | null {
    if (!(error instanceof HttpError) || !error.data) {
      return null
    }

    const data = error.data as Record<string, unknown>

    // 尝试多种常见的后端响应格式
    if (typeof data.message === 'string') {
      return data.message
    }
    if (typeof data.msg === 'string') {
      return data.msg
    }
    if (typeof data.error === 'string') {
      return data.error
    }
    if (typeof data.errorMessage === 'string') {
      return data.errorMessage
    }

    return null
  }

  /**
   * 获取 HTTP 状态码
   */
  private getStatusCode(error: unknown): number {
    if (error instanceof HttpError) {
      return error.status
    }
    return 0
  }

  /**
   * 获取默认错误消息
   */
  private getDefaultMessage(type: ErrorType): { title: string; message: string } {
    return ERROR_MESSAGES[type] || ERROR_MESSAGES[ErrorType.UNKNOWN_ERROR]
  }

  /**
   * 判断是否应该通知用户
   * 某些错误（如静默刷新失败）可能不需要立即通知
   */
  private shouldNotifyUser(type: ErrorType): boolean {
    // 所有错误类型默认都应该通知用户
    return true
  }
}

/**
 * 创建默认错误处理器实例
 */
export function createErrorHandler(): ErrorHandler {
  return new DefaultErrorHandler()
}

/**
 * 默认错误处理器单例
 */
export const errorHandler = createErrorHandler()
