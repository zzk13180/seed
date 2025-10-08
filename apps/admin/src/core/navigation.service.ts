/**
 * @file 导航服务
 * @description 抽象导航操作，解耦 Controller 与 vue-router 的直接依赖
 * @module core/navigation.service
 *
 * @exports NavigationService - 导航服务接口，Controller 依赖此接口
 * @exports RouterNavigationService - Vue Router 导航实现
 * @exports LazyRouterNavigationService - 懒加载导航实现（解决循环依赖）
 */

import type { Router, RouteLocationRaw } from 'vue-router'

/**
 * 导航服务接口 - 抽象导航操作
 * Controller 通过接口依赖，不直接依赖 vue-router
 */
export interface NavigationService {
  push(to: string | RouteLocationRaw): Promise<void>
  replace(to: string | RouteLocationRaw): Promise<void>
  back(): void
  forward(): void
  go(delta: number): void
  getCurrentPath(): string
}

/**
 * Vue Router 导航服务实现
 */
export class RouterNavigationService implements NavigationService {
  constructor(private readonly router: Router) {}

  async push(to: string | RouteLocationRaw): Promise<void> {
    await this.router.push(to)
  }

  async replace(to: string | RouteLocationRaw): Promise<void> {
    await this.router.replace(to)
  }

  back(): void {
    this.router.back()
  }

  forward(): void {
    this.router.forward()
  }

  go(delta: number): void {
    this.router.go(delta)
  }

  getCurrentPath(): string {
    return this.router.currentRoute.value.path
  }
}

/**
 * 懒加载 Router 导航服务实现
 * 用于解决循环依赖问题，router 在首次使用时才获取
 */
export class LazyRouterNavigationService implements NavigationService {
  private _router: Router | null = null

  constructor(private readonly getRouter: () => Router) {}

  private get router(): Router {
    if (!this._router) {
      this._router = this.getRouter()
    }
    return this._router
  }

  async push(to: string | RouteLocationRaw): Promise<void> {
    await this.router.push(to)
  }

  async replace(to: string | RouteLocationRaw): Promise<void> {
    await this.router.replace(to)
  }

  back(): void {
    this.router.back()
  }

  forward(): void {
    this.router.forward()
  }

  go(delta: number): void {
    this.router.go(delta)
  }

  getCurrentPath(): string {
    return this.router.currentRoute.value.path
  }
}
