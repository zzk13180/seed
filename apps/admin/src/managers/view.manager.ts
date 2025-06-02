import { Subject } from 'rxjs'
import type { Observable } from 'rxjs'

export interface ViewState {
  viewCenter: { x: number; y: number } // 视图中心点坐标
  viewScale: number // 视图缩放级别
  inputMovement: boolean // 是否允许用户通过输入设备移动视图
}

export interface ViewManagerOptions {
  /**
   * 可选的回调函数，用于判断是否应阻止对给定目标元素的拖动操作。
   */
  shouldPreventDrag?: (target: EventTarget | null) => boolean
}

export class ViewManager {
  private readonly MAX_SCALE = 5000 // 最大允许的缩放级别
  private readonly MIN_SCALE = 0.001 // 最小允许的缩放级别
  private readonly ZOOM_FACTOR = 1.05 // 每次缩放的倍率因子

  private state: ViewState // 视图状态
  container: HTMLElement // 容器元素，用于监听事件和获取尺寸
  private options: ViewManagerOptions // 视图管理器配置选项

  // 拖拽开始时的状态记录
  private drag_start?: { x: number; y: number; ref_center_x: number; ref_center_y: number }

  // 多点触控的第一个触摸点
  private touch1: { clientX: number; clientY: number } | null = null
  // 多点触控的第二个触摸点
  private touch2: { clientX: number; clientY: number } | null = null
  // 初始触摸点之间的距离，用于计算缩放
  private initialTouchDistance: number = 0

  // 事件处理时间戳，用于事件节流
  private event_timestamp: number = performance.now()
  // 事件节流的超时ID
  private event_timeout: number | undefined = undefined

  private containerWidth: number = 0 // 容器宽度的缓存
  private containerHeight: number = 0 // 容器高度的缓存
  private resizeObserver: ResizeObserver | null = null // 用于监听容器尺寸变化的 ResizeObserver

  // 绑定的事件处理函数
  private _boundHandlers: {
    dragStart: (event: MouseEvent | TouchEvent) => void
    dragMove: (event: MouseEvent | TouchEvent) => void
    dragEnd: () => void
    zoom: (event: WheelEvent | TouchEvent) => void
    touchStart: (event: TouchEvent) => void
    touchMove: (event: TouchEvent) => void
    touchEnd: (event: TouchEvent) => void
    handleResize: () => void
  }

  // 添加一个 Subject 用于发出视图变化事件
  #viewChangeSubject = new Subject<ViewState>()

  constructor(state: ViewState, container: HTMLElement, options: ViewManagerOptions = {}) {
    this.container = container
    this.state = state
    this.options = options

    this._boundHandlers = {
      dragStart: this.handleDragStart.bind(this),
      dragMove: this.handleDragMove.bind(this),
      dragEnd: this.handleDragEnd.bind(this),
      zoom: this.handleZoom.bind(this),
      touchStart: this._handleTouchStart.bind(this),
      touchMove: this._handleTouchMove.bind(this),
      touchEnd: this._handleTouchEnd.bind(this),
      handleResize: this._handleResize.bind(this),
    }

    this.addListeners()
    this.setupResizeObserver()
    this.updateContainerDimensions()
  }

  initialize(): void {
    // 初始化时发送一次视图状态
    this.#viewChangeSubject.next(this.getState())
  }

  destroy(): void {
    this.removeListeners()
    if (this.resizeObserver) {
      this.resizeObserver.disconnect()
      this.resizeObserver = null
    }
    this.#viewChangeSubject.complete() // 完成 Subject，释放资源
  }

  // 暴露一个 Observable 供外部订阅
  get viewChange$(): Observable<ViewState> {
    return this.#viewChangeSubject.asObservable()
  }

  get getContainerWidth(): number {
    return this.containerWidth
  }

  get getContainerHeight(): number {
    return this.containerHeight
  }

  /**
   * 获取当前视图状态
   */
  getState(): ViewState {
    return { ...this.state }
  }

  /**
   * 设置视图状态
   */
  setState(state: ViewState): void {
    this.state = { ...state }
    this.sendUpdateEvent()
  }

  /**
   * 添加事件监听器
   */
  addListeners(): void {
    this.container.addEventListener('mousedown', this._boundHandlers.dragStart)
    this.container.addEventListener('mousemove', this._boundHandlers.dragMove)
    this.container.addEventListener('mouseup', this._boundHandlers.dragEnd)
    this.container.addEventListener('mouseleave', this._boundHandlers.dragEnd)
    this.container.addEventListener('wheel', this._boundHandlers.zoom, { passive: false })

    this.container.addEventListener('touchstart', this._boundHandlers.touchStart, {
      passive: false,
    })
    this.container.addEventListener('touchmove', this._boundHandlers.touchMove, { passive: false })
    this.container.addEventListener('touchend', this._boundHandlers.touchEnd)
    this.container.addEventListener('touchcancel', this._boundHandlers.touchEnd)
  }

  /**
   * 移除事件监听器
   */
  removeListeners(): void {
    this.container.removeEventListener('mousedown', this._boundHandlers.dragStart)
    this.container.removeEventListener('mousemove', this._boundHandlers.dragMove)
    this.container.removeEventListener('mouseup', this._boundHandlers.dragEnd)
    this.container.removeEventListener('mouseleave', this._boundHandlers.dragEnd)
    this.container.removeEventListener('wheel', this._boundHandlers.zoom)

    this.container.removeEventListener('touchstart', this._boundHandlers.touchStart)
    this.container.removeEventListener('touchmove', this._boundHandlers.touchMove)
    this.container.removeEventListener('touchend', this._boundHandlers.touchEnd)
    this.container.removeEventListener('touchcancel', this._boundHandlers.touchEnd)

    if (this.resizeObserver) {
      this.resizeObserver.disconnect()
      this.resizeObserver = null
    } else {
      window.removeEventListener('resize', this._boundHandlers.handleResize)
    }
  }

  /**
   * 设置是否允许通过输入（鼠标/触摸）移动视图
   * @param value 是否允许移动
   */
  setInputMovementEnabled(value: boolean): void {
    this.state.inputMovement = value
    if (!value) {
      this.drag_start = undefined
    }
  }

  /**
   * 重置视图到默认中心和缩放级别
   * @param defaultCenter 可选的默认中心点
   * @param defaultScale 可选的默认缩放级别
   */
  resetView(defaultCenter = { x: 0, y: 0 }, defaultScale = 50.0): void {
    this.state.viewCenter = { ...defaultCenter }
    this.state.viewScale = defaultScale
    this.drag_start = undefined
    this.sendUpdateEvent()
  }

  /**
   * 将固定坐标（地图坐标）转换为屏幕坐标
   */
  fixedToScreen(coords: { x: number; y: number }): { x: number; y: number; z: number } {
    return {
      x: (coords.x - this.state.viewCenter.x) * this.state.viewScale + this.containerWidth / 2,
      y: (-coords.y - this.state.viewCenter.y) * this.state.viewScale + this.containerHeight / 2,
      z: 0,
    }
  }

  /**
   * 将屏幕坐标转换为固定坐标（地图坐标）
   */
  screenToFixed(coords: { x: number; y: number }): { x: number; y: number; z: number } {
    return {
      x: (coords.x - this.containerWidth / 2) / this.state.viewScale + this.state.viewCenter.x,
      y: -((coords.y - this.containerHeight / 2) / this.state.viewScale) - this.state.viewCenter.y,
      z: 0,
    }
  }

  /**
   * 获取指定屏幕像素长度在地图单位中的对应长度
   */
  getPixelsInMapUnits(length: number): number {
    const p1 = this.screenToFixed({ x: 0, y: 0 })
    const p2 = this.screenToFixed({ x: length, y: 0 })
    return Math.abs(p1.x - p2.x)
  }

  /**
   * 获取指定地图单位长度在屏幕上的对应像素长度
   */
  getMapUnitsInPixels(length: number): number {
    const p1 = this.fixedToScreen({ x: 0, y: 0 })
    const p2 = this.fixedToScreen({ x: length, y: 0 })
    return Math.abs(p1.x - p2.x)
  }

  // 更新容器尺寸缓存
  private updateContainerDimensions(): void {
    if (this.container) {
      this.containerWidth = this.container.clientWidth
      this.containerHeight = this.container.clientHeight
    }
  }

  // 处理容器尺寸变化事件
  private _handleResize(): void {
    this.updateContainerDimensions()
    this.#viewChangeSubject.next(this.getState())
  }

  // 设置 ResizeObserver 监听容器尺寸变化
  private setupResizeObserver(): void {
    if (typeof ResizeObserver !== 'undefined') {
      this.resizeObserver = new ResizeObserver(this._boundHandlers.handleResize)
      this.resizeObserver.observe(this.container)
    } else {
      // ResizeObserver 不支持时的后备方案
      window.addEventListener('resize', this._boundHandlers.handleResize)
    }
  }

  // 发送视图更新事件
  private sendUpdateEvent(): void {
    if (this.event_timeout !== undefined) {
      clearTimeout(this.event_timeout)
    }
    const delta = performance.now() - this.event_timestamp

    if (delta > 12) {
      this.#viewChangeSubject.next(this.getState())
      this.event_timestamp = performance.now()
    } else {
      this.event_timeout = window.setTimeout(() => {
        this.#viewChangeSubject.next(this.getState())
        this.event_timeout = undefined
      }, 12 - delta)
    }
  }

  private handleDragStart(event: MouseEvent | TouchEvent): void {
    if (!this.state.inputMovement) return

    if (this.options.shouldPreventDrag && this.options.shouldPreventDrag((event as Event).target)) {
      return
    }

    if ('touches' in event) {
      if (event.touches.length === 1) {
        event.preventDefault()
      }
    }

    const clientX = 'touches' in event ? event.touches[0].clientX : event.clientX
    const clientY = 'touches' in event ? event.touches[0].clientY : event.clientY

    this.drag_start = {
      x: clientX,
      y: clientY,
      ref_center_x: this.state.viewCenter.x,
      ref_center_y: this.state.viewCenter.y,
    }
  }

  private handleDragMove(event: MouseEvent | TouchEvent): void {
    if (this.drag_start === undefined) return
    if (!this.state.inputMovement) {
      this.drag_start = undefined
      return
    }

    if (this.options.shouldPreventDrag && this.options.shouldPreventDrag((event as Event).target)) {
      this.drag_start = undefined
      return
    }

    if ('touches' in event && event.touches.length === 1) {
      event.preventDefault()
    }

    const clientX = 'touches' in event ? event.touches[0].clientX : event.clientX
    const clientY = 'touches' in event ? event.touches[0].clientY : event.clientY

    const delta = {
      x: this.drag_start.x - clientX,
      y: this.drag_start.y - clientY,
    }

    this.state.viewCenter = {
      x: this.drag_start.ref_center_x + delta.x / this.state.viewScale,
      y: this.drag_start.ref_center_y + delta.y / this.state.viewScale,
    }
    this.sendUpdateEvent()
  }

  private handleDragEnd(): void {
    this.drag_start = undefined
  }

  private clamp(value: number, min: number, max: number): number {
    return Math.max(min, Math.min(value, max))
  }

  private handleZoom(event: WheelEvent | TouchEvent): void {
    if (!this.state.inputMovement) return

    event.preventDefault()

    let scaleChange: number
    let clientX: number // Cursor X relative to viewport
    let clientY: number // Cursor Y relative to viewport

    const oldScale = this.state.viewScale

    if ('touches' in event) {
      if (event.touches.length !== 2) return
      const touch1 = event.touches[0]
      const touch2 = event.touches[1]

      clientX = (touch1.clientX + touch2.clientX) / 2
      clientY = (touch1.clientY + touch2.clientY) / 2

      const dx = touch1.clientX - touch2.clientX
      const dy = touch1.clientY - touch2.clientY
      const currentDistance = Math.sqrt(dx * dx + dy * dy)

      if (this.initialTouchDistance === 0) {
        this.initialTouchDistance = currentDistance
        return
      }

      scaleChange = currentDistance / this.initialTouchDistance
      const newScaleAttempt = oldScale * scaleChange
      this.state.viewScale = this.clamp(newScaleAttempt, this.MIN_SCALE, this.MAX_SCALE)

      if (oldScale === 0)
        scaleChange = 1 // Avoid division by zero if oldScale was 0
      else scaleChange = this.state.viewScale / oldScale

      this.initialTouchDistance = currentDistance
    } else {
      // eslint-disable-next-line @typescript-eslint/no-unnecessary-type-assertion
      const wheelEvent = event as WheelEvent
      const new_scale_attempt =
        oldScale * (wheelEvent.deltaY < 0 ? this.ZOOM_FACTOR : 1 / this.ZOOM_FACTOR)
      this.state.viewScale = this.clamp(new_scale_attempt, this.MIN_SCALE, this.MAX_SCALE)

      if (oldScale === 0)
        scaleChange = 1 // Avoid division by zero
      else scaleChange = this.state.viewScale / oldScale

      clientX = wheelEvent.clientX
      clientY = wheelEvent.clientY
    }

    if (Math.abs(scaleChange - 1) < 1e-9 && oldScale === this.state.viewScale) {
      // Scale did not effectively change (e.g. clamped at min/max)
      // Only send update if there was an attempt to change, to reflect potential clamping.
      // If scale truly didn't change and wasn't at a boundary, no need to update.
      // However, the original logic always proceeded. For safety and consistency with original, we proceed.
      // If scaleChange is exactly 1, the center adjustment formula results in no change.
    }

    // Convert viewport clientX/Y to coordinates relative to the container for screenToFixed
    const containerRect = this.container.getBoundingClientRect()
    const screenPoint = {
      x: clientX - containerRect.left,
      y: clientY - containerRect.top,
    }
    const mapPoint = this.screenToFixed(screenPoint)

    this.state.viewCenter = {
      x: this.state.viewCenter.x + (mapPoint.x - this.state.viewCenter.x) * (1 - 1 / scaleChange),
      y: this.state.viewCenter.y + (-mapPoint.y - this.state.viewCenter.y) * (1 - 1 / scaleChange),
    }

    this.sendUpdateEvent()
  }

  private _handleTouchStart(event: TouchEvent): void {
    if (!this.state.inputMovement) return

    if (this.options.shouldPreventDrag && this.options.shouldPreventDrag((event as Event).target)) {
      return
    }

    if (event.touches.length === 2) {
      this.drag_start = undefined // Stop any single-touch drag

      this.touch1 = { clientX: event.touches[0].clientX, clientY: event.touches[0].clientY }
      this.touch2 = { clientX: event.touches[1].clientX, clientY: event.touches[1].clientY }
      const dx = this.touch1.clientX - this.touch2.clientX
      const dy = this.touch1.clientY - this.touch2.clientY
      this.initialTouchDistance = Math.sqrt(dx * dx + dy * dy)
    } else if (event.touches.length === 1) {
      this.touch1 = { clientX: event.touches[0].clientX, clientY: event.touches[0].clientY }
      this.touch2 = null
      this.initialTouchDistance = 0
      this.handleDragStart(event)
    }
  }

  private _handleTouchMove(event: TouchEvent): void {
    if (!this.state.inputMovement) return

    if (event.touches.length === 2 && this.touch1 && this.touch2) {
      if (this.drag_start) this.drag_start = undefined // Ensure drag stops if transitioning to pinch
      this.handleZoom(event)
    } else if (event.touches.length === 1 && this.drag_start) {
      this.handleDragMove(event)
    }
  }

  private _handleTouchEnd(event: TouchEvent): void {
    if (event.touches.length < 2) {
      this.touch1 = null
      this.touch2 = null
      this.initialTouchDistance = 0
    }
    if (event.touches.length < 1) {
      this.handleDragEnd()
    }
  }
}
