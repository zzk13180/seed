import { Subject } from 'rxjs'

/**
   地图坐标系 (无限延伸):
                Y+ (向上)
                |
                |
    (-2,2)  (-1,2)  (0,2)  (1,2)  (2,2)
                |
    (-2,1)  (-1,1)  (0,1)  (1,1)  (2,1)
                |
    (-2,0)──(-1,0)──(0,0)──(1,0)──(2,0)──── X+ (向右)
                |
    (-2,-1) (-1,-1) (0,-1) (1,-1) (2,-1)
                |
    (-2,-2) (-1,-2) (0,-2) (1,-2) (2,-2)
                |

   屏幕坐标系 (容器范围内):
   (0,0) ────────────────────── X+ (向右)
     |
     |    容器可视区域
     |    (containerWidth × containerHeight)
     |
     |
     |
     Y+ (向下)
*/

export interface ViewState {
  viewCenter: { x: number; y: number } // 视图中心点坐标（地图坐标系）
  viewScale: number // 视图缩放级别（数值越大，显示越大）
  inputMovement: boolean // 是否允许用户移动/缩放地图
}

export interface ViewManagerOptions {
  /**
   * 可选的回调函数，用于判断是否应阻止对给定目标元素的拖动操作。
   * 例如：当用户点击按钮、输入框等UI元素时，应该阻止地图拖拽
   */
  shouldPreventDrag?: (target: EventTarget | null) => boolean
}

/**
 * 视图管理器类
 *
 * 主要职责：
 * 1. 管理地图视图的中心位置和缩放级别
 * 2. 处理用户的鼠标和触摸交互（拖拽、缩放）
 * 3. 提供坐标系转换功能（屏幕坐标 ↔ 地图坐标）
 * 4. 通过 RxJS Observable 发布视图状态变化事件
 *
 * 支持的交互方式：
 * - 鼠标：拖拽移动、滚轮缩放
 * - 触摸：单指拖拽、双指缩放
 */
export class ViewManager {
  container: HTMLElement

  readonly #viewStateSubject = new Subject<ViewState>()

  readonly viewStateChange$ = this.#viewStateSubject.asObservable()

  /**
   * 视图状态：包含中心点坐标、缩放级别、是否允许交互
   * 默认：中心点(0,0)，缩放级别50，允许交互
   */
  private state: ViewState = {
    viewCenter: { x: 0, y: 0 },
    viewScale: 50,
    inputMovement: true,
  }

  /*
   * 拖拽状态管理
   * 记录拖拽开始时的鼠标/触摸位置和视图中心点，用于计算拖拽偏移量
   */
  private drag_start?: {
    x: number
    y: number
    ref_center_x: number
    ref_center_y: number
  }

  /*
   * 触摸事件状态管理
   * 用于处理复杂的触摸交互：单指拖拽 vs 双指缩放
   */
  private touch1: { clientX: number; clientY: number } | null = null // 第一个触摸点
  private touch2: { clientX: number; clientY: number } | null = null // 第二个触摸点
  private initialTouchDistance: number = 0 // 双指初始距离，用于计算缩放比例

  /*
   * 性能优化相关
   * 通过节流机制避免过于频繁的状态更新事件
   */
  private event_timestamp: number = performance.now() // 上次事件处理时间戳
  private event_timeout: ReturnType<typeof requestAnimationFrame> | undefined = undefined // 用于节流的定时器ID

  /*
   * 容器尺寸缓存
   * 避免频繁访问 DOM 获取容器尺寸，提升性能
   */
  private containerWidth: number = 0
  private containerHeight: number = 0
  private resizeObserver: ResizeObserver | null = null // 监听容器尺寸变化

  /*
   * 缩放限制和配置
   * 防止用户缩放过度导致性能问题或显示异常
   */
  private readonly MAX_SCALE = 5000 // 最大缩放级别（放大5000倍）
  private readonly MIN_SCALE = 0.001 // 最小缩放级别（缩小到0.001倍）
  private readonly ZOOM_FACTOR = 1.05 // 每次滚轮缩放的倍率（5%增减）

  private readonly options: ViewManagerOptions

  /*
   * 预绑定的事件处理函数
   * 避免在添加/移除监听器时重复创建函数，确保正确移除监听器
   */
  private readonly _boundHandlers = {
    dragStart: this.handleDragStart.bind(this),
    dragMove: this.handleDragMove.bind(this),
    dragEnd: this.handleDragEnd.bind(this),
    zoom: this.handleZoom.bind(this),
    touchStart: this.handleTouchStart.bind(this),
    touchMove: this.handleTouchMove.bind(this),
    touchEnd: this.handleTouchEnd.bind(this),
    handleResize: this.handleResize.bind(this),
  }

  /**
   * 构造函数
   *
   * 初始化流程：
   * 1. 保存容器引用和配置选项
   * 2. 添加各种事件监听器（鼠标、触摸、滚轮）
   * 3. 设置容器尺寸监听器
   * 4. 更新容器尺寸缓存
   */
  constructor(container: HTMLElement, options: ViewManagerOptions = {}) {
    this.container = container
    this.options = options

    this.addListeners()
    this.setupResizeObserver()
    this.updateContainerDimensions()
  }

  get getContainerWidth(): number {
    return this.containerWidth
  }

  get getContainerHeight(): number {
    return this.containerHeight
  }

  /**
   * 销毁实例，清理资源
   *
   * 清理流程：
   * 1. 移除所有事件监听器
   * 2. 断开尺寸监听器
   * 3. 完成 RxJS Subject，防止内存泄漏
   */
  destroy(): void {
    // 清理定时器
    if (this.event_timeout !== undefined) {
      cancelAnimationFrame(this.event_timeout)
      this.event_timeout = undefined
    }

    this.removeListeners()

    if (this.resizeObserver) {
      this.resizeObserver.disconnect()
      this.resizeObserver = null
    }

    // 重置状态
    this.drag_start = undefined
    this.touch1 = null
    this.touch2 = null

    // 完成 Subject，释放资源
    this.#viewStateSubject.complete()
  }

  /**
   * 获取当前视图状态（浅拷贝）
   * 返回拷贝以防止外部直接修改内部状态
   */
  getState(): ViewState {
    return { ...this.state }
  }

  /**
   * 设置视图状态（浅拷贝）
   * 更新状态并触发变化事件通知订阅者
   */
  setState(state: ViewState): void {
    this.state = { ...state }
    this.sendUpdateEvent()
  }

  /**
   * 添加所有必要的事件监听器
   *
   * 监听事件类型：
   * - 鼠标事件：按下、移动、抬起、离开容器
   * - 滚轮事件：缩放操作
   * - 触摸事件：开始、移动、结束、取消
   */
  addListeners(): void {
    this.container.addEventListener('mousedown', this._boundHandlers.dragStart)
    this.container.addEventListener('mousemove', this._boundHandlers.dragMove)
    this.container.addEventListener('mouseup', this._boundHandlers.dragEnd)
    this.container.addEventListener('mouseleave', this._boundHandlers.dragEnd) // 鼠标离开容器时停止拖拽
    this.container.addEventListener('wheel', this._boundHandlers.zoom, {
      passive: false,
    })

    this.container.addEventListener('touchstart', this._boundHandlers.touchStart, {
      passive: false, // 需要调用 preventDefault()
    })
    this.container.addEventListener('touchmove', this._boundHandlers.touchMove, { passive: false })
    this.container.addEventListener('touchend', this._boundHandlers.touchEnd)
    this.container.addEventListener('touchcancel', this._boundHandlers.touchEnd) // 触摸被系统取消时
  }

  /**
   * 移除所有事件监听器
   * 在组件销毁时调用，防止内存泄漏
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
   *
   * 使用场景：
   * - 在显示模态对话框时暂时禁用地图交互
   * - 在执行某些操作时锁定视图
   */
  setInputMovementEnabled(value: boolean): void {
    this.state.inputMovement = value
    if (!value) {
      this.drag_start = undefined // 立即停止当前的拖拽操作
    }
  }

  /**
   * 重置视图到默认中心和缩放级别
   *
   * 使用场景：
   * - 用户点击"回到原点"按钮
   * - 初始化或重新加载地图数据
   */
  resetView(defaultCenter = { x: 0, y: 0 }, defaultScale = 50): void {
    this.state.viewCenter = { ...defaultCenter }
    this.state.viewScale = defaultScale
    this.drag_start = undefined // 停止任何进行中的拖拽
    this.sendUpdateEvent()
  }

  /**
   * 坐标系转换：地图坐标 → 屏幕坐标
   *
   * 转换公式：
   * screenX = (mapX - centerX) * scale + containerWidth/2
   * screenY = (-mapY - centerY) * scale + containerHeight/2
   *
   * 注意：Y轴翻转（地图Y轴向上为正，屏幕Y轴向下为正）
   */
  fixedToScreen(coords: { x: number; y: number }): {
    x: number
    y: number
    z: number
  } {
    const { x: mapX, y: mapY } = coords
    const { x: centerX, y: centerY } = this.state.viewCenter
    const scale = this.state.viewScale
    const width = this.containerWidth
    const height = this.containerHeight

    const x = (mapX - centerX) * scale + width / 2
    const y = (-mapY - centerY) * scale + height / 2

    return {
      x,
      y,
      z: 0,
    }
  }

  /**
   * 坐标系转换：屏幕坐标 → 地图坐标
   *
   * 转换公式（上述转换的逆运算）：
   * mapX = (screenX - containerWidth/2) / scale + centerX
   * mapY = -((screenY - containerHeight/2) / scale) - centerY
   */
  screenToFixed(coords: { x: number; y: number }): {
    x: number
    y: number
    z: number
  } {
    const { x: mapX, y: mapY } = coords
    const { x: centerX, y: centerY } = this.state.viewCenter
    const scale = this.state.viewScale
    const width = this.containerWidth
    const height = this.containerHeight

    const x = (mapX - width / 2) / scale + centerX
    const y = -((mapY - height / 2) / scale) - centerY

    return {
      x,
      y,
      z: 0,
    }
  }

  /**
   * 获取指定屏幕像素长度在地图单位中的对应长度
   * 像素 → 地图单位
   * 使用场景：根据当前缩放级别计算地图上的距离
   * 例如：在当前缩放下，10像素代表多少米？
   */
  getPixelsInMapUnits(length: number): number {
    return length / this.state.viewScale
  }

  /**
   * 获取指定地图单位长度在屏幕上的对应像素长度
   * 地图单位 → 像素
   * 使用场景：根据地图距离计算屏幕显示大小
   * 例如：地图上1米的距离在屏幕上显示多少像素？
   */
  getMapUnitsInPixels(length: number): number {
    return length * this.state.viewScale
  }

  /**
   * 更新容器尺寸缓存
   * 缓存容器尺寸避免频繁的DOM查询，提升性能
   */
  private updateContainerDimensions(): void {
    if (this.container) {
      this.containerWidth = this.container.clientWidth
      this.containerHeight = this.container.clientHeight
    }
  }

  /**
   * 处理容器尺寸变化事件
   *
   * 触发时机：
   * 1. 浏览器窗口大小改变
   * 2. 容器元素尺寸改变（通过 ResizeObserver）
   */
  private handleResize(): void {
    this.updateContainerDimensions()
    this.#viewStateSubject.next(this.getState()) // 通知尺寸变化
  }

  /**
   * 设置容器尺寸变化监听器
   *
   * 优先使用 ResizeObserver（更精确），
   * 不支持时降级到 window.resize 事件
   */
  private setupResizeObserver(): void {
    if (typeof ResizeObserver === 'undefined') {
      // ResizeObserver 不支持时的后备方案
      window.addEventListener('resize', this._boundHandlers.handleResize)
    } else {
      this.resizeObserver = new ResizeObserver(this._boundHandlers.handleResize)
      this.resizeObserver.observe(this.container)
    }
  }

  /**
   * 发送视图更新事件
   */
  private sendUpdateEvent(): void {
    if (this.event_timeout !== undefined) {
      cancelAnimationFrame(this.event_timeout)
    }

    const delta = performance.now() - this.event_timestamp

    if (delta > 16) {
      // 约60fps
      this.#viewStateSubject.next(this.getState())
      this.event_timestamp = performance.now()
    } else {
      this.event_timeout = requestAnimationFrame(() => {
        this.#viewStateSubject.next(this.getState())
        this.event_timeout = undefined
        this.event_timestamp = performance.now()
      })
    }
  }

  /**
   * 处理拖拽开始事件（鼠标按下或触摸开始）
   *
   * 处理流程：
   * 1. 检查是否允许交互和是否应阻止拖拽
   * 2. 记录开始位置和当前视图中心点
   * 3. 为后续的拖拽移动计算做准备
   */
  private handleDragStart(event: MouseEvent | TouchEvent): void {
    if (!this.state.inputMovement) return

    // 检查是否应该阻止拖拽（例如：用户点击了按钮）
    if (this.options.shouldPreventDrag?.((event as Event).target)) {
      return
    }

    // 单指触摸时阻止默认行为（防止页面滚动）
    if ('touches' in event && event.touches.length === 1) {
      event.preventDefault()
    }

    // 获取鼠标或触摸点的客户端坐标
    const clientX = 'touches' in event ? event.touches[0]!.clientX : event.clientX
    const clientY = 'touches' in event ? event.touches[0]!.clientY : event.clientY

    // 记录拖拽开始状态
    this.drag_start = {
      x: clientX,
      y: clientY,
      ref_center_x: this.state.viewCenter.x, // 拖拽开始时的视图中心X
      ref_center_y: this.state.viewCenter.y, // 拖拽开始时的视图中心Y
    }
  }

  /**
   * 处理拖拽移动事件（鼠标移动或触摸移动）
   *
   * 计算逻辑：
   * 1. 计算当前位置与拖拽开始位置的偏移
   * 2. 将像素偏移转换为地图坐标偏移
   * 3. 更新视图中心点位置
   */
  private handleDragMove(event: MouseEvent | TouchEvent): void {
    if (this.drag_start === undefined) return
    if (!this.state.inputMovement) {
      this.drag_start = undefined
      return
    }

    // 检查是否应该停止拖拽
    if (this.options.shouldPreventDrag?.((event as Event).target)) {
      this.drag_start = undefined
      return
    }

    // 单指触摸时阻止默认行为
    if ('touches' in event && event.touches.length === 1) {
      event.preventDefault()
    }

    // 获取当前鼠标或触摸点位置
    const clientX = 'touches' in event ? event.touches[0]!.clientX : event.clientX
    const clientY = 'touches' in event ? event.touches[0]!.clientY : event.clientY

    // 计算拖拽偏移量（像素）
    const delta = {
      x: this.drag_start.x - clientX,
      y: this.drag_start.y - clientY,
    }

    // 将像素偏移转换为地图坐标偏移，并更新视图中心
    this.state.viewCenter = {
      x: this.drag_start.ref_center_x + delta.x / this.state.viewScale,
      y: this.drag_start.ref_center_y + delta.y / this.state.viewScale,
    }
    this.sendUpdateEvent()
  }

  /**
   * 处理拖拽结束事件（鼠标抬起、触摸结束或鼠标离开容器）
   * 简单地清除拖拽状态
   */
  private handleDragEnd(): void {
    this.drag_start = undefined
  }

  /**
   * 数值限制工具函数
   * 确保数值在指定范围内
   */
  private clamp(value: number, min: number, max: number): number {
    return Math.max(min, Math.min(value, max))
  }

  /**
   * 处理缩放事件（滚轮缩放或双指缩放）
   *
   * 缩放算法：
   * 1. 计算缩放中心点（鼠标位置或双指中心）
   * 2. 将缩放中心转换为地图坐标
   * 3. 执行缩放操作
   * 4. 调整视图中心，使缩放中心在屏幕上的位置保持不变
   */
  private handleZoom(event: WheelEvent | TouchEvent): void {
    if (!this.state.inputMovement) return

    event.preventDefault()

    let scaleChange: number
    let clientX: number // 缩放中心X（相对于视口）
    let clientY: number // 缩放中心Y（相对于视口）

    const oldScale = this.state.viewScale

    if ('touches' in event) {
      // 双指缩放处理
      if (event.touches.length !== 2) return
      const touch1 = event.touches[0]!
      const touch2 = event.touches[1]!

      // 计算双指中心点
      clientX = (touch1.clientX + touch2.clientX) / 2
      clientY = (touch1.clientY + touch2.clientY) / 2

      // 计算当前双指距离
      const dx = touch1.clientX - touch2.clientX
      const dy = touch1.clientY - touch2.clientY
      const currentDistance = Math.hypot(dx, dy)

      // 初始化或更新缩放参考距离
      if (this.initialTouchDistance === 0) {
        this.initialTouchDistance = currentDistance
        return
      }

      // 计算缩放比例并应用限制
      scaleChange = currentDistance / this.initialTouchDistance
      const newScaleAttempt = oldScale * scaleChange
      this.state.viewScale = this.clamp(newScaleAttempt, this.MIN_SCALE, this.MAX_SCALE)

      // 重新计算实际的缩放比例（可能被限制了）
      if (oldScale === 0)
        scaleChange = 1 // 避免除零错误
      else scaleChange = this.state.viewScale / oldScale

      this.initialTouchDistance = currentDistance
    } else {
      // 滚轮缩放处理
      const wheelEvent = event
      const new_scale_attempt =
        oldScale * (wheelEvent.deltaY < 0 ? this.ZOOM_FACTOR : 1 / this.ZOOM_FACTOR)
      this.state.viewScale = this.clamp(new_scale_attempt, this.MIN_SCALE, this.MAX_SCALE)

      // 计算实际的缩放比例
      if (oldScale === 0)
        scaleChange = 1 // 避免除零错误
      else scaleChange = this.state.viewScale / oldScale

      clientX = wheelEvent.clientX
      clientY = wheelEvent.clientY
    }

    // 将视口坐标转换为容器内坐标，用于 screenToFixed 计算
    const containerRect = this.container.getBoundingClientRect()
    const screenPoint = {
      x: clientX - containerRect.left,
      y: clientY - containerRect.top,
    }
    const mapPoint = this.screenToFixed(screenPoint)

    /*
     * 缩放中心保持算法：
     * 目标：缩放后，鼠标指向的地图点在屏幕上的位置保持不变
     *
     * 原理：
     * 1. 缩放前鼠标指向地图点M，屏幕位置S
     * 2. 缩放改变了视图中心和缩放级别
     * 3. 需要调整视图中心，使点M仍然显示在屏幕位置S
     *
     * 公式推导：
     * 设缩放前中心为C1，缩放后中心为C2，缩放比例为k
     * 要求：点M在缩放前后的屏幕位置相同
     * 解得：C2 = C1 + (M - C1) * (1 - 1/k)
     */
    this.state.viewCenter = {
      x: this.state.viewCenter.x + (mapPoint.x - this.state.viewCenter.x) * (1 - 1 / scaleChange),
      y: this.state.viewCenter.y + (-mapPoint.y - this.state.viewCenter.y) * (1 - 1 / scaleChange),
    }

    this.sendUpdateEvent()
  }

  /**
   * 处理触摸开始事件
   *
   * 触摸交互状态机：
   * - 1个触摸点：进入拖拽模式
   * - 2个触摸点：进入缩放模式，停止拖拽
   * - 更多触摸点：忽略
   */
  private handleTouchStart(event: TouchEvent): void {
    if (!this.state.inputMovement) return

    if (this.options.shouldPreventDrag?.((event as Event).target)) {
      return
    }

    if (event.touches.length === 2) {
      /*
       * 双指缩放开始时的状态转换：
       * 1. 用户开始用一个手指拖拽地图 → drag_start 被设置
       * 2. 用户放下第二个手指准备缩放 → 检测到 touches.length === 2
       * 3. 立即停止单指拖拽 → drag_start = undefined
       * 4. 开始处理双指缩放操作
       */
      this.drag_start = undefined

      // 记录双指位置和初始距离
      this.touch1 = {
        clientX: event.touches[0]!.clientX,
        clientY: event.touches[0]!.clientY,
      }
      this.touch2 = {
        clientX: event.touches[1]!.clientX,
        clientY: event.touches[1]!.clientY,
      }
      const dx = this.touch1.clientX - this.touch2.clientX
      const dy = this.touch1.clientY - this.touch2.clientY
      this.initialTouchDistance = Math.hypot(dx, dy)
    } else if (event.touches.length === 1) {
      // 单指触摸：准备拖拽
      this.touch1 = {
        clientX: event.touches[0]!.clientX,
        clientY: event.touches[0]!.clientY,
      }
      this.touch2 = null
      this.initialTouchDistance = 0
      this.handleDragStart(event) // 开始拖拽
    }
  }

  /**
   * 处理触摸移动事件
   *
   * 根据当前触摸点数量决定处理方式：
   * - 2个触摸点且已记录：处理双指缩放
   * - 1个触摸点且正在拖拽：处理单指拖拽
   */
  private handleTouchMove(event: TouchEvent): void {
    if (!this.state.inputMovement) return

    if (event.touches.length === 2 && this.touch1 && this.touch2) {
      // 双指缩放移动
      if (this.drag_start) this.drag_start = undefined // 确保停止拖拽
      this.handleZoom(event)
    } else if (event.touches.length === 1 && this.drag_start) {
      // 单指拖拽移动
      this.handleDragMove(event)
    }
  }

  /**
   * 处理触摸结束事件
   *
   * 状态清理：
   * - 触摸点少于2个：清除双指缩放状态
   * - 没有触摸点：清除拖拽状态
   */
  private handleTouchEnd(event: TouchEvent): void {
    if (event.touches.length < 2) {
      // 双指缩放结束
      this.touch1 = null
      this.touch2 = null
      this.initialTouchDistance = 0
    }
    if (event.touches.length === 0) {
      // 所有触摸结束
      this.handleDragEnd()
    }
  }
}
