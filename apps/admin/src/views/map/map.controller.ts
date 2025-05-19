import type { MapState } from './map.store'

export class MapController {
  private readonly ZOOM_FACTOR = 1.05
  private readonly MAX_SCALE = 5000
  private readonly MIN_SCALE = 0.001
  private state: MapState
  private container: HTMLElement | null = null
  private drag_start:
    | { x: number; y: number; ref_center_x: number; ref_center_y: number }
    | undefined = undefined

  private touch1: { clientX: number; clientY: number } | null = null
  private touch2: { clientX: number; clientY: number } | null = null
  private initialTouchDistance = 0
  private event_timestamp = performance.now()
  private event_timeout: number | undefined = undefined
  private _boundHandlers: Record<string, (event: any) => void> = {}

  // 网格相关属性
  private gridCanvas: HTMLCanvasElement | null = null
  private gridCtx: CanvasRenderingContext2D | null = null

  constructor(state: MapState) {
    this.state = state

    // 绑定事件处理方法
    this._boundHandlers = {
      dragStart: this.handleDragStart.bind(this),
      dragMove: this.handleDragMove.bind(this),
      dragEnd: this.handleDragEnd.bind(this),
      zoom: this.handleZoom.bind(this),
      touchStart: this._handleTouchStart.bind(this),
      touchMove: this._handleTouchMove.bind(this),
      touchEnd: this._handleTouchEnd.bind(this),
    }
  }

  public initialize(container: HTMLElement): void {
    console.log('MapController initialized with container')
    this.container = container
    this.addListeners()
  }

  public destroy(): void {
    this.removeListeners()
    if (this.event_timeout) {
      clearTimeout(this.event_timeout)
    }
    this.gridCanvas = null
    this.gridCtx = null
  }

  /**
   * 发送视图更新事件（带防抖）
   */
  sendUpdateEvent(): void {
    if (this.event_timeout !== undefined) {
      clearTimeout(this.event_timeout)
    }
    const delta = performance.now() - this.event_timestamp

    if (delta > 12) {
      this._triggerViewChange()
      this.event_timestamp = performance.now()
    } else {
      this.event_timeout = setTimeout(() => {
        this._triggerViewChange()
      }, 12 - delta) as unknown as number
    }
  }

  /**
   * 触发视图变化事件并调用回调
   * @private
   */
  private _triggerViewChange(): void {
    window.dispatchEvent(new Event('view_changed'))

    if (typeof this.state.viewChangeCallback === 'function') {
      this.state.viewChangeCallback({
        center: this.state.viewCenter,
        scale: this.state.viewScale,
      })
    }

    // 视图变化时重绘网格
    this.drawGrid()
  }

  /**
   * 设置是否允许输入移动
   * @param {boolean} value - 是否启用
   */
  setInputMovementEnabled(value: boolean): void {
    this.state.inputMovement = value
  }

  /**
   * 重置视图到初始状态
   */
  resetView(): void {
    this.state.viewCenter = { x: 0, y: 0 }
    this.state.viewScale = 50.0
    this.drag_start = undefined

    this.sendUpdateEvent()
  }

  /**
   * 将地图坐标转换为屏幕坐标
   * @param {Object} coords - 地图坐标
   * @return {Object} - 屏幕坐标
   */
  fixedToScreen(coords: { x: number; y: number }): { x: number; y: number; z: number } {
    return {
      x: (coords.x - this.state.viewCenter.x) * this.state.viewScale + window.innerWidth / 2,
      y: (-coords.y - this.state.viewCenter.y) * this.state.viewScale + window.innerHeight / 2,
      z: 0,
    }
  }

  /**
   * 将屏幕坐标转换为地图坐标
   * @param {Object} coords - 屏幕坐标
   * @return {Object} - 地图坐标
   */
  screenToFixed(coords: { x: number; y: number }): { x: number; y: number; z: number } {
    return {
      x: (coords.x - window.innerWidth / 2) / this.state.viewScale + this.state.viewCenter.x,
      y: -(coords.y - window.innerHeight / 2) / this.state.viewScale - this.state.viewCenter.y,
      z: 0,
    }
  }

  /**
   * 获取屏幕像素长度在地图单位中的长度
   * @param {number} length - 屏幕像素长度
   * @return {number} - 地图单位长度
   */
  getPixelsInMapUnits(length: number): number {
    const p1 = this.screenToFixed({ x: 0, y: 0 })
    const p2 = this.screenToFixed({ x: length, y: 0 })
    return Math.abs(p1.x - p2.x)
  }

  /**
   * 获取地图单位长度在屏幕上的像素长度
   * @param {number} length - 地图单位长度
   * @return {number} - 屏幕像素长度
   */
  getMapUnitsInPixels(length: number): number {
    const p1 = this.fixedToScreen({ x: 0, y: 0 })
    const p2 = this.fixedToScreen({ x: length, y: 0 })
    return Math.abs(p1.x - p2.x)
  }

  /**
   * 处理拖拽开始事件
   * @param {Event} event - 事件对象
   */
  handleDragStart(event: MouseEvent | TouchEvent): void {
    if (!this.state.inputMovement) return

    const clientX = 'touches' in event ? event.touches[0].clientX : event.clientX
    const clientY = 'touches' in event ? event.touches[0].clientY : event.clientY

    this.drag_start = {
      x: clientX,
      y: clientY,
      ref_center_x: this.state.viewCenter.x,
      ref_center_y: this.state.viewCenter.y,
    }
  }

  /**
   * 处理拖拽移动事件
   * @param {Event} event - 事件对象
   */
  handleDragMove(event: MouseEvent | TouchEvent): void {
    if (this.drag_start === undefined) return

    if (!this.state.inputMovement) {
      this.drag_start = undefined
      return
    }

    if (this.hasClassInParentChain(event.target as HTMLElement, 'inputelement')) {
      return
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

  /**
   * 处理拖拽结束事件
   */
  handleDragEnd(): void {
    this.drag_start = undefined
  }

  /**
   * 处理缩放事件（鼠标滚轮或触摸缩放）
   * @param {Event} event - 事件对象
   */
  handleZoom(event: WheelEvent | TouchEvent): void {
    if (!this.state.inputMovement) return

    let scaleChange: number
    let centerX: number
    let centerY: number

    if ('touches' in event) {
      if (event.touches.length !== 2) return
      const [touch1, touch2] = Array.from(event.touches)
      centerX = (touch1.clientX + touch2.clientX) / 2
      centerY = (touch1.clientY + touch2.clientY) / 2
      const dx = touch1.clientX - touch2.clientX
      const dy = touch1.clientY - touch2.clientY
      const distance = Math.sqrt(dx * dx + dy * dy)
      scaleChange = distance / this.initialTouchDistance
      this.initialTouchDistance = distance
      const new_scale = this.state.viewScale * scaleChange
      this.state.viewScale = this.clamp(new_scale, this.MIN_SCALE, this.MAX_SCALE)
    } else {
      const wheelEvent = event
      const new_scale =
        this.state.viewScale * (-wheelEvent.deltaY < 0 ? 1 / this.ZOOM_FACTOR : this.ZOOM_FACTOR)
      const clamped_scale = this.clamp(new_scale, this.MIN_SCALE, this.MAX_SCALE)
      scaleChange = clamped_scale / this.state.viewScale
      centerX = wheelEvent.clientX
      centerY = wheelEvent.clientY
      this.state.viewScale = clamped_scale
    }

    const screenPoint = { x: centerX, y: centerY }
    const mapPoint = this.screenToFixed(screenPoint)

    const newCenter = {
      x: this.state.viewCenter.x + (mapPoint.x - this.state.viewCenter.x) * (1 - 1 / scaleChange),
      y: this.state.viewCenter.y + (-mapPoint.y - this.state.viewCenter.y) * (1 - 1 / scaleChange),
    }

    this.state.viewCenter = newCenter
    this.sendUpdateEvent()
  }

  /**
   * 处理触摸开始事件
   * @param {TouchEvent} event - 触摸事件
   * @private
   */
  private _handleTouchStart(event: TouchEvent): void {
    if (event.touches.length === 2) {
      if (!this.touch1 || !this.touch2) {
        this.touch1 = {
          clientX: event.touches[0].clientX,
          clientY: event.touches[0].clientY,
        }
        this.touch2 = {
          clientX: event.touches[1].clientX,
          clientY: event.touches[1].clientY,
        }
        const dx = this.touch1.clientX - this.touch2.clientX
        const dy = this.touch1.clientY - this.touch2.clientY
        this.initialTouchDistance = Math.sqrt(dx * dx + dy * dy)
      }
    } else {
      this.touch1 = null
      this.touch2 = null
      this.handleDragStart(event)
    }
  }

  /**
   * 处理触摸移动事件
   * @param {TouchEvent} event - 触摸事件
   * @private
   */
  private _handleTouchMove(event: TouchEvent): void {
    if (event.touches.length === 2) {
      this.handleZoom(event)
    } else {
      this.handleDragMove(event)
    }
  }

  /**
   * 处理触摸结束事件
   * @param {TouchEvent} event - 触摸事件
   * @private
   */
  private _handleTouchEnd(event: TouchEvent): void {
    this.touch1 = null
    this.touch2 = null
    this.handleDragEnd()
  }

  /**
   * 添加事件监听器
   */
  addListeners(): void {
    if (!this.container) return

    this.container.addEventListener('mousedown', this._boundHandlers.dragStart)
    this.container.addEventListener('mousemove', this._boundHandlers.dragMove)
    this.container.addEventListener('mouseup', this._boundHandlers.dragEnd)
    this.container.addEventListener('mouseleave', this._boundHandlers.dragEnd)
    this.container.addEventListener('wheel', this._boundHandlers.zoom)

    this.container.addEventListener('touchstart', this._boundHandlers.touchStart)
    this.container.addEventListener('touchmove', this._boundHandlers.touchMove)
    this.container.addEventListener('touchend', this._boundHandlers.touchEnd)
  }

  /**
   * 移除事件监听器
   */
  removeListeners(): void {
    if (!this.container) return

    this.container.removeEventListener('mousedown', this._boundHandlers.dragStart)
    this.container.removeEventListener('mousemove', this._boundHandlers.dragMove)
    this.container.removeEventListener('mouseup', this._boundHandlers.dragEnd)
    this.container.removeEventListener('mouseleave', this._boundHandlers.dragEnd)
    this.container.removeEventListener('wheel', this._boundHandlers.zoom)

    this.container.removeEventListener('touchstart', this._boundHandlers.touchStart)
    this.container.removeEventListener('touchmove', this._boundHandlers.touchMove)
    this.container.removeEventListener('touchend', this._boundHandlers.touchEnd)
  }

  /**
   * 设置视图中心和缩放级别
   * @param {Object} center - 中心点坐标 {x, y}
   * @param {number} scale - 缩放级别
   */
  setView(center?: { x: number; y: number }, scale?: number): void {
    if (center) {
      this.state.viewCenter = { ...center }
    }

    if (scale !== undefined) {
      this.state.viewScale = this.clamp(scale, this.MIN_SCALE, this.MAX_SCALE)
    }

    this.sendUpdateEvent()
  }

  /**
   * 获取当前视图状态
   * @return {Object} 包含中心点和缩放级别的对象
   */
  getView(): { center: { x: number; y: number }; scale: number } {
    return {
      center: { ...this.state.viewCenter },
      scale: this.state.viewScale,
    }
  }

  /**
   * 限制数值在指定范围内
   * @param {number} val - 要限制的值
   * @param {number} from - 最小值
   * @param {number} to - 最大值
   * @return {number} - 限制后的值
   */
  private clamp(val: number, from: number, to: number): number {
    if (val > to) return to
    if (val < from) return from
    return val
  }

  /**
   * 检查元素是否具有指定的 CSS 类
   * @param {HTMLElement} element - 要检查的元素
   * @param {string} className - 要检查的类名
   * @return {boolean} - 元素是否具有该类
   */
  private hasClass(element: HTMLElement, className: string): boolean {
    if (element.classList) {
      return element.classList.contains(className)
    } else {
      return new RegExp(`(^| )${className}( |$)`, 'gi').test(element.className)
    }
  }

  /**
   * 检查元素或其祖先元素是否具有指定的 CSS 类
   * @param {HTMLElement} element - 起始元素
   * @param {string} className - 要检查的类名
   * @return {boolean} - 元素或其祖先是否具有该类
   */
  private hasClassInParentChain(element: HTMLElement | null, className: string): boolean {
    if (!element) {
      return false
    }

    if (this.hasClass(element, className)) {
      return true
    }

    return this.hasClassInParentChain(element.parentElement, className)
  }

  get computedExample() {
    return this.state.rosBridgeServerUrl
  }

  // 网格相关方法
  /**
   * 初始化网格
   */
  public initializeGrid(canvas: HTMLCanvasElement): void {
    this.gridCanvas = canvas
    this.gridCtx = canvas.getContext('2d', { colorSpace: 'srgb' })
    this.resizeGridCanvas()
  }

  /**
   * 调整网格画布大小
   */
  public resizeGridCanvas(): void {
    if (!this.gridCanvas) return

    this.gridCanvas.height = window.innerHeight
    this.gridCanvas.width = window.innerWidth
    this.drawGrid()
  }

  /**
   * 计算缩放级别
   */
  private calculateScale(value: number): number {
    const magnitude = Math.floor(Math.log10(value))
    value /= Math.pow(10, magnitude)

    if (value < 1.5) {
      value = 1.0
    } else if (value < 3.5) {
      value = 2.0
    } else if (value < 7.5) {
      value = 5.0
    } else {
      value = 10.0
    }

    return value * Math.pow(10, magnitude)
  }

  /**
   * 绘制屏幕线条
   */
  // eslint-disable-next-line max-params
  private drawScreenLine(
    startX: number,
    startY: number,
    endX: number,
    endY: number,
    color: string,
    lineWidth: number,
  ): void {
    if (!this.gridCtx) return

    this.gridCtx.beginPath()
    this.gridCtx.strokeStyle = color
    this.gridCtx.lineWidth = lineWidth

    this.gridCtx.moveTo(parseInt(startX.toString(), 10), parseInt(startY.toString(), 10))
    this.gridCtx.lineTo(parseInt(endX.toString(), 10), parseInt(endY.toString(), 10))

    this.gridCtx.stroke()
  }

  /**
   * 绘制网格线
   */
  // eslint-disable-next-line max-params
  private drawGridLines(
    minX: number,
    minY: number,
    maxX: number,
    maxY: number,
    gridSize: number,
    subdivisions: number,
  ): void {
    if (!this.gridCtx || !this.gridCanvas) return

    const subdivisionSize = gridSize / subdivisions

    // 计算所有网格交叉点的屏幕位置
    const xPositions = new Map<number, number>()
    const yPositions = new Map<number, number>()

    // 计算主网格位置
    for (let x = minX; x <= maxX; x += gridSize) {
      const screenPos = this.fixedToScreen({ x, y: 0 }).x
      xPositions.set(x, parseInt(screenPos.toString(), 10))
    }

    for (let y = minY; y <= maxY; y += gridSize) {
      const screenPos = this.fixedToScreen({ x: 0, y }).y
      yPositions.set(y, parseInt(screenPos.toString(), 10))
    }

    if (subdivisions > 1) {
      // 计算子网格位置
      for (let x = minX; x <= maxX; x += gridSize) {
        for (let subX = 1; subX < subdivisions; subX++) {
          const curSubX = x + subX * subdivisionSize
          const screenPos = this.fixedToScreen({ x: curSubX, y: 0 }).x
          xPositions.set(curSubX, parseInt(screenPos.toString(), 10))
        }
      }

      for (let y = minY; y <= maxY; y += gridSize) {
        for (let subY = 1; subY < subdivisions; subY++) {
          const curSubY = y + subY * subdivisionSize
          const screenPos = this.fixedToScreen({ x: 0, y: curSubY }).y
          yPositions.set(curSubY, parseInt(screenPos.toString(), 10))
        }
      }
    }

    // 绘制子网格线
    this.gridCtx.beginPath()
    this.gridCtx.globalAlpha = 0.65
    this.gridCtx.strokeStyle = this.state.gridConfig.colour_sub
    this.gridCtx.lineWidth = 1

    // 垂直子网格线
    for (let x = minX; x <= maxX; x += gridSize) {
      for (let subX = 1; subX < subdivisions; subX++) {
        const curSubX = x + subX * subdivisionSize
        const screenX = xPositions.get(curSubX)
        if (screenX !== undefined) {
          this.gridCtx.moveTo(screenX, 0)
          this.gridCtx.lineTo(screenX, this.gridCanvas.height)
        }
      }
    }

    // 水平子网格线
    for (let y = minY; y <= maxY; y += gridSize) {
      for (let subY = 1; subY < subdivisions; subY++) {
        const curSubY = y + subY * subdivisionSize
        const screenY = yPositions.get(curSubY)
        if (screenY !== undefined) {
          this.gridCtx.moveTo(0, screenY)
          this.gridCtx.lineTo(this.gridCanvas.width, screenY)
        }
      }
    }

    this.gridCtx.stroke()

    // 绘制主网格线
    this.gridCtx.beginPath()
    this.gridCtx.globalAlpha = 1.0
    this.gridCtx.strokeStyle = this.state.gridConfig.colour
    this.gridCtx.lineWidth = this.state.gridConfig.thickness

    // 垂直主线
    for (let x = minX; x <= maxX; x += gridSize) {
      const screenX = xPositions.get(x)
      if (screenX !== undefined) {
        this.gridCtx.moveTo(screenX, 0)
        this.gridCtx.lineTo(screenX, this.gridCanvas.height)
      }
    }

    // 水平主线
    for (let y = minY; y <= maxY; y += gridSize) {
      const screenY = yPositions.get(y)
      if (screenY !== undefined) {
        this.gridCtx.moveTo(0, screenY)
        this.gridCtx.lineTo(this.gridCanvas.width, screenY)
      }
    }

    this.gridCtx.stroke()
  }

  /**
   * 绘制网格刻度
   */
  private drawGridScale(gridSize: number, width: number, height: number): void {
    if (!this.gridCtx) return

    const xoffset = width < height ? 40 : 100 // 在垂直/移动设备上紧凑显示
    const yoffset = 40

    // 在右下角绘制比例信息
    const scaleTo = this.screenToFixed({ x: width - xoffset, y: height - yoffset })
    const xScaleStart = this.fixedToScreen({ x: scaleTo.x - gridSize, y: 0 }).x

    this.drawScreenLine(
      xScaleStart,
      parseInt((height - yoffset).toString(), 10),
      parseInt((width - xoffset).toString(), 10),
      parseInt((height - yoffset).toString(), 10),
      this.state.gridConfig.colour,
      2,
    )

    this.drawScreenLine(
      xScaleStart,
      parseInt((height - yoffset - 5).toString(), 10),
      xScaleStart,
      parseInt((height - yoffset + 5).toString(), 10),
      this.state.gridConfig.colour,
      2,
    )

    this.drawScreenLine(
      parseInt((width - xoffset).toString(), 10),
      parseInt((height - yoffset - 5).toString(), 10),
      parseInt((width - xoffset).toString(), 10),
      parseInt((height - yoffset + 5).toString(), 10),
      this.state.gridConfig.colour,
      2,
    )

    const lineLength = parseInt((width - xoffset).toString(), 10) - xScaleStart

    let scaleText = `${gridSize} m`
    if (gridSize >= 1000) scaleText = `${gridSize / 1000} km`
    else if (gridSize < 1) scaleText = `${gridSize * 100} cm`

    this.gridCtx.font = '16px Monospace'
    this.gridCtx.textAlign = 'center'
    this.gridCtx.fillStyle = this.state.gridConfig.colour
    this.gridCtx.fillText(
      scaleText,
      parseInt((xScaleStart + lineLength / 2).toString(), 10),
      parseInt((height - 23).toString(), 10),
    )
  }

  /**
   * 绘制网格
   */
  public drawGrid(): void {
    if (!this.gridCanvas || !this.gridCtx) return

    const width = this.gridCanvas.width
    const height = this.gridCanvas.height

    this.gridCtx.strokeStyle = this.state.gridConfig.colour
    this.gridCtx.lineWidth = this.state.gridConfig.thickness

    const topLeft = this.screenToFixed({ x: 0, y: 0 })
    const bottomRight = this.screenToFixed({ x: width, y: height })

    const widthMeters = Math.abs(bottomRight.x - topLeft.x)
    const heightMeters = Math.abs(bottomRight.y - topLeft.y)

    let gridSize = this.state.gridConfig.size

    if (this.state.gridConfig.autoscale === 'Very Fine')
      gridSize = this.calculateScale(Math.min(widthMeters, heightMeters) / 21)
    else if (this.state.gridConfig.autoscale === 'Fine')
      gridSize = this.calculateScale(Math.min(widthMeters, heightMeters) / 14)
    else if (this.state.gridConfig.autoscale === 'Coarse')
      gridSize = this.calculateScale(Math.min(widthMeters, heightMeters) / 7)
    else if (this.state.gridConfig.autoscale === 'Rough')
      gridSize = this.calculateScale(Math.min(widthMeters, heightMeters) / 3)

    const minX = topLeft.x - (topLeft.x % gridSize) - gridSize
    const maxX = bottomRight.x + (gridSize - (bottomRight.x % gridSize))

    const minY = bottomRight.y - (bottomRight.y % gridSize) - gridSize
    const maxY = topLeft.y + (gridSize - (topLeft.y % gridSize))

    this.gridCtx.clearRect(0, 0, width, height)

    let tempSubdivisions = this.state.gridConfig.subdivisions

    if (this.state.gridConfig.autoscale === 'Off') {
      let linesX = (maxX - minX) / (gridSize / (tempSubdivisions + 1))
      let linesY = (maxY - minY) / (gridSize / (tempSubdivisions + 1))

      // 当线条太多且还可以减少细分时
      while ((linesX > 300 || linesY > 300) && tempSubdivisions > 0) {
        tempSubdivisions--
        linesX = (maxX - minX) / (gridSize / (tempSubdivisions + 1))
        linesY = (maxY - minY) / (gridSize / (tempSubdivisions + 1))
      }

      // 如果即使没有细分，线条仍然太多
      if (linesX > 300 || linesY > 300) {
        this.gridCtx.clearRect(0, 0, width, height)
        console.warn('Too many lines to render, increase step size')
        return
      }
    }

    this.drawGridLines(minX, minY, maxX, maxY, gridSize, tempSubdivisions)

    if (this.state.gridConfig.autoscale !== 'Off') {
      this.drawGridScale(gridSize, width, height)
    }
  }
}
