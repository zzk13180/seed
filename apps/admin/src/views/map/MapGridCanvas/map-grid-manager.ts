import type { MapState } from '../map.store'
import type { ViewManager } from '@/managers/view.manager'
import type { Subscription } from 'rxjs'

export interface MapGridEvents {
  onResize?: () => void
}

export class MapGridManager {
  private gridCanvas: HTMLCanvasElement | null = null
  private gridCtx: CanvasRenderingContext2D | null = null
  private resizeObserver: ResizeObserver | null = null
  private subscriptions: Subscription[] = []
  private readonly mapState: MapState
  private readonly events: MapGridEvents
  private readonly viewManager: ViewManager | null = null

  private readonly _boundHandlers = {
    resize: this.resize.bind(this),
  }

  constructor(
    mapState: MapState,
    canvas: HTMLCanvasElement,
    viewManager: ViewManager,
    events: MapGridEvents = {},
  ) {
    this.mapState = mapState
    this.gridCanvas = canvas
    this.viewManager = viewManager
    this.events = events
  }

  /**
   * 初始化网格
   */
  initialize(): void {
    if (!this.gridCanvas) return

    this.gridCtx = this.gridCanvas.getContext('2d', { colorSpace: 'srgb' })

    this.subscriptions.push(
      this.viewManager.viewChange$.subscribe(() => {
        // 同步视图状态到MapState
        if (this.viewManager) {
          this.mapState.viewState.viewCenter = this.viewManager.getState().viewCenter
          this.mapState.viewState.viewScale = this.viewManager.getState().viewScale
          this.mapState.viewState.inputMovement = this.viewManager.getState().inputMovement
          this.resize()
        }
      }),
    )

    // 设置resize观察器
    this.setupResizeObserver()

    // 监听窗口resize事件
    window.addEventListener('resize', this._boundHandlers.resize)
    globalThis.addEventListener('orientationchange', this._boundHandlers.resize)

    // 初始绘制
    this.resize()
  }

  /**
   * 销毁网格管理器
   */
  destroy(): void {
    window.removeEventListener('resize', this._boundHandlers.resize)
    globalThis.removeEventListener('orientationchange', this._boundHandlers.resize)

    if (this.resizeObserver) {
      this.resizeObserver.disconnect()
      this.resizeObserver = null
    }

    this.gridCanvas = null
    this.gridCtx = null

    for (const sub of this.subscriptions) sub.unsubscribe()
    this.subscriptions = []
  }

  /**
   * 调整网格画布大小
   */
  private resize(): void {
    if (!this.gridCanvas) return

    this.gridCanvas.width = this.viewManager.getContainerWidth
    this.gridCanvas.height = this.viewManager.getContainerHeight

    this.draw()

    // 触发自定义resize事件回调
    if (this.events.onResize) {
      this.events.onResize()
    }
  }

  /**
   * 设置ResizeObserver观察画布尺寸变化
   */
  private setupResizeObserver(): void {
    if (!this.gridCanvas) return

    this.resizeObserver = new ResizeObserver(() => {
      this.resize()
    })

    this.resizeObserver.observe(this.gridCanvas)
  }

  /**
   * 将地图坐标转换为屏幕坐标
   */
  private fixedToScreen(coords: { x: number; y: number }): { x: number; y: number; z: number } {
    if (this.viewManager) {
      const result = this.viewManager.fixedToScreen(coords)
      return { ...result, z: 0 }
    }

    // 默认返回
    return { x: 0, y: 0, z: 0 }
  }

  /**
   * 将屏幕坐标转换为地图坐标
   */
  private screenToFixed(coords: { x: number; y: number }): { x: number; y: number; z: number } {
    if (this.viewManager) {
      const result = this.viewManager.screenToFixed(coords)
      return { ...result, z: 0 }
    }

    // 默认返回
    return { x: 0, y: 0, z: 0 }
  }

  /**
   * 获取屏幕像素长度在地图单位中的长度
   */
  private getPixelsInMapUnits(length: number): number {
    if (this.viewManager) {
      return this.viewManager.getPixelsInMapUnits(length)
    }
    return 0
  }

  /**
   * 获取地图单位长度在屏幕上的像素长度
   */
  private getMapUnitsInPixels(length: number): number {
    if (this.viewManager) {
      return this.viewManager.getMapUnitsInPixels(length)
    }
    return 0
  }

  /**
   * 设置是否允许输入移动
   */
  private setInputMovementEnabled(value: boolean): void {
    if (this.viewManager) {
      this.viewManager.setInputMovementEnabled(value)
      this.mapState.viewState.inputMovement = value
    }
  }

  /**
   * 重置视图到初始状态
   */
  private resetView(): void {
    if (this.viewManager) {
      this.viewManager.resetView({ x: 0, y: 0 }, 50)

      // 同步状态
      this.mapState.viewState.viewCenter = { x: 0, y: 0 }
      this.mapState.viewState.viewScale = 50
    }
  }

  /**
   * 设置视图中心和缩放级别
   */
  private setView(center?: { x: number; y: number }, scale?: number): void {
    if (!this.viewManager) return

    const currentState = this.viewManager.getState()

    if (center) {
      currentState.viewCenter = { ...center }
      this.mapState.viewState.viewCenter = { ...center }
    }

    if (scale !== undefined) {
      currentState.viewScale = scale
      this.mapState.viewState.viewScale = scale
    }

    this.viewManager.setState(currentState)
  }

  /**
   * 获取当前视图状态
   */
  private getView(): { center: { x: number; y: number }; scale: number } {
    if (this.viewManager) {
      const mapState = this.viewManager.getState()
      return {
        center: { ...mapState.viewCenter },
        scale: mapState.viewScale,
      }
    }

    return {
      center: { ...this.mapState.viewState.viewCenter },
      scale: this.mapState.viewState.viewScale,
    }
  }

  /**
   * 计算缩放级别
   */
  private calculateScale(value: number): number {
    const magnitude = Math.floor(Math.log10(value))
    value /= Math.pow(10, magnitude)

    if (value < 1.5) {
      value = 1
    } else if (value < 3.5) {
      value = 2
    } else if (value < 7.5) {
      value = 5
    } else {
      value = 10
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
    this.gridCtx.globalAlpha = this.mapState.gridConfig.subGridOpacity
    this.gridCtx.strokeStyle = this.mapState.gridConfig.colour_sub
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
    this.gridCtx.globalAlpha = 1
    this.gridCtx.strokeStyle = this.mapState.gridConfig.colour
    this.gridCtx.lineWidth = this.mapState.gridConfig.thickness

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
    const scaleTo = this.screenToFixed({
      x: width - xoffset,
      y: height - yoffset,
    })
    const xScaleStart = this.fixedToScreen({
      x: scaleTo.x - gridSize,
      y: 0,
    }).x

    this.drawScreenLine(
      xScaleStart,
      parseInt((height - yoffset).toString(), 10),
      parseInt((width - xoffset).toString(), 10),
      parseInt((height - yoffset).toString(), 10),
      this.mapState.gridConfig.colour,
      2,
    )

    this.drawScreenLine(
      xScaleStart,
      parseInt((height - yoffset - 5).toString(), 10),
      xScaleStart,
      parseInt((height - yoffset + 5).toString(), 10),
      this.mapState.gridConfig.colour,
      2,
    )

    this.drawScreenLine(
      parseInt((width - xoffset).toString(), 10),
      parseInt((height - yoffset - 5).toString(), 10),
      parseInt((width - xoffset).toString(), 10),
      parseInt((height - yoffset + 5).toString(), 10),
      this.mapState.gridConfig.colour,
      2,
    )

    const lineLength = parseInt((width - xoffset).toString(), 10) - xScaleStart

    let scaleText = `${gridSize} m`
    if (gridSize >= 1000) scaleText = `${gridSize / 1000} km`
    else if (gridSize < 1) scaleText = `${gridSize * 100} cm`

    this.gridCtx.textAlign = 'center'
    this.gridCtx.fillStyle = this.mapState.gridConfig.colour
    this.gridCtx.fillText(
      scaleText,
      parseInt((xScaleStart + lineLength / 2).toString(), 10),
      parseInt((height - 23).toString(), 10),
    )
  }

  /**
   * 绘制网格
   */
  private draw(): void {
    if (!this.gridCanvas || !this.gridCtx) return

    const width = this.gridCanvas.width
    const height = this.gridCanvas.height

    this.gridCtx.strokeStyle = this.mapState.gridConfig.colour
    this.gridCtx.lineWidth = this.mapState.gridConfig.thickness

    const topLeft = this.screenToFixed({ x: 0, y: 0 })
    const bottomRight = this.screenToFixed({ x: width, y: height })

    const widthMeters = Math.abs(bottomRight.x - topLeft.x)
    const heightMeters = Math.abs(bottomRight.y - topLeft.y)

    let gridSize = this.mapState.gridConfig.size

    switch (this.mapState.gridConfig.autoscale) {
      case 'Very Fine': {
        gridSize = this.calculateScale(Math.min(widthMeters, heightMeters) / 21)
        break
      }
      case 'Fine': {
        gridSize = this.calculateScale(Math.min(widthMeters, heightMeters) / 14)
        break
      }
      case 'Coarse': {
        gridSize = this.calculateScale(Math.min(widthMeters, heightMeters) / 7)
        break
      }
      case 'Rough': {
        {
          gridSize = this.calculateScale(Math.min(widthMeters, heightMeters) / 3)
          // No default
        }
        break
      }
    }

    const minX = topLeft.x - (topLeft.x % gridSize) - gridSize
    const maxX = bottomRight.x + (gridSize - (bottomRight.x % gridSize))

    const minY = bottomRight.y - (bottomRight.y % gridSize) - gridSize
    const maxY = topLeft.y + (gridSize - (topLeft.y % gridSize))

    this.gridCtx.clearRect(0, 0, width, height)

    let tempSubdivisions = this.mapState.gridConfig.subdivisions

    if (this.mapState.gridConfig.autoscale === 'Off') {
      let linesX = (maxX - minX) / (gridSize / (tempSubdivisions + 1))
      let linesY = (maxY - minY) / (gridSize / (tempSubdivisions + 1))

      while (
        (linesX > this.mapState.gridConfig.maxGridLines ||
          linesY > this.mapState.gridConfig.maxGridLines) &&
        tempSubdivisions > 0
      ) {
        tempSubdivisions--
        linesX = (maxX - minX) / (gridSize / (tempSubdivisions + 1))
        linesY = (maxY - minY) / (gridSize / (tempSubdivisions + 1))
      }

      if (
        linesX > this.mapState.gridConfig.maxGridLines ||
        linesY > this.mapState.gridConfig.maxGridLines
      ) {
        this.gridCtx.clearRect(0, 0, width, height)
        console.warn('Too many lines to render, increase step size')
        return
      }
    }

    this.drawGridLines(minX, minY, maxX, maxY, gridSize, tempSubdivisions)

    if (this.mapState.gridConfig.autoscale !== 'Off') {
      this.drawGridScale(gridSize, width, height)
    }
  }
}
