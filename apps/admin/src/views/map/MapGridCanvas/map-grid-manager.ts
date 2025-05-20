import type { MapState } from '../map.store'
import { ViewManager, type ViewState } from '@/managers/view-manager'

export interface MapGridEvents {
  onResize?: () => void
}

export class MapGridManager {
  private state: MapState
  private gridCanvas: HTMLCanvasElement | null = null
  private gridCtx: CanvasRenderingContext2D | null = null
  private container: HTMLElement | null = null
  private events: MapGridEvents
  private resizeObserver: ResizeObserver | null = null

  // 使用ViewManager替换原有的事件处理
  private viewManager: ViewManager | null = null

  constructor(
    state: MapState,
    canvas: HTMLCanvasElement,
    container: HTMLElement,
    events: MapGridEvents = {},
  ) {
    this.state = state
    this.gridCanvas = canvas
    this.container = container
    this.events = events
  }

  /**
   * 初始化网格
   */
  public initialize(): void {
    if (!this.gridCanvas || !this.container) return

    this.gridCtx = this.gridCanvas.getContext('2d', { colorSpace: 'srgb' })

    // 创建视图状态
    const viewState: ViewState = {
      center: this.state.viewCenter,
      scale: this.state.viewScale,
      inputMovement: this.state.inputMovement,
    }

    // 初始化ViewManager
    this.viewManager = new ViewManager(this.container, viewState, {
      onViewChange: () => {
        // 同步视图状态到MapState
        if (this.viewManager) {
          this.state.viewCenter = this.viewManager.getState().center
          this.state.viewScale = this.viewManager.getState().scale
          this.state.inputMovement = this.viewManager.getState().inputMovement

          // 更新网格绘制
          this.draw()
        }
      },
      shouldPreventDrag: target => {
        // 可以根据需要决定是否阻止某些元素的拖动
        return false
      },
    })

    this.viewManager.initialize()

    // 设置resize观察器
    this.setupResizeObserver()

    // 监听窗口resize事件
    window.addEventListener('resize', this.resize)
    window.addEventListener('orientationchange', this.resize)

    // 初始绘制
    this.resize()
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
   * 调整网格画布大小
   */
  public resize = (): void => {
    if (!this.gridCanvas || !this.container) return

    this.gridCanvas.width = this.container.clientWidth
    this.gridCanvas.height = this.container.clientHeight

    this.draw()

    // 触发自定义resize事件回调
    if (this.events.onResize) {
      this.events.onResize()
    }
  }

  /**
   * 销毁网格管理器
   */
  public destroy(): void {
    if (this.viewManager) {
      this.viewManager.destroy()
      this.viewManager = null
    }

    window.removeEventListener('resize', this.resize)
    window.removeEventListener('orientationchange', this.resize)

    if (this.resizeObserver) {
      this.resizeObserver.disconnect()
      this.resizeObserver = null
    }

    this.gridCanvas = null
    this.gridCtx = null
    this.container = null
  }

  /**
   * 将地图坐标转换为屏幕坐标
   */
  public fixedToScreen(coords: { x: number; y: number }): { x: number; y: number; z: number } {
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
  public screenToFixed(coords: { x: number; y: number }): { x: number; y: number; z: number } {
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
  public getPixelsInMapUnits(length: number): number {
    if (this.viewManager) {
      return this.viewManager.getPixelsInMapUnits(length)
    }
    return 0
  }

  /**
   * 获取地图单位长度在屏幕上的像素长度
   */
  public getMapUnitsInPixels(length: number): number {
    if (this.viewManager) {
      return this.viewManager.getMapUnitsInPixels(length)
    }
    return 0
  }

  /**
   * 设置是否允许输入移动
   */
  public setInputMovementEnabled(value: boolean): void {
    if (this.viewManager) {
      this.viewManager.setInputMovementEnabled(value)
      this.state.inputMovement = value
    }
  }

  /**
   * 重置视图到初始状态
   */
  public resetView(): void {
    if (this.viewManager) {
      this.viewManager.resetView({ x: 0, y: 0 }, 50.0)

      // 同步状态
      this.state.viewCenter = { x: 0, y: 0 }
      this.state.viewScale = 50.0
    }
  }

  /**
   * 设置视图中心和缩放级别
   */
  public setView(center?: { x: number; y: number }, scale?: number): void {
    if (!this.viewManager) return

    const currentState = this.viewManager.getState()

    if (center) {
      currentState.center = { ...center }
      this.state.viewCenter = { ...center }
    }

    if (scale !== undefined) {
      currentState.scale = scale
      this.state.viewScale = scale
    }

    this.viewManager.setState(currentState)
  }

  /**
   * 获取当前视图状态
   */
  public getView(): { center: { x: number; y: number }; scale: number } {
    if (this.viewManager) {
      const state = this.viewManager.getState()
      return {
        center: { ...state.center },
        scale: state.scale,
      }
    }

    return {
      center: { ...this.state.viewCenter },
      scale: this.state.viewScale,
    }
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
  public draw(): void {
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

      while ((linesX > 300 || linesY > 300) && tempSubdivisions > 0) {
        tempSubdivisions--
        linesX = (maxX - minX) / (gridSize / (tempSubdivisions + 1))
        linesY = (maxY - minY) / (gridSize / (tempSubdivisions + 1))
      }

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
