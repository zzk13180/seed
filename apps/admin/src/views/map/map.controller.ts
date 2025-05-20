import { MapGridManager } from './MapGridCanvas/map-grid-manager'
import type { MapState } from './map.store'

export class MapController {
  private state: MapState
  private gridManager: MapGridManager | null = null

  constructor(state: MapState) {
    this.state = state
  }

  /**
   * 初始化网格管理器
   */
  public initializeGrid(canvas: HTMLCanvasElement, container: HTMLElement): void {
    if (this.gridManager) {
      this.destroyGrid()
    }

    this.gridManager = new MapGridManager(this.state, canvas, container)
    this.gridManager.initialize()
  }

  /**
   * 销毁网格管理器
   */
  public destroyGrid(): void {
    if (this.gridManager) {
      this.gridManager.destroy()
      this.gridManager = null
    }
  }

  get computedExample() {
    return this.state.rosBridgeServerUrl
  }
}
