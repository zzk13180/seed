import { MapGridManager } from './MapGridCanvas/map-grid-manager'
import type { MapState } from './map.store'
import type { Subscription } from 'rxjs'
import { ViewManager } from '@/managers/view.manager'

export class MapController {
  private state: MapState
  private viewManager: ViewManager | null = null

  private gridManager: MapGridManager | null = null

  private subscriptions: Subscription[] = []

  constructor(state: MapState) {
    this.state = state
  }

  private initializeViewManager(container: HTMLElement) {
    if (this.viewManager) {
      return
    }
    this.viewManager = new ViewManager(this.state.viewState, container)
    this.subscriptions.push(
      this.viewManager.viewChange$.subscribe(() => {
        console.log('MapController viewChange')
      }),
    )
    console.log('MapController initializeViewManager', performance.now())
  }

  initialize(container: HTMLElement) {
    this.initializeViewManager(container)
  }

  destroy() {
    this.viewManager.destroy()
    this.subscriptions.forEach(sub => sub.unsubscribe())
    this.subscriptions = []
  }

  initializeGrid(canvas: HTMLCanvasElement): void {
    console.log('initializeGrid', performance.now())
    if (this.gridManager) {
      this.destroyGrid()
    }

    this.gridManager = new MapGridManager(this.state, canvas, this.viewManager)
    this.gridManager.initialize()
    console.log('MapGridManager initialized', performance.now())
  }

  destroyGrid(): void {
    console.log('destroyGrid', performance.now())
    if (this.gridManager) {
      this.gridManager.destroy()
      this.gridManager = null
    }
  }
}
