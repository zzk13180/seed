import { ViewManager } from './managers/view.manager'
import { MapGridManager } from './MapGridCanvas/map-grid-manager'
import type { MapState } from './map.store'
import type { Subscription } from 'rxjs'

export class MapController {
  private viewManager: ViewManager | null = null
  private gridManager: MapGridManager | null = null
  private subscriptions: Subscription[] = []
  private readonly state: MapState

  constructor(state: MapState) {
    this.state = state
  }

  initialize(container: HTMLElement) {
    this.initializeViewManager(container)
  }

  destroy() {
    this.viewManager.destroy()
    for (const sub of this.subscriptions) sub.unsubscribe()
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

  private initializeViewManager(container: HTMLElement) {
    if (this.viewManager) {
      return
    }
    this.viewManager = new ViewManager(container)
    this.subscriptions.push(
      this.viewManager.viewStateChange$.subscribe(() => {
        console.log('MapController viewChange')
      }),
    )
    console.log('MapController initializeViewManager', performance.now())
  }
}
