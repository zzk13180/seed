import { onGlobalEvent } from '@seed/common/utils/event.util'
import type { HexBoard } from '../HexBoard'

export class CameraControllingMouseListener {
  mode = 'pan'
  priorMapX = 0
  priorMapY = 0
  priorCanvasX = 0
  priorCanvasY = 0
  rotateMultiplier = 1 / 500
  zoomMultiplier = 5

  constructor(private board: HexBoard) {
    onGlobalEvent('pointerDown', () => {
      if (this.board.positionData.clickedItem) {
        this.priorMapX = this.board.positionData.mapX
        this.priorMapY = this.board.positionData.mapY
        this.priorCanvasX = this.board.positionData.canvasX
        this.priorCanvasY = this.board.positionData.canvasY
      }
    })
    onGlobalEvent('pointerMove', () => {
      if (!this.board.positionData.clickedItem) {
        let dx = 0
        let dy = 0
        if (this.mode === 'pan') {
          dx = this.priorMapX - this.board.positionData.mapX
          dy = this.priorMapY - this.board.positionData.mapY
          this.board.pan(dx, dy)
        } else if (this.mode === 'tilt') {
          dx = this.priorCanvasX - this.board.positionData.canvasX
          dy = this.priorCanvasY - this.board.positionData.canvasY
          this.priorCanvasX = this.board.positionData.canvasX
          this.priorCanvasY = this.board.positionData.canvasY
          this.board.tilt(Math.PI * (dx + dy) * this.rotateMultiplier)
        } else if (this.mode === 'spin') {
          const dx = this.priorCanvasX - this.board.positionData.canvasX
          const dy = this.priorCanvasY - this.board.positionData.canvasY
          this.priorCanvasX = this.board.positionData.canvasX
          this.priorCanvasY = this.board.positionData.canvasY
          this.board.spin(Math.PI * (dx + dy) * this.rotateMultiplier)
        } else if (this.mode === 'zoom') {
          dx = this.priorCanvasX - this.board.positionData.canvasX
          dy = this.priorCanvasY - this.board.positionData.canvasY
          this.priorCanvasX = this.board.positionData.canvasX
          this.priorCanvasY = this.board.positionData.canvasY
          this.board.zoom((dx + dy) * this.zoomMultiplier)
        }
      }
    })
  }

  setMode(mode: string) {
    this.mode = mode
  }
}
