export class PlanarPositioningPipelineNode {
  constructor(private hexDimensions) {}
  pipeData(data) {
    let i, mesh, item, pixelCoordinates
    for (i = 0; i < data.added.length; i++) {
      mesh = data.added[i]
      item = mesh.data.item
      pixelCoordinates = this.hexDimensions.getPixelCoordinates(item.u, item.v)
      mesh.position.x = pixelCoordinates.x
      mesh.position.y = pixelCoordinates.y
    }
    for (i = 0; i < data.updated.length; i++) {
      mesh = data.updated[i]
      item = mesh.data.item
      if (item.skipCellCentering) {
        continue
      }
      pixelCoordinates = this.hexDimensions.getPixelCoordinates(item.u, item.v)
      mesh.position.x = pixelCoordinates.x
      mesh.position.y = pixelCoordinates.y
    }
    return { added: data.added, removed: data.removed, updated: data.updated }
  }
}
