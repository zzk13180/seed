// stack items within the same cell
export class ZStackingPipelineNode {
  cellGroupsMap: any = {}

  constructor(private stackStep: number) {}

  pipeData(data) {
    let i
    let mesh
    let item
    let groupKey
    let cellGroup

    for (i = 0; i < data.added.length; i++) {
      mesh = data.added[i]
      item = mesh.data.item
      groupKey = `${item.u}:${item.v}`
      if (this.cellGroupsMap[groupKey]) {
        cellGroup = this.cellGroupsMap[groupKey]
      } else {
        cellGroup = {}
        cellGroup.data = {}

        this.cellGroupsMap[groupKey] = cellGroup
        cellGroup.mouseDown = false
        cellGroup.drawnItemCount = 0

        cellGroup.previousDrawnItem = cellGroup
        cellGroup.nextDrawnItem = cellGroup

        cellGroup.position = {}
        cellGroup.position.z = -1 * this.stackStep
      }
      mesh.position.z = cellGroup.previousDrawnItem.position.z + this.stackStep
      mesh.data.baseZ = mesh.position.z
      if (cellGroup.previousDrawnItem.data && cellGroup.previousDrawnItem.data.height) {
        mesh.position.z = mesh.position.z + cellGroup.previousDrawnItem.data.height / 2
        mesh.data.baseZ = mesh.position.z
      }

      if (mesh.data && mesh.data.height) {
        mesh.position.z = mesh.position.z + mesh.data.height / 2
      }

      cellGroup.previousDrawnItem.nextDrawnItem = mesh
      mesh.previousDrawnItem = cellGroup.previousDrawnItem
      cellGroup.previousDrawnItem = mesh
      mesh.nextDrawnItem = cellGroup
      return { added: data.added, removed: data.removed, updated: data.updated }
    }
  }
}
