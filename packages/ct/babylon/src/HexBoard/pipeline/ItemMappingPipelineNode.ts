export class ItemMappingPipelineNode {
  meshMap: any = {}
  constructor(private meshFactoryMap, private scene) {}

  pipeData(data: any) {
    let mesh: any = null
    let item: any = null
    const removed: any[] = []
    const added: any[] = []
    for (let i = 0; i < data.removed.length; i++) {
      item = data.removed[i]
      mesh = this.meshMap[item.id]
      mesh.dispose()
      delete this.meshMap[item.id]
      removed.push(mesh)
    }
    for (let i = 0; i < data.added.length; i++) {
      item = data.added[i]
      if (this.meshFactoryMap[item.type]) {
        mesh = this.meshFactoryMap[item.type](item, this.scene)
      }
      if (!mesh) {
        continue
      }
      if (!mesh.data) {
        mesh.data = {}
      }
      mesh.data.item = item
      added.push(mesh)
      this.meshMap[item.id] = mesh
    }
    return { added, removed, updated: data.updated }
  }
}
