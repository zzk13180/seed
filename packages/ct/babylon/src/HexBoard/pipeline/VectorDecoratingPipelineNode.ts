export class VectorDecoratingPipelineNode {
  constructor(private vectorFactory, private scene) {}
  pipeData(data) {
    let mesh
    if (!data || !data.added) return
    for (let i = 0; i < data.added.length; i++) {
      mesh = data.added[i]

      if (mesh.data && mesh.data.item && mesh.data.item.vectors) {
        for (let j = 0; j < mesh.data.item.vectors.length; j++) {
          const vector = this.vectorFactory.getMesh(mesh.data.item.vectors[j], this.scene)
          vector.position.x = mesh.position.x
          vector.position.y = mesh.position.y
          vector.position.z = mesh.data.baseZ - (j + 1)
        }
      }
    }
  }
}
