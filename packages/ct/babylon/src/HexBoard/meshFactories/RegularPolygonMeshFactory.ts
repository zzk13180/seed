import { Mesh, MeshBuilder, Color3, StandardMaterial, Scene } from '@babylonjs/core'
import { hex2color3 } from '@seed/common/utils/conver.util'
import type { Hexagonal } from '@seed/common/utils/hexagonal.util'
import type { Item } from '../types'

export class RegularPolygonMeshFactory {
  internalId = 0
  constructor(private readonly hexDefinition: Hexagonal) {}

  getMesh(item: Item, scene: Scene) {
    const cylinder = MeshBuilder.CreateCylinder(
      item.id,
      {
        diameterTop: item.diameter - item.thickness,
        diameterBottom: item.diameter,
        tessellation: item.sides,
        height: item.thickness,
        sideOrientation: Mesh.DOUBLESIDE,
      },
      scene,
    )
    const material = new StandardMaterial(`textureX${this.internalId}`, scene)
    const rgb = hex2color3(item.color)
    material.diffuseColor = new Color3(rgb.r / 256, rgb.g / 256, rgb.b / 256)
    material.ambientColor = material.diffuseColor
    cylinder.material = material

    this.internalId++
    cylinder.data = {}
    cylinder.data.item = item
    cylinder.rotation.y = -Math.PI / 2
    cylinder.rotation.z = -Math.PI / 2
    return cylinder
  }
}
