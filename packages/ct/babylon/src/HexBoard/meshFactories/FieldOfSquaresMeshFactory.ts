import { MeshBuilder, Color3, StandardMaterial, Scene } from '@babylonjs/core'
import { hex2color3 } from '@seed/common/utils/conver.util'
import type { Hexagonal } from '@seed/common/utils/hexagonal.util'
import type { Item } from '../types'
export class FieldOfSquaresMeshFactory {
  constructor(
    private readonly hexDefinition: Hexagonal,
    private minSize,
    private maxSize,
    private colors,
  ) {}

  getMesh(item: Item, scene: Scene) {
    const cube1 = this.createSquare(
      (-1 * this.hexDefinition.hexagon_edge_to_edge_width) / 2,
      0,
      -1 * this.hexDefinition.hexagon_half_wide_width,
      0,
      scene,
    )
    const cube2 = this.createSquare(
      0,
      this.hexDefinition.hexagon_edge_to_edge_width / 2,
      -1 * this.hexDefinition.hexagon_half_wide_width,
      0,
      scene,
    )
    const cube3 = this.createSquare(
      0,
      this.hexDefinition.hexagon_edge_to_edge_width / 2,
      0,
      this.hexDefinition.hexagon_half_wide_width,
      scene,
    )
    const cube4 = this.createSquare(
      (-1 * this.hexDefinition.hexagon_edge_to_edge_width) / 2,
      0,
      0,
      this.hexDefinition.hexagon_half_wide_width,
      scene,
    )

    const parentMesh = MeshBuilder.CreateBox('Box1', { size: 0 }, scene)
    parentMesh.data = {}
    parentMesh.data.item = item
    parentMesh.visibility = 0
    cube1.parent = cube2.parent = cube3.parent = cube4.parent = parentMesh

    return parentMesh
  }

  createSquare(minX, maxX, minY, maxY, scene) {
    let x = this.random(minX, maxX)
    let y = this.random(minY, maxY)
    let hexCoords = this.hexDefinition.getReferencePoint(x, y)

    while (hexCoords.u !== 0 || hexCoords.v !== 0) {
      x = this.random(minX, maxX)
      y = this.random(minY, maxY)
      hexCoords = this.hexDefinition.getReferencePoint(x, y)
    }

    const color = this.colors[this.random(0, this.colors.length - 1)]

    const size = this.random(this.minSize, this.maxSize)

    const box = MeshBuilder.CreateBox('Box1', { size }, scene)
    box.rotation.x = (Math.random() * Math.PI) / 2
    box.rotation.y = (Math.random() * Math.PI) / 2
    box.rotation.z = (Math.random() * Math.PI) / 2
    box.position.x = x
    box.position.y = y
    const material = new StandardMaterial('textureX', scene)
    const rgb = hex2color3(color)
    material.diffuseColor = new Color3(rgb.r / 256, rgb.g / 256, rgb.b / 256)
    box.material = material

    return box
  }

  random(min, max) {
    return Math.round(Math.random() * (max - min) + min)
  }
}
