import {
  StandardMaterial,
  Color3,
  Vector3,
  SolidParticleSystem,
  MeshBuilder,
  Nullable,
  Scene,
  Mesh,
} from '@babylonjs/core'
import { hex2color3 } from '@seed/common/utils/conver.util'
import { onGlobalEvent } from '@seed/common/utils/event.util'
import type { HexBoard } from '../HexBoard'

export class StarryContext {
  private scene: Nullable<Scene> | undefined
  private outerRadius: number
  private starColors: string[]
  private mesh: Mesh

  constructor(private readonly board: HexBoard, private readonly density) {
    this.scene = this.board.scene
    this.outerRadius = this.board.camera.maxZ - 50
    this.starColors = ['#ffffff', '#ffe9c4', '#d4fbff']

    const myPositionFunction = (particle) => {
      const u = Math.random()
      const v = Math.random()
      const longitude = 2 * Math.PI * u
      const colatitude = Math.acos(v - 1)
      particle.position.x = this.outerRadius * Math.sin(colatitude) * Math.cos(longitude)
      particle.position.y = this.outerRadius * Math.sin(colatitude) * Math.sin(longitude)
      particle.position.z = this.outerRadius * Math.cos(colatitude)
      const color = hex2color3(this.starColors[Math.round(Math.random() * (2 - 0) + 0)])

      particle.color = new Color3(color.r / 256, color.g / 256, color.b / 256)

      const scale = Math.random() * 0.2 + 0.2
      particle.scaling = new Vector3(scale, scale, scale)
    }

    const hexagon = MeshBuilder.CreateSphere(
      't',
      {
        segments: 16,
        diameter: 100,
      },
      this.scene,
    )

    const SPS = new SolidParticleSystem('SPS', this.scene, {
      updatable: true,
      isPickable: false,
    })

    SPS.addShape(hexagon, this.density, { positionFunction: myPositionFunction })
    SPS.billboard = true
    const mesh = SPS.buildMesh()
    const material = new StandardMaterial('texture1', this.scene)
    material.emissiveColor = new Color3(1, 1, 1)
    mesh.material = material
    this.mesh = mesh
    hexagon.dispose()
    this.mesh.position.y = 1000
    this.mesh.position.z = 1000
    onGlobalEvent('updateCameraPosition', () => {
      this.mesh.position.x = this.board.positionData.cameraX
      this.mesh.position.y = this.board.positionData.cameraY
      this.mesh.position.z = this.board.positionData.cameraZ
    })
  }
}
