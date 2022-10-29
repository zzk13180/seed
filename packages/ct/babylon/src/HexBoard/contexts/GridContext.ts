import {
  StandardMaterial,
  Color3,
  Color4,
  SolidParticleSystem,
  Mesh,
  Scene,
  MeshBuilder,
} from '@babylonjs/core'
import { onGlobalEvent } from '@seed/common/utils/event.util'
import type { Hexagonal } from '@seed/common/utils/hexagonal.util'
import type { HexBoard } from '../HexBoard'

export class GridContext {
  private scene: Scene
  private middleX = 0
  private middleY = 0
  private positionArray: { x: number; y: number }[]
  private gridParent: Mesh

  // eslint-disable-next-line max-params
  constructor(
    private readonly hexDimensions: Hexagonal,
    private readonly board: HexBoard,
    private readonly color: Color3,
    private readonly radius: number,
    private readonly fadeRadius: number,
    private readonly baseAlpha: number,
  ) {
    this.scene = this.board.scene
    this.positionArray = this.createPositionArray(hexDimensions, radius + fadeRadius)

    const hexagon = MeshBuilder.CreateDisc(
      't',
      {
        radius: hexDimensions.hexagon_half_wide_width - 2,
        tessellation: 6,
        sideOrientation: Mesh.DOUBLESIDE,
      },
      this.scene,
    )

    const nb = this.positionArray.length
    const SPS = new SolidParticleSystem('SPS', this.scene, {
      updatable: true,
      isPickable: false,
    })
    SPS.addShape(hexagon, nb)

    SPS.initParticles = () => {
      for (let p = 0; p < SPS.nbParticles; p++) {
        SPS.particles[p]!.position.x = this.positionArray[p]!.x
        SPS.particles[p]!.position.y = this.positionArray[p]!.y
        SPS.particles[p]!.position.z = 0
        SPS.particles[p]!.rotation.z = Math.PI / 2
        const distanceFromViewPoint = Math.sqrt(
          Math.pow(SPS.particles[p]!.position.x - this.middleX, 2) +
            Math.pow(SPS.particles[p]!.position.y - this.middleY, 2),
        )
        let alpha = this.baseAlpha
        if (
          distanceFromViewPoint > this.radius * this.hexDimensions.hexagon_narrow_width &&
          distanceFromViewPoint <
            (this.radius + this.fadeRadius) * this.hexDimensions.hexagon_narrow_width
        ) {
          alpha =
            (-this.baseAlpha /
              (this.fadeRadius * this.hexDimensions.hexagon_narrow_width)) *
              (distanceFromViewPoint -
                this.radius * this.hexDimensions.hexagon_narrow_width) +
            this.baseAlpha
        } else if (
          distanceFromViewPoint >=
          (this.radius + this.fadeRadius) * this.hexDimensions.hexagon_narrow_width
        ) {
          alpha = 0
        }
        SPS.particles[p]!.color = new Color4(
          this.color.r / 256,
          this.color.g / 256,
          this.color.b / 256,
          alpha,
        )
      }
    }

    const mesh = SPS.buildMesh()
    mesh.hasVertexAlpha = true
    SPS.initParticles()
    SPS.setParticles()
    this.gridParent = mesh
    const material = new StandardMaterial('texture1', this.scene)
    material.emissiveColor = new Color3(1, 1, 1)
    mesh.material = material
    hexagon.dispose()
    onGlobalEvent('updatePosition', () => {
      const hexCoordinates = this.hexDimensions.getReferencePoint(
        this.board.positionData.middleX,
        this.board.positionData.middleY,
      )
      const centerHexPixelCoordinates = this.hexDimensions.getPixelCoordinates(
        hexCoordinates.u,
        hexCoordinates.v,
      )
      this.gridParent.position.x = centerHexPixelCoordinates.x
      this.gridParent.position.y = centerHexPixelCoordinates.y
      this.middleX = this.board.positionData.middleX - centerHexPixelCoordinates.x
      this.middleY = this.board.positionData.middleY - centerHexPixelCoordinates.y
      SPS.setParticles()
    })
  }

  createPositionArray(hexDimensions: Hexagonal, radius: number) {
    const positionArray: { x: number; y: number }[] = []
    let pixelCoordinates
    let u = 0
    let v = 0
    positionArray.push({ y: 0, x: 0 })
    for (let i = 1; i < radius + 1; i++) {
      for (v = -i; v <= 0; v++) {
        pixelCoordinates = hexDimensions.getPixelCoordinates(i, v)

        positionArray.push({ y: pixelCoordinates.y, x: pixelCoordinates.x })
      }
      for (v = 0; v <= i; v++) {
        pixelCoordinates = hexDimensions.getPixelCoordinates(-i, v)
        positionArray.push({ y: pixelCoordinates.y, x: pixelCoordinates.x })
      }
      for (u = -i + 1; u <= 0; u++) {
        pixelCoordinates = hexDimensions.getPixelCoordinates(u, i)
        positionArray.push({ y: pixelCoordinates.y, x: pixelCoordinates.x })
      }
      for (u = 0; u < i; u++) {
        pixelCoordinates = hexDimensions.getPixelCoordinates(u, -i)
        positionArray.push({ y: pixelCoordinates.y, x: pixelCoordinates.x })
      }
      for (u = -i + 1, v = -1; v > -i; u++, v--) {
        pixelCoordinates = hexDimensions.getPixelCoordinates(u, v)
        positionArray.push({ y: pixelCoordinates.y, x: pixelCoordinates.x })
      }
      for (u = i - 1, v = 1; v < i; u--, v++) {
        pixelCoordinates = hexDimensions.getPixelCoordinates(u, v)
        positionArray.push({ y: pixelCoordinates.y, x: pixelCoordinates.x })
      }
    }

    return positionArray
  }
}
