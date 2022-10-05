import {
  Mesh,
  MeshBuilder,
  DynamicTexture,
  Color3,
  StandardMaterial,
  Scene,
} from '@babylonjs/core'
import type { Hexagonal } from '@seed/common/utils/hexagonal.util'
import type { Item } from '../types'

export class ImageMeshFactory {
  offScreenCanvasMap: any = {}
  constructor(private readonly hexDefinition: Hexagonal) {}

  getMesh(item: Item, scene: Scene) {
    if (!item.size) item.size = 50
    if (!item.img) item.img = './test.svg'

    const diameter = (item.size * this.hexDefinition.hexagon_edge_to_edge_width) / 100

    const square = MeshBuilder.CreatePlane('plane', { size: diameter }, scene)

    const myMaterial = new StandardMaterial('myMaterial', scene)

    if (item.isEmissive) {
      myMaterial.emissiveColor = new Color3(1, 1, 1)
    }

    const imageHolder = this.offScreenCanvasMap[item.img]
    myMaterial.diffuseTexture = myMaterial.emissiveTexture = new DynamicTexture(
      'dynamic texture',
      512,
      scene,
      true,
    )

    const textureContext = myMaterial.diffuseTexture.getContext()

    textureContext.fillStyle = '#653700'
    textureContext.fillRect(0, 0, diameter / 2, diameter / 2)
    textureContext.fillRect(diameter / 2, diameter / 2, diameter, diameter)
    textureContext.fillStyle = '#000000'
    textureContext.fillRect(diameter / 2, 0, diameter, diameter / 2)
    textureContext.fillRect(0, diameter / 2, diameter / 2, diameter)
    myMaterial.diffuseTexture.update(true)
    if (!imageHolder) {
      const img = document.createElement('img')
      img.width = 512
      img.height = 512
      img.onload = () => {
        const canvas = document.createElement('canvas')

        canvas.width = 512
        canvas.height = 512
        const offscreenContext = canvas.getContext('2d')
        offscreenContext?.drawImage(
          img,
          0,
          0,
          img.width,
          img.height,
          0,
          0,
          canvas.width,
          canvas.height,
        )

        textureContext.putImageData(offscreenContext?.getImageData(0, 0, 512, 512), 0, 0)
        this.offScreenCanvasMap[item.img] = {
          offscreenContext,
        }
        this.offScreenCanvasMap[item.img].pixelArray = offscreenContext?.getImageData(
          0,
          0,
          512,
          512,
        ).data
        myMaterial.diffuseTexture.update(true)
      }
      img.src = item.img
    } else {
      const offscreenContext = imageHolder.offscreenContext
      textureContext.putImageData(offscreenContext.getImageData(0, 0, 512, 512), 0, 0)
    }

    myMaterial.emissiveTexture.hasAlpha = true
    myMaterial.diffuseTexture.hasAlpha = true
    myMaterial.backFaceCulling = false

    square.material = myMaterial

    if (item.angle) {
      square.rotation.z = item.angle
    }

    square.data = {}

    square.rotation.x = Math.PI
    if (item.vertical) {
      square.rotation.y = -Math.PI / 2

      square.billboardMode = Mesh.BILLBOARDMODE_Z

      square.data.height = diameter
    }

    square.data.hitTestAlpha = (x, y) => {
      const pixelAlpha =
        this.offScreenCanvasMap[item.img].pixelArray[
          (Math.floor((1 - y) * 512) * 512 + Math.floor(x * 512)) * 4 + 3
        ]
      return pixelAlpha !== 0
    }
    square.data.item = item
    return square
  }
}
