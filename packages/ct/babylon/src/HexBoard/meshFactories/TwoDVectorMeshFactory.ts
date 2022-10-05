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

export class TwoDVectorMeshFactory {
  offScreenCanvasMap: any = {}
  imageDataWidth = 0
  imageDataHeight = 0
  pixelArray
  constructor(private readonly hexDefinition: Hexagonal) {}

  getMesh(item: Item, scene: Scene) {
    if (!item.vectorU) item.vectorU = 1
    if (!item.vectorV) item.vectorV = 0
    if (!item.lineWidth) item.lineWidth = 3

    const vectorPixelCoordinates = this.hexDefinition.getPixelCoordinates(
      item.vectorU,
      item.vectorV,
    )

    const magnitude = Math.sqrt(
      vectorPixelCoordinates.x * vectorPixelCoordinates.x +
        vectorPixelCoordinates.y * vectorPixelCoordinates.y,
    )
    const height = this.hexDefinition.hexagon_edge_to_edge_width
    const width = magnitude + item.lineWidth
    const lineWidthRatio = item.lineWidth / width
    const square = MeshBuilder.CreatePlane('plane', { width, height }, scene)

    const myMaterial = new StandardMaterial('myMaterial', scene)

    if (item.isEmissive) {
      myMaterial.ambientColor =
        myMaterial.diffuseColor =
        myMaterial.emissiveColor =
          new Color3(1, 1, 1)
    }

    myMaterial.diffuseTexture = myMaterial.emissiveTexture = new DynamicTexture(
      'dynamic texture',
      { width, height },
      scene,
      true,
    )

    const canvas = document.createElement('canvas')
    const size = myMaterial.diffuseTexture.getSize()
    canvas.width = size.width
    this.imageDataWidth = size.width
    this.imageDataHeight = size.height
    canvas.height = size.height
    const canvasLineWidth = lineWidthRatio * size.width
    const offscreenContext = canvas.getContext('2d')
    const textureContext = myMaterial.diffuseTexture.getContext()

    offscreenContext!.lineWidth = canvasLineWidth
    // @ts-ignore TODO
    offscreenContext!.strokeStyle = item.lineColor
    offscreenContext!.lineCap = 'round'

    offscreenContext?.beginPath()
    offscreenContext?.moveTo(canvasLineWidth / 2, size.height / 2)
    offscreenContext?.lineTo(size.width - canvasLineWidth / 2, size.height / 2)
    offscreenContext?.stroke()

    offscreenContext?.beginPath()
    offscreenContext?.moveTo(size.width - canvasLineWidth / 2, size.height / 2)
    offscreenContext?.lineTo(
      size.width - size.height / 4,
      size.height / 2 + size.height / 4,
    )
    offscreenContext?.stroke()

    offscreenContext?.beginPath()
    offscreenContext?.moveTo(size.width - canvasLineWidth / 2, size.height / 2)
    offscreenContext?.lineTo(
      size.width - size.height / 4,
      size.height / 2 - size.height / 4,
    )
    offscreenContext?.stroke()

    this.pixelArray = offscreenContext?.getImageData(0, 0, size.width, size.height)
    textureContext.putImageData(this.pixelArray, 0, 0)

    myMaterial.diffuseTexture.update(true)

    myMaterial.emissiveTexture.hasAlpha = true
    myMaterial.diffuseTexture.hasAlpha = true

    myMaterial.backFaceCulling = false

    square.material = myMaterial

    if (item.angle) {
      square.rotation.z = item.angle
    }

    square.rotation.x = Math.PI

    const parentMesh = Mesh.CreateBox('Box1', 0, scene)
    parentMesh.data = {}
    square.position.x = width / 2

    parentMesh.data.hitTestAlpha = (x, y) => {
      const pixelAlpha =
        this.pixelArray.data[
          (Math.floor((1 - y) * this.imageDataWidth) * this.imageDataHeight +
            Math.floor(x * this.imageDataWidth)) *
            4 +
            3
        ]
      return pixelAlpha !== 0
    }

    square.data = parentMesh.data
    parentMesh.data.item = item
    parentMesh.visibility = 0
    square.parent = parentMesh
    return parentMesh
  }
}
