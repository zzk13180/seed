import {
  DynamicTexture,
  StandardMaterial,
  Mesh,
  Color3,
  MeshBuilder,
} from '@babylonjs/core'

export class SphereMeshFactory {
  constructor(private hexDefinition) {}
  getMesh(item, scene) {
    const diameter = (item.size * this.hexDefinition.hexagon_edge_to_edge_width) / 100
    const sphere = MeshBuilder.CreateSphere(
      item.id,
      {
        segments: 16,
        diameter,
      },
      scene,
    )

    const latitudeTexture = new DynamicTexture('dynamic texture', 512, scene, true)
    const latitudeMaterial = new StandardMaterial('mat', scene)
    latitudeMaterial.diffuseTexture = latitudeTexture
    latitudeMaterial.specularColor = new Color3(0, 0, 0)
    latitudeMaterial.emissiveColor = new Color3(0.1, 0.1, 0.1)
    latitudeMaterial.backFaceCulling = true

    sphere.material = latitudeMaterial

    let context = latitudeTexture.getContext()
    let size = latitudeTexture.getSize()

    context.fillStyle = item.backgroundColor
    context.fillRect(0, 0, size.width, size.height)

    let lineWidth = 2 * (item.lineWidth / (Math.PI * diameter)) * size.height
    context.lineWidth = lineWidth
    context.strokeStyle = item.lineColor

    context.beginPath()
    context.moveTo(0, size.height / 2)
    context.lineTo(size.width, size.height / 2)
    context.stroke()

    context.beginPath()
    context.moveTo(0, size.height / 6)
    context.lineTo(size.width, size.height / 6)
    context.stroke()

    context.beginPath()
    context.moveTo(0, size.height / 3)
    context.lineTo(size.width, size.height / 3)
    context.stroke()

    context.beginPath()
    context.moveTo(0, (2 * size.height) / 3)
    context.lineTo(size.width, (2 * size.height) / 3)
    context.stroke()

    context.beginPath()
    context.moveTo(0, (5 * size.height) / 6)
    context.lineTo(size.width, (5 * size.height) / 6)
    context.stroke()

    context.lineWidth = 1
    const equatorLineWidth = lineWidth
    for (let i = 0; i < size.height; i++) {
      lineWidth =
        equatorLineWidth *
        ((Math.PI * diameter) /
          (2 * Math.PI * (diameter * Math.sin((Math.PI * i) / size.height))))

      context.beginPath()
      context.moveTo(size.height / 12 - 0.5 * lineWidth, i + 0.5)
      context.lineTo(size.height / 12 + 0.5 * lineWidth, i + 0.5)
      context.stroke()

      context.beginPath()
      context.moveTo(size.height / 4 - 0.5 * lineWidth, i + 0.5)
      context.lineTo(size.height / 4 + 0.5 * lineWidth, i + 0.5)
      context.stroke()

      context.beginPath()
      context.moveTo((5 * size.height) / 12 - 0.5 * lineWidth, i + 0.5)
      context.lineTo((5 * size.height) / 12 + 0.5 * lineWidth, i + 0.5)
      context.stroke()

      context.beginPath()
      context.moveTo((7 * size.height) / 12 - 0.5 * lineWidth, i + 0.5)
      context.lineTo((7 * size.height) / 12 + 0.5 * lineWidth, i + 0.5)
      context.stroke()

      context.beginPath()
      context.moveTo((9 * size.height) / 12 - 0.5 * lineWidth, i + 0.5)
      context.lineTo((9 * size.height) / 12 + 0.5 * lineWidth, i + 0.5)
      context.stroke()

      context.beginPath()
      context.moveTo((11 * size.height) / 12 - 0.5 * lineWidth, i + 0.5)
      context.lineTo((11 * size.height) / 12 + 0.5 * lineWidth, i + 0.5)
      context.stroke()
    }
    latitudeTexture.update(true)
    if (item.borderStar) {
      latitudeMaterial.emissiveColor = new Color3(1, 1, 1)
      const corona = MeshBuilder.CreateDisc(
        't',
        {
          radius: diameter / 2 + item.borderStar.radius2,
          tessellation: 20,
          sideOrientation: Mesh.DOUBLESIDE,
        },
        scene,
      )

      const coronaTexture = new DynamicTexture('dynamic texture', 512, scene, true)
      coronaTexture.hasAlpha = true

      const coronaMaterial = new StandardMaterial('mat', scene)
      coronaMaterial.emissiveColor = new Color3(1, 1, 1)
      coronaMaterial.diffuseTexture = coronaTexture
      corona.material = coronaMaterial

      context = coronaTexture.getContext()
      size = coronaTexture.getSize()

      let rot = (Math.PI / 2) * 3
      let x = size.width / 2
      let y = size.height / 2
      const step = Math.PI / item.borderStar.points

      const outerRadius = size.width / 2
      const innerRadius =
        (outerRadius / (diameter / 2 + item.borderStar.radius2)) *
        (diameter / 2 + item.borderStar.radius1)

      context.beginPath()
      context.moveTo(size.width / 2, size.height / 2 - outerRadius)
      for (let i = 0; i < item.borderStar.points; i++) {
        x = size.width / 2 + Math.cos(rot) * outerRadius
        y = size.height / 2 + Math.sin(rot) * outerRadius
        context.lineTo(x, y)
        rot += step

        x = size.width / 2 + Math.cos(rot) * innerRadius
        y = size.height / 2 + Math.sin(rot) * innerRadius
        context.lineTo(x, y)
        rot += step
      }
      context.lineTo(size.width / 2, size.height / 2 - outerRadius)
      context.closePath()
      context.lineWidth = 0
      context.strokeStyle = item.lineColor
      context.stroke()
      context.fillStyle = item.borderStar.borderColor
      context.fill()
      coronaTexture.update(true)

      corona.billboardMode = Mesh.BILLBOARDMODE_ALL
      corona.parent = sphere
    }

    sphere.rotation.x = Math.PI / 2

    sphere.data = {}
    sphere.data.item = item
    return sphere
  }
}
