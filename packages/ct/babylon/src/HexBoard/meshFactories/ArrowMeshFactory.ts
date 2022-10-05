import {
  Mesh,
  MeshBuilder,
  Vector3,
  Color3,
  StandardMaterial,
  Scene,
} from '@babylonjs/core'
import { hex2color3 } from '@seed/common/utils/conver.util'
import type { Hexagonal } from '@seed/common/utils/hexagonal.util'
import type { Item } from '../types'

export class ArrowMeshFactory {
  constructor(private readonly hexDefinition: Hexagonal) {}

  getMesh(item: Item, scene: Scene) {
    if (!item.lineWidth) item.lineWidth = 3
    if (!item.lineColor) item.lineColor = '#000000'
    const segment1 = MeshBuilder.CreateTube(
      'segment1',
      {
        path: [
          new Vector3(-this.hexDefinition.hexagon_edge_to_edge_width / 2, 0, 0),
          new Vector3(0, -1 * this.hexDefinition.hexagon_half_wide_width, 0),
        ],
        radius: item.lineWidth,
        tessellation: 20,
        cap: 0,
      },
      scene,
    )

    const sphere1 = MeshBuilder.CreateTube(
      'sphere1',
      {
        path: [
          new Vector3(-this.hexDefinition.hexagon_edge_to_edge_width / 2, 0, 0),
          new Vector3(0, -1 * this.hexDefinition.hexagon_half_wide_width, 0),
        ],
        radius: 20,
        tessellation: 2 * item.lineWidth,
      },
      scene,
    )
    sphere1.position.x = 0
    sphere1.position.y = -1 * this.hexDefinition.hexagon_half_wide_width

    const segment2 = MeshBuilder.CreateTube(
      'segment2',
      {
        path: [
          new Vector3(0, -1 * this.hexDefinition.hexagon_half_wide_width, 0),
          new Vector3(0, -this.hexDefinition.edgeSize / 2, 0),
        ],
        radius: item.lineWidth,
        tessellation: 20,
        cap: 0,
      },
      scene,
    )

    const sphere2 = MeshBuilder.CreateSphere(
      'sphere2',
      {
        segments: 20,
        diameter: 2 * item.lineWidth,
      },
      scene,
    )
    sphere2.position.x = 0
    sphere2.position.y = -this.hexDefinition.edgeSize / 2

    const segment3 = MeshBuilder.CreateTube(
      'new',
      {
        path: [
          new Vector3(0, -this.hexDefinition.edgeSize / 2, 0),
          new Vector3(
            this.hexDefinition.hexagon_edge_to_edge_width / 2,
            -this.hexDefinition.edgeSize / 2,
            0,
          ),
        ],
        radius: item.lineWidth,
        tessellation: 20,
        cap: 0,
      },
      scene,
    )

    const sphere3 = MeshBuilder.CreateSphere(
      'sphere1',
      {
        segments: 20,
        diameter: 2 * item.lineWidth,
      },
      scene,
    )
    sphere3.position.x = this.hexDefinition.hexagon_edge_to_edge_width / 2
    sphere3.position.y = -this.hexDefinition.edgeSize / 2

    const segment4 = MeshBuilder.CreateTube(
      'segment4',
      {
        path: [
          new Vector3(
            this.hexDefinition.hexagon_edge_to_edge_width / 2,
            -this.hexDefinition.edgeSize / 2,
            0,
          ),
          new Vector3(
            this.hexDefinition.hexagon_edge_to_edge_width / 2,
            this.hexDefinition.edgeSize / 2,
            0,
          ),
        ],
        radius: item.lineWidth,
        tessellation: 20,
        cap: 0,
      },
      scene,
    )

    const sphere4 = MeshBuilder.CreateSphere(
      'sphere4',
      {
        segments: 20,
        diameter: 2 * item.lineWidth,
      },
      scene,
    )
    sphere4.position.x = this.hexDefinition.hexagon_edge_to_edge_width / 2
    sphere4.position.y = this.hexDefinition.edgeSize / 2

    const segment5 = MeshBuilder.CreateTube(
      'segment5',
      {
        path: [
          new Vector3(
            this.hexDefinition.hexagon_edge_to_edge_width / 2,
            this.hexDefinition.edgeSize / 2,
            0,
          ),
          new Vector3(0, this.hexDefinition.edgeSize / 2, 0),
        ],
        radius: item.lineWidth,
        tessellation: 20,
        cap: 0,
      },
      scene,
    )

    const sphere5 = MeshBuilder.CreateSphere(
      'sphere5',
      {
        segments: 20,
        diameter: 2 * item.lineWidth,
      },
      scene,
    )
    sphere5.position.x = 0
    sphere5.position.y = this.hexDefinition.edgeSize / 2

    const segment6 = MeshBuilder.CreateTube(
      'segment6',
      {
        path: [
          new Vector3(0, this.hexDefinition.edgeSize / 2, 0),
          new Vector3(0, this.hexDefinition.hexagon_half_wide_width, 0),
        ],
        radius: item.lineWidth,
        tessellation: 20,
        cap: 0,
      },
      scene,
    )

    const sphere6 = MeshBuilder.CreateSphere(
      'sphere1',
      {
        segments: 20,
        diameter: 2 * item.lineWidth,
      },
      scene,
    )
    sphere6.position.x = 0
    sphere6.position.y = this.hexDefinition.hexagon_half_wide_width

    const segment7 = MeshBuilder.CreateTube(
      'segment7',
      {
        path: [
          new Vector3(0, this.hexDefinition.hexagon_half_wide_width, 0),
          new Vector3(-this.hexDefinition.hexagon_edge_to_edge_width / 2, 0, 0),
        ],
        radius: item.lineWidth,
        tessellation: 20,
        cap: 0,
      },
      scene,
    )

    const sphere7 = MeshBuilder.CreateSphere(
      'sphere7',
      {
        segments: 20,
        diameter: 2 * item.lineWidth,
      },
      scene,
    )
    sphere7.position.x = -this.hexDefinition.hexagon_edge_to_edge_width / 2
    sphere7.position.y = 0

    let arrow: any = Mesh.MergeMeshes([
      segment1,
      segment2,
      segment3,
      segment4,
      segment5,
      segment6,
      segment7,
      sphere1,
      sphere2,
      sphere3,
      sphere4,
      sphere5,
      sphere6,
      sphere7,
    ])

    const arrowBorderMaterial = new StandardMaterial('arrowBorderMaterial', scene)
    let rgb = hex2color3(item.lineColor)
    arrowBorderMaterial.diffuseColor = new Color3(rgb.r / 256, rgb.g / 256, rgb.b / 256)
    arrowBorderMaterial.emissiveColor = new Color3(rgb.r / 256, rgb.g / 256, rgb.b / 256)
    arrow.material = arrowBorderMaterial

    if (item.fillColor) {
      const shape1 = [
        new Vector3(0, -1 * this.hexDefinition.hexagon_half_wide_width, 0),
        new Vector3(0, this.hexDefinition.hexagon_half_wide_width, 0),
        new Vector3(-this.hexDefinition.hexagon_edge_to_edge_width / 2, 0, 0),
        new Vector3(0, -1 * this.hexDefinition.hexagon_half_wide_width, 0),
      ]

      const shape2 = [
        new Vector3(0, this.hexDefinition.edgeSize / 2, 0),
        new Vector3(0, -this.hexDefinition.edgeSize / 2, 0),
        new Vector3(
          this.hexDefinition.hexagon_edge_to_edge_width / 2,
          -this.hexDefinition.edgeSize / 2,
          0,
        ),
        new Vector3(
          this.hexDefinition.hexagon_edge_to_edge_width / 2,
          this.hexDefinition.edgeSize / 2,
          0,
        ),
        new Vector3(0, this.hexDefinition.edgeSize / 2, 0),
      ]

      const extrudedCenter1 = MeshBuilder.ExtrudeShape(
        'extruded',
        {
          shape: shape1,
          path: [
            new Vector3(0, 0, -1 * item.lineWidth),
            new Vector3(0, 0, item.lineWidth),
          ],
          scale: 1,
          rotation: 0,
          cap: Mesh.CAP_END,
        },
        scene,
      )

      const extrudedCenter2 = MeshBuilder.ExtrudeShape(
        'extruded',
        {
          shape: shape2,
          path: [
            new Vector3(0, 0, -1 * item.lineWidth),
            new Vector3(0, 0, item.lineWidth),
          ],
          scale: 1,
          rotation: 0,
          cap: Mesh.CAP_END,
        },
        scene,
      )

      const arrowCenterMaterial = new StandardMaterial('arrowCenterMaterial', scene)
      rgb = hex2color3(item.fillColor)
      arrowCenterMaterial.diffuseColor = new Color3(rgb.r / 256, rgb.g / 256, rgb.b / 256)
      arrowCenterMaterial.specularColor = new Color3(
        rgb.r / 256,
        rgb.g / 256,
        rgb.b / 256,
      )
      arrowCenterMaterial.emissiveColor = new Color3(
        rgb.r / 256,
        rgb.g / 256,
        rgb.b / 256,
      )
      extrudedCenter1.material = arrowCenterMaterial

      arrow = Mesh.MergeMeshes([arrow, extrudedCenter1, extrudedCenter2])
    }

    arrow.scaling.x = item.scaleLength
    arrow.scaling.y = item.scaleWidth
    arrow.rotation.z = ((item.rotation || 0) * Math.PI) / 180

    arrow.data = {}
    arrow.data.item = item
    return arrow
  }
}
