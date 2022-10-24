import {
  IPointerEvent,
  Ray,
  AbstractMesh,
  Scene,
  ArcRotateCamera,
  Vector3,
  Engine,
  Color4,
  Color3,
  Plane,
  FxaaPostProcess,
  Matrix,
  PointLight,
  Nullable,
} from '@babylonjs/core'
import { Hexagonal } from '@seed/common/utils/hexagonal.util'
import { hex2color3 } from '@seed/common/utils/conver.util'
import { emitGlobalEvent } from '@seed/common/utils/event.util'
import { GridContext } from './contexts/GridContext'
import { StarryContext } from './contexts/StarryContext'
import { CameraControllingMouseListener } from './listeners/CameraControllingMouseListener'

export class HexBoard {
  hexDimensions: Hexagonal
  engine: Engine
  scene: Scene
  camera: ArcRotateCamera
  positionData = {
    cameraX: 0,
    cameraY: 0,
    cameraZ: 0,
    middleX: 0,
    middleY: 0,
    mapX: 0,
    mapY: 0,
    canvasX: 0,
    canvasY: 0,
    clickedItem: null,
    mouseMoved: false,
  }

  private plane: Plane
  private cameraAlpha = Math.PI / 4
  private cameraBeta = 0
  private cameraRadius = Math.sqrt(1000 * 1000)
  private readonly radiusMin = 100
  private readonly alphaMax = Math.PI / 2 - Math.PI / 360
  private readonly alphaMin = Math.PI / 6
  private cameraTargetX = 0
  private cameraTargetY = 0
  private down = false
  private mouseMoved = false
  private initialDownX = 0
  private initialDownY = 0

  constructor(
    private readonly canvas: HTMLCanvasElement,
    private readonly rgb: { r: number; g: number; b: number } = hex2color3('#000000'),
  ) {
    this.engine = new Engine(this.canvas, true)
    this.scene = new Scene(this.engine)
    this.camera = new ArcRotateCamera(
      'ArcRotateCamera',
      0,
      0,
      0,
      Vector3.Zero(),
      this.scene,
    )
    new FxaaPostProcess('fxaa', 1.0, this.camera)
    this.plane = Plane.FromPositionAndNormal(Vector3.Zero(), new Vector3(0, 0, 1))
    this.hexDimensions = new Hexagonal(55, 1)
    this.camera.upVector = new Vector3(0, 0, 1)
    this.camera.upperBetaLimit = Math.PI
    this.camera.allowUpsideDown = true
    this.camera.setTarget(Vector3.Zero())
    this.scene.clearColor = new Color4(
      this.rgb.r / 256,
      this.rgb.g / 256,
      this.rgb.b / 256,
    )
    this.scene.ambientColor = new Color3(0.3, 0.3, 0.3)
    this.scene.onPointerDown = this.handlePointerDown.bind(this)
    this.scene.onPointerMove = this.handlePointerMove.bind(this)
    this.scene.onPointerUp = this.handlePointerUp.bind(this)
    this.engine.runRenderLoop(() => this.scene.render())
    this.updateCameraPosition()
  }

  private handlePointerDown(e: IPointerEvent) {
    e.preventDefault()
    this.down = true
    this.mouseMoved = false

    const pageX = e.pageX
    const pageY = e.pageY

    const relativeX = pageX - this.canvas.offsetLeft
    this.initialDownX = relativeX
    const relativeY = pageY - this.canvas.offsetTop
    this.initialDownY = relativeY

    const tRay: Ray = this.scene.createPickingRay(
      relativeX,
      relativeY,
      Matrix.Identity(),
      this.camera,
    )
    const pickResult: any = this.intersectRayPlane(tRay, this.plane)

    if (!pickResult) return

    const mousePickResult = this.scene.pick(relativeX, relativeY, (mesh) => {
      const getFurthestAncestor = (mesh: AbstractMesh) => {
        if (mesh.parent) {
          return getFurthestAncestor(mesh.parent)
        }
        return mesh
      }
      const furthestAncestor = getFurthestAncestor(mesh)
      if (furthestAncestor.data && furthestAncestor.data.item) {
        if (mesh.data && mesh.data.hitTestAlpha) {
          const meshPickResult = this.scene.pick(
            relativeX,
            relativeY,
            function (predicateMesh) {
              return (
                predicateMesh.data &&
                predicateMesh.data.item &&
                predicateMesh.data.item.id === mesh.data.item.id
              )
            },
          )

          if (meshPickResult && meshPickResult.hit) {
            const textureCoordinates = meshPickResult.getTextureCoordinates()
            return mesh.data.hitTestAlpha(textureCoordinates?.x, textureCoordinates?.y)
          }
        }
        return true
      }
      return false
    })

    let clickedItem: any = null
    if (mousePickResult?.hit) {
      clickedItem = mousePickResult.pickedMesh
      if (clickedItem.parent) {
        clickedItem = clickedItem.parent
      }
    }

    this.positionData.canvasX = relativeX
    this.positionData.canvasY = relativeY
    this.positionData.mapX = pickResult.x
    this.positionData.mapY = pickResult.y
    this.positionData.clickedItem = clickedItem

    emitGlobalEvent('pointerDown', this.positionData)
  }

  private handlePointerMove(e: IPointerEvent) {
    e.preventDefault()
    if (this.down === false) {
      return
    }
    const pageX = e.pageX
    const pageY = e.pageY

    const relativeX = pageX - this.canvas.offsetLeft
    const relativeY = pageY - this.canvas.offsetTop

    if (
      !(
        Math.abs(this.initialDownX - relativeX) > 5 ||
        Math.abs(this.initialDownY - relativeY) > 5
      )
    ) {
      return
    }

    const tRay = this.scene.createPickingRay(
      relativeX,
      relativeY,
      Matrix.Identity(),
      this.camera,
    )
    const pickResult: any = this.intersectRayPlane(tRay, this.plane)

    if (!pickResult) {
      return
    }

    this.positionData.canvasX = relativeX
    this.positionData.canvasY = relativeY
    this.positionData.mapX = pickResult.x
    this.positionData.mapY = pickResult.y
    this.mouseMoved = true

    emitGlobalEvent('pointerMove', this.positionData)
  }

  private handlePointerUp(e: IPointerEvent) {
    e.preventDefault()
    if (this.down === false) {
      return
    }
    this.down = false
    const pageX = e.pageX
    const pageY = e.pageY

    const relativeX = pageX - this.canvas.offsetLeft
    const relativeY = pageY - this.canvas.offsetTop
    const tRay = this.scene.createPickingRay(
      relativeX,
      relativeY,
      Matrix.Identity(),
      this.camera,
    )
    const pickResult: any = this.intersectRayPlane(tRay, this.plane)

    if (!pickResult) return

    this.positionData.canvasX = relativeX
    this.positionData.canvasY = relativeY
    this.positionData.mapX = pickResult.x
    this.positionData.mapY = pickResult.y
    this.positionData.mouseMoved = this.mouseMoved
    this.positionData.clickedItem = null
    emitGlobalEvent('pointerUp', this.positionData)
    if (!this.mouseMoved) {
      const hexagonalCoordinates = this.hexDimensions.getReferencePoint(e.x, e.y)
      emitGlobalEvent('pointerClick', hexagonalCoordinates)
    }
    this.mouseMoved = false
  }

  clear() {
    this.scene.dispose()
  }

  pan(dx: number, dy: number) {
    this.cameraTargetX = this.cameraTargetX + dx
    this.cameraTargetY = this.cameraTargetY + dy
    this.updatePosition()
  }

  tilt(dAlpha: number) {
    this.cameraAlpha = Math.min(
      Math.max(this.cameraAlpha + dAlpha, this.alphaMin),
      this.alphaMax,
    )
    this.updateCameraPosition()
  }

  spin(dBeta: number) {
    this.cameraBeta = this.cameraBeta + dBeta
    this.updateCameraPosition()
  }

  zoom(dRadius: number) {
    this.cameraRadius = this.cameraRadius + dRadius
    this.cameraRadius = Math.max(this.cameraRadius, this.radiusMin)
    this.updateCameraPosition()
  }

  centerOnCell(u: number, v: number) {
    const pixelCoordinates = this.hexDimensions.getPixelCoordinates(u, v)
    this.cameraTargetX = pixelCoordinates.x
    this.cameraTargetY = pixelCoordinates.y
    this.updatePosition()
  }

  updateCameraPosition() {
    const cpX =
      this.cameraTargetX +
      Math.sin(this.cameraBeta) * Math.cos(this.cameraAlpha) * this.cameraRadius
    const cpY =
      this.cameraTargetY +
      Math.cos(this.cameraBeta) * Math.cos(this.cameraAlpha) * this.cameraRadius
    const cpZ = Math.sin(this.cameraAlpha) * this.cameraRadius
    this.camera.setPosition(new Vector3(cpX, cpY, cpZ))
    this.positionData.cameraX = cpX
    this.positionData.cameraY = cpY
    this.positionData.cameraZ = cpZ
    emitGlobalEvent('updateCameraPosition', this.positionData)
  }

  updatePosition() {
    this.camera.target.x = this.cameraTargetX
    this.camera.target.y = this.cameraTargetY
    this.updateCameraPosition()
    this.positionData.middleX = this.cameraTargetX
    this.positionData.middleY = this.cameraTargetY
    emitGlobalEvent('updatePosition', this.positionData)
  }

  intersectRayPlane(pRay: Ray, pPlane: Plane) {
    let tIsecPoint: Nullable<Vector3> = null
    const tDot = Vector3.Dot(pRay.direction, pPlane.normal)
    if (tDot !== 0.0) {
      const t = -pPlane.signedDistanceTo(pRay.origin) / tDot
      if (t >= 0.0) {
        const tDirS = pRay.direction.scale(t)
        tIsecPoint = pRay.origin.add(tDirS)
      }
    }
    return tIsecPoint
  }

  init() {
    const handleWindowResize = () => {
      const displayWidth = this.canvas.clientWidth
      const displayHeight = this.canvas.clientHeight
      const dpr = window.devicePixelRatio || 1
      const ratio = dpr
      this.canvas.width = displayWidth * ratio
      this.canvas.height = displayHeight * ratio
      this.engine.resize()
      const hexagonalCoordinates = this.hexDimensions.getReferencePoint(
        this.cameraTargetX,
        this.cameraTargetY,
      )
      this.centerOnCell(hexagonalCoordinates.u, hexagonalCoordinates.v)
    }
    handleWindowResize()
    window.addEventListener('resize', handleWindowResize)
    new GridContext(this.hexDimensions, this, hex2color3('#808080'), 15, 15, 0.5)
    new StarryContext(this, 2000)
    new CameraControllingMouseListener(this)
    const light = new PointLight('light1', new Vector3(0, 0, 1), this.scene)
    light.intensity = 0.5
  }
}
