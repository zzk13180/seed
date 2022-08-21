export class Hexagonal {
  edgeSize: number
  h: number
  r: number
  twiddle: number
  hexagon_half_wide_width: number
  hexagon_wide_width: number
  hexagon_edge_to_edge_width: number
  hexagon_scaled_half_edge_size: number
  hexagon_narrow_width: number

  constructor(edgeSize: number, private readonly vScale: number) {
    this.edgeSize = edgeSize
    this.h = Math.sin((30 * Math.PI) / 180) * this.edgeSize
    this.r = Math.cos((30 * Math.PI) / 180) * this.edgeSize
    this.twiddle = this.edgeSize % 2 ? 0.5 : 0
    this.hexagon_half_wide_width = Math.round(this.vScale * (this.edgeSize / 2 + this.h))
    this.hexagon_wide_width = 2 * this.hexagon_half_wide_width
    this.hexagon_edge_to_edge_width = 2 * Math.round(this.r)
    this.hexagon_scaled_half_edge_size = Math.round(this.vScale * (this.edgeSize / 2))
    this.hexagon_narrow_width =
      this.hexagon_half_wide_width + this.hexagon_scaled_half_edge_size
  }

  getPixelCoordinates(u: number, v: number) {
    const y = this.hexagon_narrow_width * u + this.twiddle
    const x = this.hexagon_edge_to_edge_width * (u * 0.5 + v) + this.twiddle
    return { x, y }
  }

  getReferencePoint(x: number, y: number) {
    const u = Math.round(y / this.hexagon_narrow_width)
    const v = Math.round(x / this.hexagon_edge_to_edge_width - u * 0.5)
    return { u, v }
  }
}
