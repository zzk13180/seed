import { reactive } from 'vue'
import type { Item } from './types'

interface Data {
  added: Item[]
  removed: Item[]
  updated: Item[]
}

export class DataSource {
  data: Data = reactive({ added: [], removed: [], updated: [] })

  constructor() {}

  addItems(items: Item[]) {
    this.data.added = items
  }

  removeItems(items: Item[]) {
    this.data.removed = items
  }

  updateItems(items: Item[]) {
    this.data.updated = items
  }

  init() {
    this.addItems([
      {
        id: 'sun',
        type: 'star',
        size: 100,
        lineWidth: 5,
        lineColor: '#f97306',
        backgroundColor: '#ffff14',
        u: 0,
        v: 0,
      },
    ])
    this.addItems([
      {
        id: 'earth',
        type: 'planet',
        size: 66,
        lineWidth: 5,
        lineColor: '#653700',
        backgroundColor: '#0343df',
        borderColor: '#ffffff',
        u: 5,
        v: 5,
      },
    ])

    this.addItems([
      {
        id: 'moon',
        type: 'moon',
        size: 33,
        lineWidth: 2.5,
        lineColor: '#929591',
        backgroundColor: '#e1e1d6',
        borderWidth: 3,
        borderColor: 'black',
        u: 3,
        v: 8,
      },
    ])

    this.addItems([
      {
        id: 'sa1',
        type: 'arrow',
        u: 0,
        v: -1,
        fillColor: '#929591',
        lineWidth: 3,
        lineColor: '#929591',
        rotation: 180,
        scaleLength: 0.75,
        scaleWidth: 0.75,
      },
    ])
    this.addItems([
      {
        id: 'sa2',
        type: 'arrow',
        u: -1,
        v: 0,
        fillColor: '#929591',
        lineWidth: 3,
        lineColor: '#929591',
        rotation: 240,
        scaleLength: 0.75,
        scaleWidth: 0.75,
      },
    ])
    this.addItems([
      {
        id: 'sa3',
        type: 'arrow',
        u: -1,
        v: 1,
        fillColor: '#929591',
        lineWidth: 3,
        lineColor: '#929591',
        rotation: 300,
        scaleLength: 0.75,
        scaleWidth: 0.75,
      },
    ])
    this.addItems([
      {
        id: 'sa4',
        type: 'arrow',
        u: 0,
        v: 1,
        fillColor: '#929591',
        lineWidth: 3,
        lineColor: '#929591',
        rotation: 0,
        scaleLength: 0.75,
        scaleWidth: 0.75,
      },
    ])
    this.addItems([
      {
        id: 'sa5',
        type: 'arrow',
        u: 1,
        v: 0,
        fillColor: '#929591',
        lineWidth: 3,
        lineColor: '#929591',
        rotation: 60,
        scaleLength: 0.75,
        scaleWidth: 0.75,
      },
    ])
    this.addItems([
      {
        id: 'sa6',
        type: 'arrow',
        u: 1,
        v: -1,
        fillColor: '#929591',
        lineWidth: 3,
        lineColor: '#929591',
        rotation: 120,
        scaleLength: 0.75,
        scaleWidth: 0.75,
      },
    ])
    this.addItems([
      {
        id: 'ea1',
        type: 'arrow',
        u: 5,
        v: 4,
        fillColor: '#929591',
        lineWidth: 3,
        lineColor: '#929591',
        rotation: 180,
        scaleLength: 0.75,
        scaleWidth: 0.75,
      },
    ])
    this.addItems([
      {
        id: 'ea2',
        type: 'arrow',
        u: 4,
        v: 5,
        fillColor: '#929591',
        lineWidth: 3,
        lineColor: '#929591',
        rotation: 240,
        scaleLength: 0.75,
        scaleWidth: 0.75,
      },
    ])
    this.addItems([
      {
        id: 'ea3',
        type: 'arrow',
        u: 4,
        v: 6,
        fillColor: '#929591',
        lineWidth: 3,
        lineColor: '#929591',
        rotation: 300,
        scaleLength: 0.75,
        scaleWidth: 0.75,
      },
    ])
    this.addItems([
      {
        id: 'ea4',
        type: 'arrow',
        u: 5,
        v: 6,
        fillColor: '#929591',
        lineWidth: 3,
        lineColor: '#929591',
        rotation: 0,
        scaleLength: 0.75,
        scaleWidth: 0.75,
      },
    ])
    this.addItems([
      {
        id: 'ea5',
        type: 'arrow',
        u: 6,
        v: 5,
        fillColor: '#929591',
        lineWidth: 3,
        lineColor: '#929591',
        rotation: 60,
        scaleLength: 0.75,
        scaleWidth: 0.75,
      },
    ])
    this.addItems([
      {
        id: 'ea6',
        type: 'arrow',
        u: 6,
        v: 4,
        fillColor: '#929591',
        lineWidth: 3,
        lineColor: '#929591',
        rotation: 120,
        scaleLength: 0.75,
        scaleWidth: 0.75,
      },
    ])

    this.addItems([
      {
        id: 'ma1',
        type: 'arrow',
        u: 3,
        v: 7,
        lineWidth: 3,
        lineColor: '#929591',
        rotation: 180,
        scaleLength: 0.75,
        scaleWidth: 0.75,
      },
    ])
    this.addItems([
      {
        id: 'ma2',
        type: 'arrow',
        u: 2,
        v: 8,
        lineWidth: 3,
        lineColor: '#929591',
        rotation: 240,
        scaleLength: 0.75,
        scaleWidth: 0.75,
      },
    ])
    this.addItems([
      {
        id: 'ma3',
        type: 'arrow',
        u: 2,
        v: 9,
        lineWidth: 3,
        lineColor: '#929591',
        rotation: 300,
        scaleLength: 0.75,
        scaleWidth: 0.75,
      },
    ])
    this.addItems([
      {
        id: 'ma4',
        type: 'arrow',
        u: 3,
        v: 9,
        lineWidth: 3,
        lineColor: '#929591',
        rotation: 0,
        scaleLength: 0.75,
        scaleWidth: 0.75,
      },
    ])
    this.addItems([
      {
        id: 'ma5',
        type: 'arrow',
        u: 4,
        v: 8,
        lineWidth: 3,
        lineColor: '#929591',
        rotation: 60,
        scaleLength: 0.75,
        scaleWidth: 0.75,
      },
    ])
    this.addItems([
      {
        id: 'ma6',
        type: 'arrow',
        u: 4,
        v: 7,
        lineWidth: 3,
        lineColor: '#929591',
        rotation: 120,
        scaleLength: 0.75,
        scaleWidth: 0.75,
      },
    ])
    this.addItems([
      {
        id: 'asteroids1',
        type: 'asteroids',
        u: -1,
        v: 10,
      },
      {
        id: 'asteroids2',
        type: 'asteroids',
        u: -2,
        v: 10,
      },
      {
        id: 'asteroids3',
        type: 'asteroids',
        u: -3,
        v: 10,
      },
    ])
    this.addItems([
      {
        id: 'asteroids4',
        type: 'asteroids',
        u: -3,
        v: 11,
      },
      {
        id: 'asteroids5',
        type: 'asteroids',
        u: -2,
        v: 11,
      },
      {
        id: 'asteroids6',
        type: 'asteroids',
        u: -2,
        v: 10,
      },
    ])
    this.addItems([
      {
        id: 'asteroids7',
        type: 'asteroids',
        u: -1,
        v: 9,
      },
      {
        id: 'asteroids8',
        type: 'asteroids',
        u: -2,
        v: 9,
      },
    ])
    this.addItems([
      {
        id: 'station',
        type: 'polygon',
        diameter: 40,
        thickness: 5,
        sides: 5,
        color: '#0343df',
        u: 6,
        v: 5,
      },
    ])
    this.addItems([
      {
        id: 'ship',
        type: 'ship',
        size: 50,
        u: 6,
        v: 5,
        angle: Math.PI,
        img: './test.svg',
        isEmissive: true,
        vectors: [
          {
            id: 'vector1',
            type: 'vector',
            size: 50,
            u: 0,
            v: 1,
            vectorU: 1,
            vectorV: 0,
            lineColor: '#0343df',
            lineWidth: 10,
            isEmissive: true,
          },
        ],
      },
    ])
    this.addItems([
      {
        id: 'vertShip',
        type: 'ship',
        size: 50,
        u: 6,
        v: 5,
        img: './test.svg',
        isEmissive: true,
        vertical: true,
      },
    ])
    this.addItems([
      {
        id: 'vertShip2',
        type: 'ship',
        size: 50,
        u: 6,
        v: 5,
        img: './test.svg',
        isEmissive: true,
        vertical: true,
        vectors: [
          {
            id: 'vector1',
            type: 'vector',
            size: 50,
            u: 0,
            v: 1,
            vectorU: 1,
            vectorV: 0,
            lineColor: '#0343df',
            lineWidth: 10,
            isEmissive: true,
          },
        ],
      },
    ])
    this.addItems([
      {
        id: 'vector1',
        type: 'vector',
        size: 50,
        u: 0,
        v: 1,
        vectorU: 1,
        vectorV: 0,
        lineColor: '#0343df',
        lineWidth: 10,
        isEmissive: true,
      },
    ])
    this.addItems([
      {
        id: 'gs1',
        type: 'polygon',
        diameter: 40,
        thickness: 5,
        sides: 3,
        color: '#15b01a',
        u: 1,
        v: 0,
      },
      {
        id: 'rs1',
        type: 'polygon',
        diameter: 40,
        thickness: 5,
        sides: 3,
        color: '#e50000',
        u: 2,
        v: 9,
      },
    ])
    this.addItems([
      {
        id: 'gs2',
        type: 'polygon',
        diameter: 40,
        thickness: 5,
        sides: 3,
        color: '#15b01a',
        u: 1,
        v: 0,
      },
      {
        id: 'rs2',
        type: 'polygon',
        diameter: 40,
        thickness: 5,
        sides: 3,
        color: '#e50000',
        u: 2,
        v: 9,
      },
    ])
    this.addItems([
      {
        id: 'gs3',
        type: 'polygon',
        diameter: 40,
        thickness: 5,
        sides: 3,
        color: '#15b01a',
        u: 1,
        v: 0,
      },
      {
        id: 'rs3',
        type: 'polygon',
        diameter: 40,
        thickness: 5,
        sides: 3,
        color: '#e50000',
        u: 2,
        v: 9,
      },
    ])
    this.addItems([
      {
        id: 'gs4',
        type: 'polygon',
        diameter: 40,
        thickness: 5,
        sides: 3,
        color: '#15b01a',
        u: 1,
        v: 0,
      },
      {
        id: 'rs4',
        type: 'polygon',
        diameter: 40,
        thickness: 5,
        sides: 3,
        color: '#e50000',
        u: 2,
        v: 9,
      },
    ])
    this.addItems([
      {
        id: 'gs5',
        type: 'polygon',
        diameter: 40,
        thickness: 5,
        sides: 3,
        color: '#15b01a',
        u: 1,
        v: 0,
      },
      {
        id: 'rs5',
        type: 'polygon',
        diameter: 40,
        thickness: 5,
        sides: 3,
        color: '#e50000',
        u: 2,
        v: 9,
      },
    ])
    this.addItems([
      {
        id: 'gs6',
        type: 'polygon',
        diameter: 40,
        thickness: 5,
        sides: 3,
        color: '#15b01a',
        u: 1,
        v: 0,
      },
      {
        id: 'rs6',
        type: 'polygon',
        diameter: 40,
        thickness: 5,
        sides: 3,
        color: '#e50000',
        u: 2,
        v: 9,
      },
    ])
    this.addItems([
      {
        id: 'gs7',
        type: 'polygon',
        diameter: 40,
        thickness: 5,
        sides: 3,
        color: '#15b01a',
        u: 1,
        v: 0,
      },
      {
        id: 'rs7',
        type: 'polygon',
        diameter: 40,
        thickness: 5,
        sides: 3,
        color: '#e50000',
        u: 2,
        v: 9,
      },
    ])
    this.addItems([
      {
        id: 'gs8',
        type: 'polygon',
        diameter: 40,
        thickness: 5,
        sides: 3,
        color: '#15b01a',
        u: 1,
        v: 0,
      },
      {
        id: 'rs8',
        type: 'polygon',
        diameter: 40,
        thickness: 5,
        sides: 3,
        color: '#e50000',
        u: 2,
        v: 9,
      },
    ])
    this.addItems([
      {
        id: 'gs9',
        type: 'polygon',
        diameter: 40,
        thickness: 5,
        sides: 3,
        color: '#15b01a',
        u: 1,
        v: 0,
      },
      {
        id: 'rs9',
        type: 'polygon',
        diameter: 40,
        thickness: 5,
        sides: 3,
        color: '#e50000',
        u: 2,
        v: 9,
      },
    ])
    this.addItems([
      {
        id: 'gs10',
        type: 'polygon',
        diameter: 40,
        thickness: 5,
        sides: 3,
        color: '#15b01a',
        u: 1,
        v: 0,
      },
      {
        id: 'rs10',
        type: 'polygon',
        diameter: 40,
        thickness: 5,
        sides: 3,
        color: '#e50000',
        u: 2,
        v: 9,
      },
    ])
    this.addItems([
      {
        id: 'gs11',
        type: 'polygon',
        diameter: 40,
        thickness: 5,
        sides: 3,
        color: '#15b01a',
        u: 1,
        v: 0,
      },
      {
        id: 'rs11',
        type: 'polygon',
        diameter: 40,
        thickness: 5,
        sides: 3,
        color: '#e50000',
        u: 2,
        v: 9,
      },
    ])
  }
}
