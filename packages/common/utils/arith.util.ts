interface IFloatArith {
  toFixed(num: number, precision: number): number
  mul(num1: number, num2: number, ...others: number[]): number
  plus(num1: number, num2: number, ...others: number[]): number
  sub(num1: number, num2: number, ...others: number[]): number
  div(num1: number, num2: number, ...others: number[]): number
}

class FloatArith implements IFloatArith {
  private static instance: FloatArith
  private constructor() {}

  public static getInstance(): FloatArith {
    if (!FloatArith.instance) {
      FloatArith.instance = new FloatArith()
    }
    return FloatArith.instance
  }

  public toFixed(num: number, precision: number): number {
    if (!precision) {
      return Math.round(num)
    }
    const tmp = Math.abs(num)
    let pair = `${tmp}e`.split('e')
    const value = Math.round(+`${pair[0]}e${+pair[1] + precision}`)
    pair = `${value}e`.split('e')
    const result = +`${pair[0]}e${+pair[1] - precision}`
    return num < 0 ? -result : result
  }

  public mul(num1: number, num2: number, ...others: number[]): number {
    if (others.length) {
      return this.mul(this.mul(num1, num2), others[0], ...others.slice(1))
    }
    if (this.isExponentialNotation(num1, num2)) {
      return num1 * num2
    }
    const num1Changed = this.f2i(num1)
    const num2Changed = this.f2i(num2)
    const baseNum = this.getLength(num1) + this.getLength(num2)
    const leftValue = num1Changed * num2Changed
    return leftValue / 10 ** baseNum
  }

  public plus(num1: number, num2: number, ...others: number[]): number {
    if (others.length) {
      return this.plus(this.plus(num1, num2), others[0], ...others.slice(1))
    }
    if (this.isExponentialNotation(num1, num2)) {
      return num1 + num2
    }
    const baseNum = 10 ** Math.max(this.getLength(num1), this.getLength(num2))
    return (this.mul(num1, baseNum) + this.mul(num2, baseNum)) / baseNum
  }

  public sub(num1: number, num2: number, ...others: number[]): number {
    if (others.length) {
      return this.sub(this.sub(num1, num2), others[0], ...others.slice(1))
    }
    if (this.isExponentialNotation(num1, num2)) {
      return num1 - num2
    }
    const baseNum = 10 ** Math.max(this.getLength(num1), this.getLength(num2))
    return (this.mul(num1, baseNum) - this.mul(num2, baseNum)) / baseNum
  }

  public div(num1: number, num2: number, ...others: number[]): number {
    if (others.length) {
      return this.div(this.div(num1, num2), others[0], ...others.slice(1))
    }
    if (this.isExponentialNotation(num1, num2)) {
      return num1 / num2
    }
    const num1Changed = this.f2i(num1)
    const num2Changed = this.f2i(num2)
    return this.mul(
      num1Changed / num2Changed,
      +(10 ** (this.getLength(num2) - this.getLength(num1))).toPrecision(15),
    )
  }

  private getLength(num: number): number {
    return (String(num).split('.')[1] || '').length
  }

  private f2i(num: number): number {
    return +String(num).replace('.', '')
  }

  // var fn = function (a, b) { console.log(/[eE]/.test(String(a)))console.log(/[eE]/.test(String(b)))return a * b}
  // fn(2e-1,2e-1) 输出 false false 0.04000000000000001
  // fn(0.000000123456,0.000000123456) 输出 true true 1.5241383936e-14
  private isExponentialNotation(num1: number, num2: number): boolean {
    return /[eE]/.test(String(num1)) || /[eE]/.test(String(num2))
  }
}

const arith = FloatArith.getInstance()

export { arith }
