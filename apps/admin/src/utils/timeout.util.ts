/* eslint-disable consistent-return */
/* eslint-disable @typescript-eslint/no-unsafe-function-type */
interface Execution {
  fn: Function // () => void
  ms: number
  startFrameTimestamp?: DOMHighResTimeStamp
}

// 基于 requestAnimationFrame 实现的 setTimeout
// 1. 更准确的定时：避免 setTimeout 过多时出现的任务堆积，导致定时任务执行时间变长。
// 2. 可以实现自定义功能：例如，可以立即同步执行所有定时任务。
// 3. 更低的CPU使用率：当页面不可见或最小化时，requestAnimationFrame会暂停，从而减少 CPU 的使用。
// 4. 避免不必要的布局和重绘，减少闪烁和跳帧：requestAnimationFrame的回调函数在浏览器下一次重绘之前执行。
class TimeoutUtil {
  private cancelId: number | undefined = undefined
  private fnId = 0
  private immediately = false
  private readonly executionSet = new Set<Execution>()
  private readonly fnMap = new Map<number, Execution>()

  // 约定 fn 执行时间不会太长
  setTimeout(fn: Function, ms: number) {
    if (this.immediately) {
      fn()
      return
    }
    const execution = { fn, ms }
    return this.addFn(execution)
  }

  clearTimeout(id: number): boolean {
    if (this.fnMap.has(id)) {
      this.fnMap.delete(id)
      return true
    }
    return false
  }

  async immediatelyRun() {
    if (this.cancelId) {
      globalThis.cancelAnimationFrame(this.cancelId)
      this.cancelId = undefined
    }
    this.immediately = true
    this.runFnMap() // 立即执行，还没到执行时间的函数
    await new Promise(resolve => setTimeout(resolve, 50))
    this.immediately = false
  }

  private addFn({ fn, ms }: Execution): number {
    const currentId = this.fnId
    this.fnMap.set(currentId, {
      fn,
      ms,
    })
    this.cancelId = globalThis.requestAnimationFrame(this.loop.bind(this))
    // 实际中很难达到这个值
    if (this.fnId === Number.MAX_SAFE_INTEGER) {
      this.fnId = 0
    } else {
      this.fnId++
    }
    return currentId
  }

  private loop(timestamp: DOMHighResTimeStamp) {
    if (this.fnMap.size === 0) {
      return
    }
    this.fnMap.forEach(this.checkTick(timestamp))
    this.runExecutionSet()
    if (this.fnMap.size > 0) {
      this.cancelId = globalThis.requestAnimationFrame(this.loop.bind(this))
    }
  }

  private checkTick(currentTimeTick: number) {
    return (value: Execution, id: number) => {
      if (value.startFrameTimestamp === undefined) {
        value.startFrameTimestamp = currentTimeTick
      }
      const elapsed = currentTimeTick - value.startFrameTimestamp
      if (elapsed > value.ms) {
        this.executionSet.add(value)
        this.fnMap.delete(id)
      }
    }
  }

  private runExecutionSet() {
    if (this.executionSet.size === 0) {
      return
    }
    const copy = new Set(this.executionSet)
    this.executionSet.clear()
    for (const value of copy) {
      try {
        value.fn()
      } catch (error) {
        console.error(error)
      }
    }
  }

  private runFnMap() {
    if (this.fnMap.size === 0) {
      return
    }
    const copy = new Map(this.fnMap)
    this.fnMap.clear()
    for (const [, execution] of copy) {
      try {
        execution.fn()
      } catch (error) {
        console.error(error)
      }
    }
  }
}

export const timeoutUtil = new TimeoutUtil()
