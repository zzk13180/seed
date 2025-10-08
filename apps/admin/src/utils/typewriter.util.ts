/* eslint-disable @typescript-eslint/no-floating-promises */
import { Subject } from 'rxjs'
import * as diffUtils from './diff.util'
import { createControlledPromise } from './promise.util'
import { timeoutUtil } from './timeout.util'
import type { ControlledPromise } from './promise.util'

export enum ChatRole {
  AI = 'AI',
  USER = 'USER',
}

export interface ChatItem {
  id: string
  text: string
  role: ChatRole
}

export const typewriterSubject = new Subject<
  ChatItem & {
    step: AnimatorStep
  }
>()

export interface SnapshotOptions {
  wait?: number // 开始打字前等待时间
  paste?: boolean // 一次性粘贴全部内容,还是逐字输出；默认逐字输出
}

// 待打字段落的内容和配置
export interface Snapshot {
  content: ChatItem
  options?: SnapshotOptions
  isTypingEnd?: boolean // 打字机效果已经结束
  typingPromise?: ControlledPromise // 打字机效果的Promise
}

export class Snapshots extends Array<Snapshot> {
  private stopFlag = false
  constructor(...args: Snapshot[]) {
    super(...args)
  }

  get last() {
    return this.at(-1)
  }

  stopCurrentTypingEffect() {
    this.stopFlag = true
  }

  executeAllTypingEffectsImmediately() {
    timeoutUtil.immediatelyRun()
  }

  async pushSnapshot(snap: Snapshot) {
    this.stopFlag = false
    const prev = this.last
    snap.typingPromise = createControlledPromise()
    this.push(snap)
    if (prev?.content.id === snap.content.id && prev?.content.role === snap.content.role) {
      const { isTypingEnd } = prev
      if (isTypingEnd) {
        this.#typewriter(prev, snap)
      } else {
        await prev.typingPromise
        this.#typewriter(prev, snap)
      }
    } else {
      this.#typewriter(prev, snap)
    }
  }

  async #typewriter(prev: Snapshot | undefined, snap: Snapshot) {
    const charSteps = animateSteps(prev, snap)
    const sleepSteps = typingAnimator(charSteps)
    for await (const step of sleepSteps) {
      if (this.stopFlag) {
        snap.isTypingEnd = true
        snap.typingPromise?.resolve()
        break
      }
      const { snap: item, text } = step
      const content = item.content
      typewriterSubject.next({
        ...content,
        text,
        step,
      })
    }
    snap.isTypingEnd = true
    snap.typingPromise?.resolve()
  }
}

export enum AnimatorType {
  INSERT = 'insert', // 走打字机 一个字符一个字符的插入
  DELETE = 'delete', // 走打字机 删除某个index的单个字符
  PASTE = 'paste', // 不走打字机 一次性粘贴全部内容
  START = 'start', // 开始打字新的一段内容
}

export interface AnimatorStep {
  type: AnimatorType
  text: string
  snap: Snapshot
  index?: number // 删除/插入的位置（单个字符）
}

function* animateSteps(prev: Snapshot | undefined, snap: Snapshot): Generator<AnimatorStep> {
  if (prev === undefined || prev.content.id !== snap.content.id) {
    yield {
      type: AnimatorType.START,
      text: '',
      snap,
    }
  }
  const steps = patchSteps(prev, snap, snap.options?.paste)
  for (const step of steps) {
    yield step
  }
}

function* patchSteps(
  prev: Snapshot | undefined,
  current: Snapshot,
  isPasted?: boolean,
): Generator<AnimatorStep> {
  const { content: prevContent } = prev || {}
  const { content: currentContent } = current

  if (isPasted) {
    yield {
      type: AnimatorType.PASTE,
      snap: current,
      text: currentContent.text,
    }
    return
  }

  const patches = diffUtils.diff(prevContent, currentContent)
  let index = 0
  for (const [type, text] of patches) {
    // 0: equal, 1: insert, -1: delete
    if (type === 0) {
      index += text.length
    }
    if (type === -1) {
      index += text.length - 1
      for (let i = text.length - 1; i >= 0; i--) {
        yield {
          type: AnimatorType.DELETE,
          snap: current,
          text: text[i],
          index,
        }
        index--
      }
      index += 1 // 修正 index 为下一个字符的位置
    }
    if (type === 1) {
      const charArray = text.split('')
      for (const char of charArray) {
        yield {
          type: AnimatorType.INSERT,
          snap: current,
          text: char,
          index,
        }
        index++
      }
    }
  }
}

async function* typingAnimator(steps: Generator<AnimatorStep>): AsyncGenerator<AnimatorStep> {
  const getOptions = (snap?: Snapshot) => ({
    ...snap?.options,
  })
  const actions: Partial<Record<AnimatorType, (step: AnimatorStep) => Promise<void>>> = {
    [AnimatorType.START]: async (step: AnimatorStep) => {
      const { wait } = getOptions(step.snap)
      if (wait) {
        await sleep(wait)
      }
    },
    [AnimatorType.INSERT]: async () => {
      await sleep(getTimeout(1)) // 1倍速
    },
    [AnimatorType.DELETE]: async () => {
      await sleep(getTimeout(0.7))
    },
  }

  for (const step of steps) {
    const action = actions[step.type]
    if (action) {
      await action(step)
    }
    yield step
  }
}

function sleep(ms: number) {
  return new Promise(resolve => timeoutUtil.setTimeout(resolve, ms))
}

function randRange(min: number, max: number): number {
  return Math.random() * (max - min) + min
}

function getTimeout(multiplier = 1): number {
  return randRange(50, 230) * multiplier
}
