import { Subject } from 'rxjs'

export const subjectsGlobal = new Map<string, Subject<any>>()

export const emitGlobalEvent = (eventName: string, data: any) => {
  subjectsGlobal.get(eventName)?.next(data)
}

export const onGlobalEvent = (eventName: string, handler: Fn) => {
  if (!subjectsGlobal.has(eventName)) subjectsGlobal.set(eventName, new Subject())
  const subject = subjectsGlobal.get(eventName)!
  const subscription = subject.subscribe(handler)
  return () => {
    subscription.unsubscribe()
    subjectsGlobal.delete(eventName)
  }
}

export const destroyGlobalEvent = () => {
  subjectsGlobal.forEach((subject) => subject.unsubscribe())
  subjectsGlobal.clear()
}
