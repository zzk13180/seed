import type { Ref } from 'vue'

export type CollectionItem<T = Record<string, any>> = {
  ref: HTMLElement | null
} & T

export type CollectionInjectionContext = {
  itemMap: Map<HTMLElement, CollectionItem>
  getItems: <T>() => CollectionItem<T>[]
  collectionRef: Ref<HTMLElement | null>
}

export type CollectionItemInjectionContext = {
  collectionItemRef: Ref<HTMLElement | null>
}
