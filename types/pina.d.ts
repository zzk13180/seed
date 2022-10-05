import 'pinia'

declare module 'pinia' {
  export interface DefineStoreOptionsInPlugin<_S, Store> {
    debounce?: Partial<Record<keyof StoreActions<Store>, number>>
  }
}
