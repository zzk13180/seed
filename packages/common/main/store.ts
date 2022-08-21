import { createPinia } from 'pinia'
import { debounce } from 'lodash'
import piniaPersist from 'pinia-plugin-persist'

const store = createPinia()

store.use(piniaPersist)

store.use(({ options, store }) => {
  if (options.debounce) {
    return Object.keys(options.debounce).reduce((debouncedActions, action) => {
      debouncedActions[action] = debounce(store[action], options.debounce[action])
      return debouncedActions
    }, {})
  }
})

export { store }
