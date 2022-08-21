import '@idux/cdk/index.css'
import '@idux/components/style/core/reset.default.css'
import '@idux/components/style/core/reset-scroll.default.css'
import '@idux/components/default.css'
import '@idux/pro/default.css'
import IduxCdk from '@idux/cdk'
import IduxComponents from '@idux/components'
import IduxPro from '@idux/pro'
import { createGlobalConfig } from '@idux/components/config'
import { IDUX_ICON_DEPENDENCIES, addIconDefinitions } from '@idux/components/icon'
import IduxProvider from './src/IduxProvider.vue'
import type { DrawerProviderRef } from '@idux/components/drawer'
import type { NotificationProviderRef } from '@idux/components/notification'
import type { ModalProviderRef } from '@idux/components/modal'
import type { MessageProviderRef } from '@idux/components/message'
import type { App } from 'vue'

addIconDefinitions(IDUX_ICON_DEPENDENCIES)

const loadIconDynamically = (iconName: string) => {
  return fetch(`/idux-icons/${iconName}.svg`).then((res) => res.text())
}

const globalConfig = createGlobalConfig({
  // import { enUS } from "@idux/components/locales"
  // locale: enUS,
  icon: { loadIconDynamically },
})

let Drawer: DrawerProviderRef | undefined
let Notification: NotificationProviderRef | undefined
let Modal: ModalProviderRef | undefined
let Message: MessageProviderRef | undefined

function registerProviders(option: {
  drawer: DrawerProviderRef
  notification: NotificationProviderRef
  modal: ModalProviderRef
  message: MessageProviderRef
}): void {
  Drawer = option.drawer
  Notification = option.notification
  Modal = option.modal
  Message = option.message
}

const idux = (app: App): void => {
  app.use(IduxCdk).use(IduxComponents).use(IduxPro).use(globalConfig)
}

export { registerProviders, idux, Drawer, Notification, Modal, Message, IduxProvider }
