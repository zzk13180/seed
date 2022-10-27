<template>
  <IxDrawerProvider ref="drawerProviderRef">
    <IxNotificationProvider>
      <IxModalProvider ref="modalProviderRef">
        <IxMessageProvider>
          <IduxProviderRegister />
          <slot></slot>
        </IxMessageProvider>
      </IxModalProvider>
    </IxNotificationProvider>
  </IxDrawerProvider>
</template>

<script setup lang="ts">
  import { onMounted, ref } from 'vue'
  import { useRouter } from 'vue-router'
  import IduxProviderRegister from './IduxProviderRegister.vue'
  import type { DrawerProviderInstance } from '@idux/components/drawer'
  import type { ModalProviderInstance } from '@idux/components/modal'

  const drawerProviderRef = ref<DrawerProviderInstance>()
  const modalProviderRef = ref<ModalProviderInstance>()

  const router = useRouter()

  onMounted(() => {
    // 每次路由切换时销毁当前的抽屉和弹窗（仅对通过 useDrawer/useModal 创建的生效）
    router.afterEach(() => {
      drawerProviderRef.value!.destroyAll()
      modalProviderRef.value!.destroyAll()
    })
  })
</script>
