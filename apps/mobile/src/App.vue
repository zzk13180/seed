<template>
  <ion-app>
    <ion-split-pane content-id="main-content">
      <ion-router-outlet id="main-content" />
      <TheMenu />
    </ion-split-pane>
  </ion-app>
</template>

<script lang="ts">
  import { IonApp, IonSplitPane, IonRouterOutlet } from '@ionic/vue'
  import { Ref, defineComponent, ref, onMounted } from 'vue'
  import { Storage } from '@ionic/storage'
  import TheMenu from './components/TheMenu.vue'
  import { useAppStore } from './stores/app.store'

  export default defineComponent({
    name: 'App',
    components: {
      IonApp,
      IonSplitPane,
      IonRouterOutlet,
      TheMenu,
    },
    setup() {
      const { state, controller } = useAppStore()
      const dark: Ref<boolean> = ref(false)

      const handleDarkModeChanged = (newDarkValue: boolean) => {
        dark.value = newDarkValue
        document.documentElement.classList.toggle('ion-palette-dark', newDarkValue)
      }

      onMounted(() => {
        const storage = new Storage()
        storage.create()
        controller.initialize()
      })

      return {
        dark,
        handleDarkModeChanged,
      }
    },
  })
</script>

<style lang="scss" scoped>
  ion-split-pane {
    --side-max-width: 270px;
  }
</style>
