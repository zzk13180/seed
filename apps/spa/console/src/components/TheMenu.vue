<template>
  <ion-menu content-id="main-content">
    <ion-content class="ion-padding">
      <ion-list lines="none">
        <ion-item>
          <ion-icon slot="start" :icon="moonOutline"></ion-icon>
          <ion-toggle v-model="localDark" label-placement="start">深色模式</ion-toggle>
        </ion-item>
      </ion-list>
    </ion-content>
  </ion-menu>
</template>

<script lang="ts">
  import { defineComponent, onMounted, ref, watch } from 'vue'

  import { IonContent, IonIcon, IonItem, IonList, IonMenu, IonToggle } from '@ionic/vue'
  import { moonOutline } from 'ionicons/icons'
  import { Storage } from '@ionic/storage'
  import router from '@/router'

  export default defineComponent({
    name: 'TheMenu',
    components: {
      IonContent,
      IonIcon,
      IonItem,
      IonList,
      IonMenu,
      IonToggle,
    },
    setup() {
      const loggedIn = ref(false)
      const storage = new Storage()

      const localDark = ref(false)
      watch(localDark, newVal => {
        document.documentElement.classList.toggle('ion-palette-dark', newVal)
      })

      const navigate = (url: string) => {
        router.push(url)
      }

      onMounted(async () => {
        await storage.create()
      })

      return {
        localDark,
        navigate,
        loggedIn,
      }
    },
    data() {
      return {
        moonOutline,
      }
    },
  })
</script>
