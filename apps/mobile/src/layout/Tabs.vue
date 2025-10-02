<template>
  <ion-page>
    <ion-tabs>
      <ion-router-outlet></ion-router-outlet>

      <ion-tab-bar slot="bottom">
        <ion-tab-button tab="home" href="/tabs/home">
          <ion-icon :icon="gameController" />
        </ion-tab-button>

        <ion-tab-button tab="map" href="/tabs/map">
          <ion-icon :icon="location" />
        </ion-tab-button>

        <ion-tab-button tab="settings" href="/tabs/settings">
          <ion-icon :icon="settingsOutline" />
        </ion-tab-button>
      </ion-tab-bar>
    </ion-tabs>
  </ion-page>
</template>

<script lang="ts">
  import {
    IonPage,
    IonTabs,
    IonRouterOutlet,
    IonTabBar,
    IonTabButton,
    IonLabel,
    IonIcon,
  } from '@ionic/vue'
  import { gameController, settingsOutline, location } from 'ionicons/icons'

  export default {
    components: {
      IonPage,
      IonTabs,
      IonRouterOutlet,
      IonTabBar,
      IonTabButton,
      IonLabel,
      IonIcon,
    },
    data() {
      return {
        location,
        gameController,
        settingsOutline,
      }
    },
  }
</script>
