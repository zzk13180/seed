<template>
  <ion-page ref="page">
    <ion-header :translucent="true">
      <ion-toolbar>
        <ion-buttons slot="start">
          <ion-menu-button></ion-menu-button>
        </ion-buttons>

        <ion-buttons slot="end">
          <ion-button fill="clear" :color="state.rosBridgeConnected ? 'success' : 'danger'">
            {{ state.rosBridgeConnected ? 'ROS 已连接' : 'ROS 未连接' }}
          </ion-button>
        </ion-buttons>

        <ion-title>Title</ion-title>
      </ion-toolbar>

      <ion-toolbar>
        <ion-segment :value="segment" @ionChange="updateSegment">
          <ion-segment-button value="navigation">navigation</ion-segment-button>
          <ion-segment-button value="charge">charge</ion-segment-button>
        </ion-segment>
      </ion-toolbar>
    </ion-header>

    <ion-content :fullscreen="false">
      <ion-header collapse="condense">
        <ion-toolbar>
          <ion-title size="large">Title</ion-title>
        </ion-toolbar>
      </ion-header>

      <ion-list>
        <!-- List -->
      </ion-list>

      <ion-fab slot="fixed" vertical="bottom" horizontal="end" ref="fab">
        <ion-fab-button>
          <ion-icon :icon="stopCircleOutline" color="danger"></ion-icon>
        </ion-fab-button>
      </ion-fab>

      <ion-toast
        position="bottom"
        :is-open="!!state.toastMessage"
        :message="state.toastMessage"
        :duration="2000"
        @didDismiss="state.toastMessage = ''"
      ></ion-toast>
    </ion-content>
  </ion-page>
</template>

<script setup lang="ts">
  import { ref, onMounted } from 'vue'
  import {
    IonPage,
    IonHeader,
    IonTitle,
    IonToolbar,
    IonButtons,
    IonMenuButton,
    IonSegment,
    IonSegmentButton,
    IonButton,
    IonContent,
    IonList,
    IonFab,
    IonFabButton,
    IonIcon,
    IonToast,
  } from '@ionic/vue'
  import { stopCircleOutline } from 'ionicons/icons'
  import { useAppStore } from '../stores/app.store'

  const segment = ref('navigation') // charge

  const page = ref<InstanceType<typeof IonPage> | null>(null)

  const { state } = useAppStore()

  const updateSegment = (e: any) => {
    segment.value = e.detail.value
  }

  onMounted(() => {})
</script>

<style scoped>
  ion-fab-button {
    --background: var(--ion-color-step-150, #fff);
    --background-hover: var(--ion-color-step-200, #f2f2f2);
    --background-focused: var(--ion-color-step-250, #d9d9d9);
    --color: var(--ion-color-primary, #3880ff);
  }

  /*
   * Material Design uses the ripple for activated
   * so only style the iOS activated background
   */
  .ios ion-fab-button {
    --background-activated: var(--ion-color-step-250, #d9d9d9);
  }

  ion-item-sliding[data-color='ionic'] ion-label {
    padding-left: 10px;
    border-left: 2px solid var(--ion-color-ionic);
  }

  ion-item-sliding[data-color='vue'] ion-label {
    padding-left: 10px;
    border-left: 2px solid var(--ion-color-vue);
  }

  ion-item-sliding[data-color='communication'] ion-label {
    padding-left: 10px;
    border-left: 2px solid var(--ion-color-communication);
  }

  ion-item-sliding[data-color='tooling'] ion-label {
    padding-left: 10px;
    border-left: 2px solid var(--ion-color-tooling);
  }

  ion-item-sliding[data-color='services'] ion-label {
    padding-left: 10px;
    border-left: 2px solid var(--ion-color-services);
  }

  ion-item-sliding[data-color='design'] ion-label {
    padding-left: 10px;
    border-left: 2px solid var(--ion-color-design);
  }

  ion-item-sliding[data-color='workshop'] ion-label {
    padding-left: 10px;
    border-left: 2px solid var(--ion-color-workshop);
  }

  ion-item-sliding[data-color='food'] ion-label {
    padding-left: 10px;
    border-left: 2px solid var(--ion-color-food);
  }

  ion-item-sliding[data-color='documentation'] ion-label {
    padding-left: 10px;
    border-left: 2px solid var(--ion-color-documentation);
  }

  ion-item-sliding[data-color='navigation'] ion-label {
    padding-left: 10px;
    border-left: 2px solid var(--ion-color-navigation);
  }
</style>
