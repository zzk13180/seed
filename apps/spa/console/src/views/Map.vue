<template>
  <ion-page>
    <ion-header :translucent="true">
      <ion-toolbar>
        <ion-title>地图</ion-title>
        <ion-buttons slot="start">
          <ion-menu-button></ion-menu-button>
        </ion-buttons>
        <ion-buttons slot="end">
          <ion-button fill="clear" :color="state.rosBridgeConnected ? 'success' : 'danger'">
            {{ state.rosBridgeConnected ? 'ROS 已连接' : 'ROS 未连接' }}
          </ion-button>
        </ion-buttons>
      </ion-toolbar>
    </ion-header>

    <ion-content :fullscreen="true" class="ion-padding">
      <!-- content -->
    </ion-content>
  </ion-page>
</template>

<script lang="ts" setup>
  import { onMounted } from 'vue'
  import {
    IonPage,
    IonHeader,
    IonToolbar,
    IonTitle,
    IonContent,
    IonButton,
    IonButtons,
    IonMenuButton,
  } from '@ionic/vue'
  import { useAppStore } from '../stores/app.store'

  const { state } = useAppStore()

  onMounted(() => {})
</script>

<style scoped>
  .controller-container {
    position: absolute;
    width: calc(100% - 2rem);
    height: calc(100% - 2rem);
    top: 0;
    left: 0;
  }

  ion-fab[vertical='bottom'][horizontal='start'] {
    padding: 12px;
  }

  .joystick-modal {
    --padding-top: 0;

    padding: 0 0 24px;
  }

  .modal-actions {
    display: flex;
    justify-content: space-between;
    padding: 12px;
    border-top: 1px solid rgb(0 0 0 / 6%);
    background: var(--ion-background-color, #fff);
  }

  .joystick-modal ion-item {
    --inner-padding-end: 12px;
  }
</style>
