<template>
  <div class="map-view-container">
    <el-card class="map-view-header">
      <div class="map-view-header-content">
        <span class="map-title">地图视图</span>
        <div class="map-status">
          <el-tag v-if="isViewReady" type="success" size="small">视图就绪</el-tag>
          <el-tag v-if="isGridReady" type="success" size="small">网格就绪</el-tag>
        </div>
      </div>
    </el-card>

    <el-card class="map-view-content" v-loading="state.loading">
      <template v-if="state.errorMessage">
        <el-alert
          :title="state.errorMessage"
          type="error"
          show-icon
          closable
          @close="controller.clearError()"
        />
      </template>

      <div class="map-view-canvas-container" ref="canvasContainerRef">
        <template v-if="isViewReady">
          <MapGridCanvas />
        </template>
      </div>
    </el-card>
  </div>
</template>

<script setup lang="ts">
  import { ref, onMounted, onBeforeUnmount } from 'vue'
  import MapGridCanvas from './MapGridCanvas/MapGridCanvas.vue'
  import { useMapStore } from './map.store'

  const canvasContainerRef = ref<HTMLDivElement | null>(null)

  const mapStore = useMapStore()
  const { state, controller, isViewReady, isGridReady } = mapStore

  onMounted(async () => {
    console.log('MapView mounted', performance.now())

    try {
      // 先初始化 BaseController
      await controller.initialize()

      // 然后初始化视图（需要 DOM 容器）
      if (canvasContainerRef.value) {
        controller.initializeView(canvasContainerRef.value)
      }
    } catch (error) {
      console.error('MapView initialization failed:', error)
    }
  })

  onBeforeUnmount(async () => {
    await controller.dispose()
  })
</script>

<style scoped lang="scss">
  .map-view-container {
    display: flex;
    flex-direction: column;
    gap: 16px;
    height: 100%;
    box-sizing: border-box;

    :deep(.el-card__body) {
      width: 100%;
      height: 100%;
    }

    .map-view-header {
      .map-view-header-content {
        display: flex;
        align-items: center;
        justify-content: space-between;

        .map-title {
          font-size: 18px;
          font-weight: 600;
        }

        .map-status {
          display: flex;
          gap: 8px;
        }
      }
    }

    .map-view-content {
      flex: 1;
      position: relative;

      :deep(.el-card__body) {
        padding: 0;
      }

      .map-view-canvas-container {
        width: 100%;
        height: 100%;
        position: absolute;
        left: 0;
        top: 0;

        > canvas {
          pointer-events: none;
        }
      }
    }
  }
</style>
