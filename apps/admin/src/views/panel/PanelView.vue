<template>
  <div class="panel-container">
    <el-card class="panel-header">header</el-card>

    <el-card class="panel-content">
      <el-button type="primary">button</el-button>
    </el-card>
  </div>
</template>

<script setup lang="ts">
  import { onMounted } from 'vue'
  import { usePanelStore } from './panel.store'

  const { state, controller } = usePanelStore()

  onMounted(() => {
    controller.initialize()
  })
</script>

<style scoped lang="scss">
  .panel-container {
    display: flex;
    flex-direction: column;
    gap: 16px;
    height: 100%;

    .panel-header {
      display: flex;
      align-items: center;
      justify-content: space-between;
    }

    .panel-content {
      flex: 1;
      overflow: auto;
    }
  }
</style>
