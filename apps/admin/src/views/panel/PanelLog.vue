<template>
  <el-card class="log-panel-main-component">
    <template #header>
      <div class="card-header">
        <span>操作日志</span>
        <el-button link type="primary" @click="controller.clearLogs()">清空日志</el-button>
      </div>
    </template>
    <div class="log-content-wrapper">
      <div class="log-content">
        <p v-for="(log, index) in state.logs" :key="index" :class="['log-item', log.type]">
          [{{ log.time }}] {{ log.message }}
        </p>
        <el-empty v-if="state.logs.length === 0" description="暂无日志" :image-size="50" />
      </div>
    </div>
  </el-card>
</template>

<script setup lang="ts">
  import { usePanelStore } from './panel.store'

  const { state, controller } = usePanelStore()
</script>

<style scoped lang="scss">
  .log-panel-main-component {
    /* flex: 1, display: flex, flex-direction: column, min-height: 0 由 PanelView.vue 控制 */
    overflow: hidden; /* 防止内部内容溢出卡片边界 */
  }

  .card-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
  }

  /* el-card 的主体部分需要 flex 布局以使 log-content 可滚动 */
  :deep(.el-card__body) {
    flex: 1;
    padding: 0 !important; /* 覆盖 Element Plus 的默认内边距 */
    display: flex;
    flex-direction: column;
    overflow: hidden; /* 确保子元素可以正确地使用 flex 和 overflow */
    min-height: 0; /* 配合 flex:1 和 overflow:auto */
  }

  .log-content-wrapper {
    flex: 1;
    display: flex;
    flex-direction: column;
    overflow-y: auto; /* 当日志内容超出时，仅包装器滚动 */
    background-color: #fdfdfd;
  }

  .log-content {
    padding: 16px;
    font-size: 0.9em;
    line-height: 1.5;
  }

  .log-item {
    word-break: break-all; /* 长消息换行 */
  }

  .log-item.success {
    color: #67c23a;
  }

  .log-item.error {
    color: #f56c6c;
  }

  .log-item.warning {
    color: #e6a23c;
  }

  .log-item.info {
    color: #909399;
  }

  .el-empty {
    padding: 16px 0; // 给空状态一些空间
  }
</style>
