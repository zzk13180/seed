<template>
  <div class="panel-container">
    <el-card class="panel-header">
      <div class="panel-header-content">
        <span class="panel-title">机器人面板</span>
        <div class="panel-status">
          <el-tag :type="statusTagType" size="small">
            {{ robotStatusText }}
          </el-tag>
          <span v-if="state.robotInfo.batteryLevel" class="battery-level">
            🔋 {{ state.robotInfo.batteryLevel }}%
          </span>
        </div>
      </div>
    </el-card>

    <el-card class="panel-content" v-loading="state.loading">
      <template v-if="state.errorMessage">
        <el-alert
          :title="state.errorMessage"
          type="error"
          show-icon
          closable
          @close="handleClearError"
        />
      </template>

      <template v-else>
        <div class="robot-info">
          <el-descriptions :column="2" border>
            <el-descriptions-item label="机器人 ID">
              {{ state.robotInfo.robotId || '-' }}
            </el-descriptions-item>
            <el-descriptions-item label="名称">
              {{ state.robotInfo.name || '-' }}
            </el-descriptions-item>
            <el-descriptions-item label="状态">
              <el-tag :type="statusTagType" size="small">
                {{ robotStatusText }}
              </el-tag>
            </el-descriptions-item>
            <el-descriptions-item label="最后更新">
              {{ formatDate(state.robotInfo.lastUpdated) }}
            </el-descriptions-item>
          </el-descriptions>
        </div>

        <div class="panel-actions">
          <el-button type="primary" @click="handleRefresh" :loading="state.loading">
            刷新
          </el-button>
        </div>
      </template>
    </el-card>
  </div>
</template>

<script setup lang="ts">
  import { computed, onMounted, onUnmounted } from 'vue'
  import { storeToRefs } from 'pinia'
  import { usePanelStore } from './panel.store'

  const panelStore = usePanelStore()
  const { state, controller } = storeToRefs(panelStore)

  // 计算属性
  const statusTagType = computed(() => {
    switch (state.value.robotInfo.status) {
      case 'online': {
        return 'success'
      }
      case 'offline': {
        return 'info'
      }
      case 'error': {
        return 'danger'
      }
      default: {
        return 'info'
      }
    }
  })

  const robotStatusText = computed(() => {
    switch (state.value.robotInfo.status) {
      case 'online': {
        return '在线'
      }
      case 'offline': {
        return '离线'
      }
      case 'error': {
        return '异常'
      }
      default: {
        return '未知'
      }
    }
  })

  // 格式化日期
  const formatDate = (dateStr?: string): string => {
    if (!dateStr) return '-'
    try {
      return new Date(dateStr).toLocaleString('zh-CN')
    } catch {
      return dateStr
    }
  }

  // 刷新数据
  const handleRefresh = async () => {
    await controller.value.loadRobotInfo()
  }

  // 清除错误
  const handleClearError = () => {
    controller.value.clearError()
  }

  onMounted(async () => {
    try {
      await controller.value.initialize()
    } catch (error) {
      console.error('Panel initialization failed:', error)
    }
  })

  onUnmounted(async () => {
    await controller.value.dispose()
  })
</script>

<style scoped lang="scss">
  .panel-container {
    display: flex;
    flex-direction: column;
    gap: 16px;
    height: 100%;

    .panel-header {
      .panel-header-content {
        display: flex;
        align-items: center;
        justify-content: space-between;

        .panel-title {
          font-size: 18px;
          font-weight: 600;
        }

        .panel-status {
          display: flex;
          align-items: center;
          gap: 12px;

          .battery-level {
            font-size: 14px;
            color: var(--el-text-color-secondary);
          }
        }
      }
    }

    .panel-content {
      flex: 1;
      overflow: auto;

      .robot-info {
        margin-bottom: 24px;
      }

      .panel-actions {
        display: flex;
        gap: 12px;
      }
    }
  }
</style>
