<template>
  <div class="panel-container">
    <el-card class="panel-header">
      <!-- 连接状态显示 -->
      <div>{{ state.connectionStatus }}</div>
    </el-card>

    <el-card class="panel-content">
      <!-- 节点操作部分 -->
      <el-divider>基本操作</el-divider>

      <!-- 包管理 -->
      <div class="section">
        <el-button type="primary" @click="controller.listPackages()">列出所有ROS包</el-button>

        <div class="form-row">
          <span class="label">包名:</span>
          <el-input v-model="state.pkgName" size="small" style="width: 200px" />
          <el-button @click="controller.listExecutables()">列出此包的可执行文件</el-button>
        </div>
      </div>

      <!-- 节点管理 -->
      <div class="section">
        <div class="form-row">
          <span class="label">启动命令:</span>
          <el-input v-model="state.nodeCmd" size="small" style="width: 350px" />
          <el-button type="success" @click="controller.startNode()">启动节点</el-button>
        </div>

        <div class="form-row">
          <span class="label">节点名:</span>
          <el-input v-model="state.nodeNameInfo" size="small" style="width: 200px" />
          <el-button @click="controller.getNodeInfo()">查看节点信息</el-button>
          <el-button type="danger" @click="controller.killNode()">关闭节点</el-button>
        </div>
      </div>

      <!-- 响应结果显示 -->
      <el-divider>操作结果</el-divider>
      <el-scrollbar height="120px">
        <pre>{{ state.serviceResponse }}</pre>
      </el-scrollbar>

      <!-- TF话题监听 -->
      <el-divider>数据监听</el-divider>
      <h3>TF 话题数据</h3>
      <div>正在监听: <code>/seed_core/tf_consolidated</code></div>
      <el-scrollbar height="150px">
        <pre>{{ state.tfConsolidatedData }}</pre>
      </el-scrollbar>

      <!-- 测试提示 -->
      <div class="test-tip">
        <div>测试方法：在终端发布消息到 /tf 话题</div>
        <el-tag type="info" effect="plain">
          rostopic pub /tf tf2_msgs/TFMessage "transforms: [{...}]" -r 1
        </el-tag>
      </div>
    </el-card>
  </div>
</template>

<script setup lang="ts">
  import { onMounted } from 'vue'
  import { usePanelStore } from './panel.store'

  const { state, controller } = usePanelStore()

  onMounted(() => {
    controller.onComponentMounted()
  })
</script>

<style scoped lang="scss">
  .panel-container {
    display: flex;
    flex-direction: column;
    gap: 16px;
    height: 100%;
    box-sizing: border-box;
  }

  .panel-header {
    display: flex;
    align-items: center;
    justify-content: space-between;
  }

  .panel-content {
    flex: 1;
    overflow: auto;
  }

  .section {
    margin: 15px 0;
  }

  .form-row {
    display: flex;
    align-items: center;
    margin: 10px 0;
    gap: 10px;
  }

  .label {
    font-weight: bold;
    width: 80px;
  }

  .test-tip {
    margin-top: 15px;
    padding: 10px;
    background-color: #f8f9fa;
    border-radius: 4px;
  }
</style>
