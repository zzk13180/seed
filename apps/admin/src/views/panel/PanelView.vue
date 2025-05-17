<template>
  <div class="panel-container">
    <el-card class="header-card">
      <!-- 连接状态显示 -->
      <div>{{ connectionStatus }}</div>
    </el-card>

    <el-card class="main-card">
      <!-- 节点操作部分 -->
      <el-divider>基本操作</el-divider>

      <!-- 包管理 -->
      <div class="section">
        <el-button type="primary" @click="listPackages">列出所有ROS包</el-button>

        <div class="form-row">
          <span class="label">包名:</span>
          <el-input v-model="pkgName" size="small" style="width: 200px" />
          <el-button @click="listExecutables">列出此包的可执行文件</el-button>
        </div>
      </div>

      <!-- 节点管理 -->
      <div class="section">
        <div class="form-row">
          <span class="label">启动命令:</span>
          <el-input v-model="nodeCmd" size="small" style="width: 350px" />
          <el-button type="success" @click="startNode">启动节点</el-button>
        </div>

        <div class="form-row">
          <span class="label">节点名:</span>
          <el-input v-model="nodeNameInfo" size="small" style="width: 200px" />
          <el-button @click="getNodeInfo">查看节点信息</el-button>
          <el-button type="danger" @click="killNode">关闭节点</el-button>
        </div>
      </div>

      <!-- 响应结果显示 -->
      <el-divider>操作结果</el-divider>
      <el-scrollbar height="120px">
        <pre>{{ serviceResponse }}</pre>
      </el-scrollbar>

      <!-- TF话题监听 -->
      <el-divider>数据监听</el-divider>
      <h3>TF 话题数据</h3>
      <div>正在监听: <code>/seed_ros_nodes/tf_consolidated</code></div>
      <el-scrollbar height="150px">
        <pre>{{ tfConsolidatedData }}</pre>
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
  import * as ROSLIB from 'roslib'
  import { ref, onMounted } from 'vue'

  // 响应式状态变量
  const connectionStatus = ref('正在连接到ROS服务器...')
  const pkgName = ref('turtlesim') // 默认包名
  const nodeCmd = ref('rosrun turtlesim turtlesim_node') // 默认启动命令
  const nodeNameInfo = ref('/turtlesim') // 默认节点名
  const serviceResponse = ref('等待操作...')
  const tfConsolidatedData = ref('等待数据...')

  // ROS连接对象
  let ros: any

  // 组件挂载时建立ROS连接
  onMounted(() => {
    // 创建与ROS桥接服务器的连接
    ros = new ROSLIB.Ros({
      url: 'ws://10.211.55.5:5001', // ROS桥接服务器地址
    })

    // 连接成功事件处理
    ros.on('connection', () => {
      connectionStatus.value = '已连接到ROS服务器'
      subscribeToTfConsolidated() // 开始订阅TF话题
    })

    // 连接错误事件处理
    ros.on('error', (error: any) => {
      connectionStatus.value = `连接错误: ${error}`
      console.error('连接错误: ', error)
    })

    // 连接关闭事件处理
    ros.on('close', () => {
      connectionStatus.value = 'ROS服务器连接已关闭'
    })
  })

  // 显示服务调用响应
  function displayServiceResponse(data: any) {
    serviceResponse.value = JSON.stringify(data, null, 2)
  }

  // 列出所有ROS包
  function listPackages() {
    // 创建服务调用
    const service = new ROSLIB.Service({
      ros,
      name: '/seed_ros_nodes/list_packages',
      serviceType: 'seed_ros_nodes/ListPackages',
    })

    // 调用服务
    service.callService(
      new ROSLIB.ServiceRequest({}),
      (result: any) => displayServiceResponse(result),
      (error: any) => displayServiceResponse({ error }),
    )
  }

  // 列出指定包中的可执行文件
  function listExecutables() {
    const service = new ROSLIB.Service({
      ros,
      name: '/seed_ros_nodes/list_executables',
      serviceType: 'seed_ros_nodes/ListExecutables',
    })

    service.callService(
      new ROSLIB.ServiceRequest({ package: pkgName.value }),
      (result: any) => displayServiceResponse(result),
      (error: any) => displayServiceResponse({ error }),
    )
  }

  // 通用节点管理服务调用函数
  function callManageNodeService(serviceName: string, nodeValue: string) {
    const service = new ROSLIB.Service({
      ros,
      name: `/seed_ros_nodes/node/${serviceName}`,
      serviceType: 'seed_ros_nodes/ManageNode',
    })

    service.callService(
      new ROSLIB.ServiceRequest({ node: nodeValue }),
      (result: any) => displayServiceResponse(result),
      (error: any) => displayServiceResponse({ error }),
    )
  }

  // 启动节点
  function startNode() {
    callManageNodeService('start', nodeCmd.value)
  }

  // 获取节点信息
  function getNodeInfo() {
    callManageNodeService('info', nodeNameInfo.value)
  }

  // 关闭节点
  function killNode() {
    callManageNodeService('kill', nodeNameInfo.value)
  }

  // 订阅TF合并话题
  function subscribeToTfConsolidated() {
    const listener = new ROSLIB.Topic({
      ros,
      name: '/seed_ros_nodes/tf_consolidated',
      messageType: 'tf2_msgs/TFMessage',
    })

    listener.subscribe((message: any) => {
      tfConsolidatedData.value = JSON.stringify(message, null, 2)
    })
  }
</script>

<style scoped lang="scss">
  .panel-container {
    display: flex;
    flex-direction: column;
    gap: 16px;
    height: 100%;
    box-sizing: border-box;
  }

  .header-card {
    display: flex;
    align-items: center;
    justify-content: space-between;
  }

  .main-card {
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
