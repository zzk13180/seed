<template>
  <div class="user-management">
    <!-- 搜索栏 -->
    <div class="search-bar">
      <el-form :inline="true" :model="state.searchForm" class="search-form">
        <el-form-item label="用户名">
          <el-input v-model="state.searchForm.username" placeholder="请输入用户名" clearable />
        </el-form-item>
        <el-form-item label="昵称">
          <el-input v-model="state.searchForm.nickname" placeholder="请输入昵称" clearable />
        </el-form-item>
        <el-form-item label="状态">
          <el-select v-model="state.searchForm.status" placeholder="请选择状态" clearable>
            <el-option label="启用" :value="UserStatus.ENABLED" />
            <el-option label="禁用" :value="UserStatus.DISABLED" />
          </el-select>
        </el-form-item>
        <el-form-item>
          <el-button type="primary" @click="controller.handleSearch()"> 搜索 </el-button>
          <el-button @click="controller.handleReset()"> 重置 </el-button>
        </el-form-item>
      </el-form>
    </div>

    <!-- 操作栏 -->
    <div class="action-bar">
      <el-button type="primary" @click="controller.openCreateDialog()"> 新增用户 </el-button>
      <el-button type="danger" :disabled="!canBatchDelete" @click="controller.handleBatchDelete()">
        批量删除
      </el-button>
    </div>

    <!-- 表格 -->
    <el-table
      v-loading="state.loading"
      :data="state.userList"
      border
      stripe
      @selection-change="controller.handleSelectionChange"
    >
      <el-table-column type="selection" width="55" />
      <el-table-column prop="id" label="ID" width="80" />
      <el-table-column prop="username" label="用户名" width="120" />
      <el-table-column prop="nickname" label="昵称" width="120" />
      <el-table-column prop="email" label="邮箱" width="180" />
      <el-table-column prop="phone" label="手机号" width="130" />
      <el-table-column prop="status" label="状态" width="100">
        <template #default="{ row }">
          <el-switch
            v-model="row.status"
            :active-value="UserStatus.ENABLED"
            :inactive-value="UserStatus.DISABLED"
            @change="controller.handleStatusChange(row)"
          />
        </template>
      </el-table-column>
      <el-table-column prop="createdAt" label="创建时间" width="180">
        <template #default="{ row }">
          {{ controller.formatDate(row.createdAt) }}
        </template>
      </el-table-column>
      <el-table-column label="操作" width="200" fixed="right">
        <template #default="{ row }">
          <el-button type="primary" link @click="controller.openEditDialog(row)">编辑</el-button>
          <el-button type="warning" link @click="controller.openResetPasswordDialog(row)"
            >重置密码</el-button
          >
          <el-button type="danger" link @click="controller.handleDelete(row)">删除</el-button>
        </template>
      </el-table-column>
    </el-table>

    <!-- 分页 -->
    <div class="pagination">
      <el-pagination
        v-model:current-page="state.pagination.page"
        v-model:page-size="state.pagination.pageSize"
        :page-sizes="[10, 20, 50, 100]"
        :total="state.pagination.total"
        layout="total, sizes, prev, pager, next, jumper"
        @size-change="controller.handleSizeChange"
        @current-change="controller.handlePageChange"
      />
    </div>

    <!-- 用户表单弹窗 -->
    <el-dialog
      v-model="state.dialogVisible"
      :title="dialogTitle"
      width="500px"
      :close-on-click-modal="false"
    >
      <el-form ref="formRef" :model="state.userForm" :rules="formRules" label-width="80px">
        <el-form-item label="用户名" prop="username">
          <el-input
            v-model="state.userForm.username"
            placeholder="请输入用户名"
            :disabled="state.isEdit"
          />
        </el-form-item>
        <el-form-item v-if="!state.isEdit" label="密码" prop="password">
          <el-input
            v-model="state.userForm.password"
            type="password"
            placeholder="请输入密码"
            show-password
          />
        </el-form-item>
        <el-form-item label="昵称" prop="nickname">
          <el-input v-model="state.userForm.nickname" placeholder="请输入昵称" />
        </el-form-item>
        <el-form-item label="邮箱" prop="email">
          <el-input v-model="state.userForm.email" placeholder="请输入邮箱" />
        </el-form-item>
        <el-form-item label="手机号" prop="phone">
          <el-input v-model="state.userForm.phone" placeholder="请输入手机号" />
        </el-form-item>
      </el-form>
      <template #footer>
        <el-button @click="controller.closeDialog()">取消</el-button>
        <el-button type="primary" :loading="state.submitLoading" @click="handleSubmit">
          确定
        </el-button>
      </template>
    </el-dialog>

    <!-- 重置密码弹窗 -->
    <el-dialog
      v-model="state.resetPasswordVisible"
      title="重置密码"
      width="400px"
      :close-on-click-modal="false"
    >
      <el-form
        ref="resetFormRef"
        :model="state.resetForm"
        :rules="resetFormRules"
        label-width="80px"
      >
        <el-form-item label="新密码" prop="password">
          <el-input
            v-model="state.resetForm.password"
            type="password"
            placeholder="请输入新密码"
            show-password
          />
        </el-form-item>
        <el-form-item label="确认密码" prop="confirmPassword">
          <el-input
            v-model="state.resetForm.confirmPassword"
            type="password"
            placeholder="请再次输入密码"
            show-password
          />
        </el-form-item>
      </el-form>
      <template #footer>
        <el-button @click="controller.closeResetPasswordDialog()">取消</el-button>
        <el-button type="primary" :loading="state.resetLoading" @click="handleResetSubmit">
          确定
        </el-button>
      </template>
    </el-dialog>
  </div>
</template>

<script setup lang="ts">
  import { ref, onMounted } from 'vue'
  import { useUserManagementStore } from './user-management.store'
  import { UserStatus } from './user-management.types'
  import type { FormInstance, FormRules } from 'element-plus'

  const store = useUserManagementStore()
  const { state, controller, dialogTitle, canBatchDelete } = store

  const formRef = ref<FormInstance>()
  const resetFormRef = ref<FormInstance>()

  // 表单验证规则（保留在视图层，因为是 UI 相关）
  const formRules: FormRules = {
    username: [
      { required: true, message: '请输入用户名', trigger: 'blur' },
      { min: 3, max: 50, message: '用户名长度在 3 到 50 个字符', trigger: 'blur' },
    ],
    password: [
      { required: true, message: '请输入密码', trigger: 'blur' },
      { min: 6, max: 50, message: '密码长度在 6 到 50 个字符', trigger: 'blur' },
    ],
    email: [{ type: 'email', message: '请输入正确的邮箱地址', trigger: 'blur' }],
    phone: [{ pattern: /^1[3-9]\d{9}$/, message: '请输入正确的手机号', trigger: 'blur' }],
  }

  const resetFormRules: FormRules = {
    password: [
      { required: true, message: '请输入新密码', trigger: 'blur' },
      { min: 6, max: 50, message: '密码长度在 6 到 50 个字符', trigger: 'blur' },
    ],
    confirmPassword: [
      { required: true, message: '请再次输入密码', trigger: 'blur' },
      {
        validator: (_rule, value, callback) => {
          if (value === state.resetForm.password) {
            callback()
          } else {
            callback(new Error('两次输入的密码不一致'))
          }
        },
        trigger: 'blur',
      },
    ],
  }

  /**
   * 提交表单（包含 UI 验证）
   */
  const handleSubmit = async () => {
    const valid = await formRef.value?.validate().catch(() => false)
    if (!valid) return
    await controller.handleSubmit()
  }

  /**
   * 提交重置密码（包含 UI 验证）
   */
  const handleResetSubmit = async () => {
    const valid = await resetFormRef.value?.validate().catch(() => false)
    if (!valid) return
    await controller.handleResetSubmit()
  }

  onMounted(() => {
    controller.fetchUserList()
  })
</script>

<style lang="scss" scoped>
  .user-management {
    padding: 20px;

    .search-bar {
      margin-bottom: 16px;
      padding: 16px;
      background: var(--el-bg-color);
      border-radius: 4px;

      .search-form {
        display: flex;
        flex-wrap: wrap;
        gap: 12px;
      }
    }

    .action-bar {
      margin-bottom: 16px;
      display: flex;
      gap: 12px;
    }

    .pagination {
      margin-top: 16px;
      display: flex;
      justify-content: flex-end;
    }
  }
</style>
