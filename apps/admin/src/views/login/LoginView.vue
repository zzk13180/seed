<template>
  <div class="login-container">
    <div class="left-section">
      <div class="login-the-logo-container">
        <TheLogo />
      </div>
    </div>
    <div class="right-section">
      <el-form ref="loginRef" :model="loginForm" :rules="loginRules" class="login-form">
        <div class="login-form-welcome">欢迎登录</div>
        <el-form-item prop="username">
          <el-input
            v-model="loginForm.username"
            type="text"
            size="large"
            clearable
            auto-complete="off"
            placeholder="请输入账号（必填）"
          >
          </el-input>
        </el-form-item>
        <el-form-item prop="password">
          <el-input
            v-model="loginForm.password"
            type="password"
            size="large"
            clearable
            auto-complete="off"
            placeholder="请输入密码（必填）"
            @keyup.enter="handleLogin"
          >
          </el-input>
        </el-form-item>
        <div class="login-form-remember-me">
          <el-checkbox v-model="loginForm.rememberMe">记住密码</el-checkbox>
        </div>
        <el-form-item class="login-form-btn">
          <el-button :loading="loading" size="large" type="primary" @click.prevent="handleLogin">
            <span v-if="!loading">登录</span>
            <span v-else>登录中...</span>
          </el-button>
        </el-form-item>
      </el-form>
    </div>
  </div>
</template>

<script setup lang="ts">
  import { ref, onMounted } from 'vue'
  import { useRouter } from 'vue-router'
  import TheLogo from '@/components/TheLogo.vue'
  import { AccessTokenUtil } from '@/utils/token.util'

  const router = useRouter()
  const loginRef = ref()
  const loading = ref(false)

  const loginForm = ref({
    username: 'admin',
    password: 'admin',
    rememberMe: false,
  })

  const loginRules = {
    username: [{ required: true, trigger: 'blur', message: '请输入您的账号' }],
    password: [{ required: true, trigger: 'blur', message: '请输入您的密码' }],
  }

  const handleLogin = () => {
    loginRef.value.validate((valid: boolean) => {
      if (valid) {
        loading.value = true

        try {
          AccessTokenUtil.setToken('mock-token')
          if (loginForm.value.rememberMe) {
            localStorage.setItem('username', loginForm.value.username)
            localStorage.setItem('password', loginForm.value.password)
            localStorage.setItem('rememberMe', 'true')
          } else {
            localStorage.removeItem('username')
            localStorage.removeItem('password')
            localStorage.removeItem('rememberMe')
          }

          router.push('/')
        } catch (error) {
          console.error('登录失败', error)
        } finally {
          loading.value = false
        }
      }
    })
  }

  const getLoginData = () => {
    const username = localStorage.getItem('username')
    const password = localStorage.getItem('password')
    const rememberMe = localStorage.getItem('rememberMe')

    if (username && password && rememberMe) {
      loginForm.value = {
        username,
        password,
        rememberMe: true,
      }
    }
  }

  onMounted(() => {
    getLoginData()
  })
</script>

<style lang="scss" scoped>
  .login-container {
    background-color: var(--el-bg-color-page);
    width: 100%;
    height: 100%;
    display: flex;
    justify-content: center;
    align-items: center;

    .left-section {
      width: 55%;
      height: 100%;

      .login-the-logo-container {
        width: 100%;
        height: var(--vv-navbar-height);
        padding-left: 32px;

        :deep(.the-logo) {
          user-select: none;
          pointer-events: none;

          .the-logo-svg {
            color: var(--el-color-primary);
          }

          h1 {
            color: var(--el-color-primary);
          }
        }
      }
    }

    .right-section {
      width: 45%;
      height: 100%;
      background-color: var(--el-bg-color);
      display: flex;
      justify-content: center;
      align-items: center;

      .login-form {
        --el-component-size-large: 48px;

        width: 320px;
        display: flex;
        flex-direction: column;
        justify-content: center;
        align-items: center;

        .login-form-welcome {
          width: 100%;
          font-weight: 600;
          font-size: 16px;
          margin-bottom: 16px;
          color: var(--el-text-color-primary);
        }

        .el-form-item {
          width: 100%;
          margin-bottom: 16px;
        }

        .login-form-remember-me {
          width: 100%;

          :deep(.el-checkbox) {
            height: 22px;
            font-weight: 400;
            font-size: 14px;
          }
        }

        .login-form-btn {
          width: 100%;
          height: 48px;
          margin-top: 48px;

          button {
            width: 100%;
            height: 100%;

            span {
              font-weight: 400;
              font-size: 16px;
            }
          }
        }

        :deep(.el-checkbox__label) {
          padding-left: 4px;
        }
      }
    }
  }

  @media (width <= 1024px) {
    .login-container {
      .left-section {
        width: 50%;
      }

      .right-section {
        width: 50%;
      }
    }
  }

  @media (width <= 800px) {
    .login-container {
      flex-direction: column;
      background-color: var(--el-bg-color);

      .left-section {
        height: auto;
      }

      .right-section {
        width: 100%;
        height: auto;
      }
    }
  }
</style>
