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
  import { useRouter } from 'vue-router'
  import TheLogo from '@/components/TheLogo.vue'
  import { AccessTokenUtil } from '@/utils/token.util'

  const router = useRouter()

  const loginForm = ref({
    tenantId: '000000',
    username: 'admin',
    password: 'admin123',
    rememberMe: false,
    code: '',
    uuid: '',
  })

  const loginRules = {
    tenantId: [{ required: true, trigger: 'blur', message: '请输入您的租户编号' }],
    username: [{ required: true, trigger: 'blur', message: '请输入您的账号' }],
    password: [{ required: true, trigger: 'blur', message: '请输入您的密码' }],
    code: [{ required: true, trigger: 'blur', message: '请输入验证码' }],
  }

  const loading = ref(false)

  const loginRef = ref()

  const handleLogin = () => {
    AccessTokenUtil.setToken('mock-token')
    router.push('/')
  }

  const getLoginData = () => {
    const tenantId = localStorage.getItem('tenantId')
    const username = localStorage.getItem('username')
    const password = localStorage.getItem('password')
    const rememberMe = localStorage.getItem('rememberMe')
    loginForm.value = {
      tenantId: tenantId === null ? String(loginForm.value.tenantId) : tenantId,
      username: username === null ? String(loginForm.value.username) : username,
      password: password === null ? String(loginForm.value.password) : String(password),
      rememberMe: rememberMe === null ? false : Boolean(rememberMe),
    } as any
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

        .login-form-the-logo-container {
          width: 100%;
          height: 128px;

          :deep(.the-logo) {
            user-select: none;
            pointer-events: none;
            gap: 4px;

            .the-logo-svg {
              color: var(--el-color-primary);
              position: relative;
              top: 1.5px;
            }

            h1 {
              font-size: 24px;
              color: var(--el-text-color-primary);
            }
          }
        }

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

        .login-input-icon {
          height: 40px;
          width: 14px;
          margin-left: 0;
        }

        .login-form-code {
          :deep(.el-input__suffix) {
            padding: 0;
            height: 43.5px;

            .el-input__suffix-inner {
              display: flex;
              flex-direction: row-reverse;

              .login-code {
                height: 100%;
                width: 100px;

                img {
                  border-top: 1px solid transparent;
                  border-bottom: 1px solid transparent;
                  width: 100%;
                  height: 100%;
                  cursor: pointer;
                  border-radius: 0 4px 4px 0;
                }
              }
            }
          }

          :deep(.el-input__wrapper) {
            padding: 0 1px 0 16px;
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
