<template>
  <transition mode="out-in">
    <router-link key="expand" to="/" class="the-logo">
      <svg v-if="svgContent" class="the-logo-svg">
        <use :xlink:href="'#vv-logo-svg-symbol'"></use>
      </svg>
      <h1>{{ title }}</h1>
    </router-link>
  </transition>
</template>

<script setup lang="ts">
  const logoUrl = '/logo/default.svg'
  const title = 'Title'
  const svgContent = ref<string | null>(null)

  onMounted(async () => {
    const response = await fetch(logoUrl)
    const svgText = await response.text()
    svgContent.value = svgText
    const symbol = document.createElementNS('http://www.w3.org/2000/svg', 'symbol')
    symbol.setAttribute('id', 'vv-logo-svg-symbol')
    symbol.innerHTML = svgText
    document.body.append(symbol)
  })
</script>

<style lang="scss" scoped>
  .the-logo {
    height: 100%;
    display: flex;
    justify-content: flex-start;
    align-items: center;
    gap: 4px;

    .the-logo-svg {
      width: 24px;
      height: 24px;
      color: #fff;
    }

    h1 {
      font-weight: 600;
      font-size: 16px;
      color: var(--el-color-white);
      margin-left: var(--vv-gap-small);
      height: 100%;
      display: flex;
      align-items: center;
    }
  }
</style>
