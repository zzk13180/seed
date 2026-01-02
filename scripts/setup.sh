#!/bin/bash
set -euo pipefail

# ============================================================
# Seed Monorepo 环境安装脚本
# 功能：自动安装 Node.js LTS、pnpm、项目依赖
# ============================================================

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
readonly VERSION="1.0.0"
readonly MIN_NODE_VERSION=20

# 加载工具函数
source "$SCRIPT_DIR/utils/colors.sh"

# 信号捕获
cleanup() {
    local exit_code=$?
    if [ $exit_code -ne 0 ]; then
        log_error "安装中断，退出码: $exit_code"
    fi
    exit $exit_code
}
trap cleanup EXIT INT TERM

# 命令行参数
SKIP_NODE=false
SKIP_PNPM=false
SKIP_DEPS=false
INSTALL_PM2=false
FORCE=false

show_help() {
    cat << EOF
${CYAN}Seed Monorepo 环境安装脚本 v${VERSION}${NC}

用法: $0 [选项]

${GREEN}选项:${NC}
    --skip-node     跳过 Node.js 安装
    --skip-pnpm     跳过 pnpm 安装
    --skip-deps     跳过项目依赖安装
    --with-pm2      同时安装 PM2
    --force         强制重新安装
    -v, --version   显示版本
    -h, --help      显示帮助

${GREEN}示例:${NC}
    $0                          # 完整安装
    $0 --skip-node              # 跳过 Node.js
    $0 --with-pm2               # 包含 PM2
    $0 --force                  # 强制重装
EOF
}

# 解析参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --skip-node)  SKIP_NODE=true; shift ;;
        --skip-pnpm)  SKIP_PNPM=true; shift ;;
        --skip-deps)  SKIP_DEPS=true; shift ;;
        --with-pm2)   INSTALL_PM2=true; shift ;;
        --force)      FORCE=true; shift ;;
        -v|--version) echo "setup.sh version $VERSION"; exit 0 ;;
        -h|--help)    show_help; exit 0 ;;
        *) log_error "未知选项: $1"; show_help; exit 1 ;;
    esac
done

# ============================================================
# 1. 安装 Node.js LTS
# ============================================================
install_nodejs() {
    log_step "检查 Node.js 环境..."
    
    if command_exists node && [ "$FORCE" != true ]; then
        local current_version=$(node --version)
        log_info "检测到 Node.js 已安装: $current_version"
        
        # 检查版本是否满足要求
        local major_version=$(echo "$current_version" | sed 's/v\([0-9]*\).*/\1/')
        if [ "$major_version" -lt "$MIN_NODE_VERSION" ]; then
            log_warn "Node.js 版本过低 (>=$MIN_NODE_VERSION 必需)，将升级..."
        else
            log_success "Node.js 版本满足要求 (最低 v$MIN_NODE_VERSION)"
            return 0
        fi
    fi

    log_info "正在获取 Node.js 最新 LTS 版本..."
    
    local LTS_VERSION_TAG=""
    if command_exists curl; then
        LTS_VERSION_TAG=$(curl -fsSL --connect-timeout 10 https://nodejs.org/dist/index.tab 2>/dev/null | awk 'NR>1 && $10!="-" {print $1; exit}')
    elif command_exists wget; then
        LTS_VERSION_TAG=$(wget -qO- --timeout=10 https://nodejs.org/dist/index.tab 2>/dev/null | awk 'NR>1 && $10!="-" {print $1; exit}')
    fi
    
    if [ -z "$LTS_VERSION_TAG" ]; then
        log_warn "无法获取最新 LTS 版本，使用默认版本 v22.12.0"
        LTS_VERSION_TAG="v22.12.0"
    fi
    
    log_info "准备安装 Node.js $LTS_VERSION_TAG..."

    # 检测系统架构
    local ARCH OS
    case "$(uname -s)" in
        Linux*)   OS="linux" ;;
        Darwin*)  OS="darwin" ;;
        *)        log_error "不支持的操作系统: $(uname -s)"; exit 1 ;;
    esac
    
    case "$(uname -m)" in
        x86_64)         ARCH="x64" ;;
        aarch64|arm64)  ARCH="arm64" ;;
        armv7l|armv6l)  ARCH="armv7l" ;;
        *)              log_error "不支持的架构: $(uname -m)"; exit 1 ;;
    esac

    local INSTALL_DIR="$HOME/.local/node"
    mkdir -p "$INSTALL_DIR"
    
    local FILE_EXT="tar.xz"
    [ "$OS" = "darwin" ] && FILE_EXT="tar.gz"
    
    local DOWNLOAD_URL="https://nodejs.org/dist/$LTS_VERSION_TAG/node-$LTS_VERSION_TAG-$OS-$ARCH.$FILE_EXT"
    log_info "下载中: $DOWNLOAD_URL"
    
    if command_exists curl; then
        curl -fsSL "$DOWNLOAD_URL" -o /tmp/node.tar.xz
    else
        wget -qO /tmp/node.tar.xz "$DOWNLOAD_URL"
    fi
    
    log_info "解压中..."
    if [ "$FILE_EXT" = "tar.gz" ]; then
        tar -xzf /tmp/node.tar.xz -C "$INSTALL_DIR" --strip-components=1
    else
        tar -xJf /tmp/node.tar.xz -C "$INSTALL_DIR" --strip-components=1
    fi
    rm -f /tmp/node.tar.xz
    
    add_to_path "$INSTALL_DIR/bin"

    if ! command_exists node; then
        log_error "Node.js 安装失败"
        exit 1
    fi
    log_success "Node.js 安装完成: $(node --version)"
}

# ============================================================
# 2. 安装 pnpm
# ============================================================
install_pnpm() {
    log_step "检查 pnpm 环境..."
    
    # 配置 npm 全局目录
    local NPM_PREFIX="$HOME/.npm-global"
    mkdir -p "$NPM_PREFIX"
    npm config set prefix "$NPM_PREFIX" 2>/dev/null || true
    add_to_path "$NPM_PREFIX/bin"
    
    if command_exists pnpm && [ "$FORCE" != true ]; then
        local current_version=$(pnpm --version 2>/dev/null || echo "unknown")
        log_info "pnpm 已安装: v$current_version"
        
        # 检查是否需要更新
        log_info "检查 pnpm 更新..."
        npm install -g pnpm@latest 2>/dev/null || true
    else
        log_info "正在安装 pnpm..."
        npm install -g pnpm@latest || { log_error "pnpm 安装失败"; exit 1; }
    fi
    
    log_success "pnpm 版本: $(pnpm --version)"
    
    # 配置 pnpm 全局安装路径
    local PNPM_GLOBAL_BIN="$HOME/.local/share/pnpm"
    mkdir -p "$PNPM_GLOBAL_BIN"
    pnpm config set global-bin-dir "$PNPM_GLOBAL_BIN" 2>/dev/null || true
    add_to_path "$PNPM_GLOBAL_BIN"
}

# ============================================================
# 3. 安装项目依赖
# ============================================================
install_dependencies() {
    log_step "安装项目依赖..."
    cd "$PROJECT_ROOT"
    
    if [ -f "package.json" ]; then
        pnpm install || { log_error "依赖安装失败"; exit 1; }
        log_success "项目依赖安装完成"
    else 
        log_warn "未找到 package.json，跳过依赖安装"
    fi
}

# ============================================================
# 4. 安装全局工具
# ============================================================
install_global_tools() {
    log_step "安装全局工具..."
    
    # 只安装 pm2，nx 通过 pnpm nx 调用本地版本
    if ! command_exists pm2; then
        log_info "安装 pm2..."
        pnpm add -g pm2 || log_warn "pm2 安装失败，可稍后手动安装"
    else
        log_info "pm2 已安装"
    fi
}

# ============================================================
# 主流程
# ============================================================
main() {
    print_header "Seed Monorepo 环境安装"
    
    # 显示配置信息
    log_info "项目目录: $PROJECT_ROOT"
    [ "$FORCE" = true ] && log_info "模式: 强制重装"
    [ "$SKIP_NODE" = true ] && log_info "跳过: Node.js"
    [ "$SKIP_PNPM" = true ] && log_info "跳过: pnpm"
    [ "$SKIP_DEPS" = true ] && log_info "跳过: 项目依赖"
    echo ""
    
    # 执行安装步骤
    [ "$SKIP_NODE" != true ] && install_nodejs
    [ "$SKIP_PNPM" != true ] && install_pnpm
    [ "$SKIP_DEPS" != true ] && install_dependencies
    
    # PM2 安装
    if [ "$INSTALL_PM2" = true ]; then
        install_global_tools
    elif [ "$SKIP_DEPS" != true ]; then
        echo ""
        if confirm "是否安装 PM2 进程管理器？"; then
            install_global_tools
        fi
    fi
    
    echo ""
    log_success "环境安装完成！"
    echo ""
    echo -e "${CYAN}下一步操作:${NC}"
    echo "  - 开发模式: pnpm nx dev admin"
    echo "  - 构建项目: pnpm nx build server"
    echo "  - PM2 部署: cd apps/server && pnpm pm2:start"
    echo "  - Docker 部署: cd apps/server && pnpm docker:up"
    echo ""
    
    # 提示重新加载 shell
    log_info "如果路径未生效，请运行: source $(get_shell_rc)"
}

main "$@"
