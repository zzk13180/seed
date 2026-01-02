#!/bin/bash
set -euo pipefail

# ============================================================
# Seed Monorepo 开发辅助脚本
# 核心理念：使用 NX 调度各项目，具体部署由各项目自行管理
# ============================================================

readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
readonly VERSION="2.0.0"

source "$SCRIPT_DIR/utils/colors.sh"

show_help() {
    cat << EOF
${CYAN}Seed Monorepo 开发辅助脚本 v${VERSION}${NC}

${YELLOW}注意: Docker/PM2 部署已移至各项目独立管理${NC}

用法: $0 <命令> [选项] [应用名称...]

${GREEN}命令:${NC}
    dev         开发模式 (通过 NX)
    build       构建项目 (通过 NX)
    test        运行测试 (通过 NX)
    lint        代码检查

${GREEN}选项:${NC}
    --parallel  并行执行
    --all       所有项目
    -h, --help  显示帮助

${GREEN}示例:${NC}
    $0 dev admin                # 开发 admin
    $0 dev admin server         # 同时开发多个应用
    $0 build --parallel         # 并行构建所有
    $0 build server             # 构建指定项目

${GREEN}各项目独立部署:${NC}
    cd apps/server && pnpm docker:up      # 后端 Docker 部署
    cd apps/server && pnpm pm2:start      # 后端 PM2 部署
    cd apps/admin && pnpm docker:build        # 前端 Docker 构建
EOF
}

# 开发模式
cmd_dev() {
    local apps=("$@")
    cd "$PROJECT_ROOT"
    
    if [ ${#apps[@]} -eq 0 ]; then
        log_warn "未指定应用，使用默认: admin"
        apps=("admin")
    fi
    
    if [ ${#apps[@]} -eq 1 ]; then
        log_step "开发模式: ${apps[0]}"
        pnpm nx dev "${apps[0]}"
    else
        log_step "并行开发: ${apps[*]}"
        pnpm nx run-many -t dev -p "${apps[*]}" --parallel
    fi
}

# 构建模式
cmd_build() {
    local apps=("$@")
    cd "$PROJECT_ROOT"
    
    local nx_args="-t build"
    [ "$PARALLEL" = true ] && nx_args+=" --parallel"
    
    if [ ${#apps[@]} -gt 0 ]; then
        log_step "构建应用: ${apps[*]}"
        pnpm nx run-many $nx_args -p "${apps[*]}"
    else
        log_step "构建所有应用"
        pnpm nx run-many $nx_args --all
    fi
    
    log_success "构建完成"
}

# 测试模式
cmd_test() {
    local apps=("$@")
    cd "$PROJECT_ROOT"
    
    if [ ${#apps[@]} -gt 0 ]; then
        log_step "测试应用: ${apps[*]}"
        pnpm nx run-many -t test -p "${apps[*]}"
    else
        log_step "运行所有测试"
        pnpm test
    fi
}

# Lint
cmd_lint() {
    cd "$PROJECT_ROOT"
    log_step "代码检查"
    pnpm lint
}

# 解析参数
COMMAND=""
PARALLEL=false
APPS=()

while [[ $# -gt 0 ]]; do
    case $1 in
        dev|build|test|lint)
            COMMAND="$1"
            shift
            ;;
        --parallel|-P)
            PARALLEL=true
            shift
            ;;
        --all)
            shift
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        -v|--version)
            echo "v$VERSION"
            exit 0
            ;;
        -*)
            log_error "未知选项: $1"
            show_help
            exit 1
            ;;
        *)
            APPS+=("$1")
            shift
            ;;
    esac
done

main() {
    case "$COMMAND" in
        dev)   cmd_dev "${APPS[@]}" ;;
        build) cmd_build "${APPS[@]}" ;;
        test)  cmd_test "${APPS[@]}" ;;
        lint)  cmd_lint ;;
        "")
            show_help
            exit 0
            ;;
        *)
            log_error "未知命令: $COMMAND"
            show_help
            exit 1
            ;;
    esac
}

main

# Docker Compose 部署
deploy_docker() {
    log_step "Docker Compose 部署"
    cd "$PROJECT_ROOT"
    
    # 检查 Docker 是否可用
    if ! command_exists docker; then
        log_error "Docker 未安装，请先运行: ./scripts/docker/install.sh"
        exit 1
    fi
    
    if ! docker info &>/dev/null; then
        log_error "Docker 服务未运行，请启动 Docker"
        exit 1
    fi
    
    # 选择 compose 文件
    local compose_file="docker-compose.yml"
    [ "$PROD" = true ] && compose_file="docker-compose.prod.yml"
    
    if [ ! -f "$compose_file" ]; then
        log_error "未找到 $compose_file"
        exit 1
    fi
    
    # 检查环境变量文件
    local env_file="${ENV_FILE:-.env}"
    if [ ! -f "$env_file" ]; then
        if [ -f ".env.example" ]; then
            log_warn "未找到 $env_file 文件，从 .env.example 复制..."
            cp .env.example "$env_file"
            log_warn "请编辑 $env_file 文件配置正确的环境变量"
        else
            log_error "未找到 $env_file 或 .env.example"
            exit 1
        fi
    fi
    
    # 构建 compose 参数
    local compose_args="-f $compose_file --env-file $env_file"
    local up_args=""
    [ "$BUILD" = true ] && up_args+=" --build"
    [ "$DETACH" = true ] && up_args+=" -d"
    
    log_info "配置: compose=$compose_file, env=$env_file"
    
    # 执行部署
    docker compose $compose_args up $up_args
    
    if [ "$DETACH" = true ]; then
        log_success "服务已在后台启动"
        log_info "查看状态: docker compose $compose_args ps"
        log_info "查看日志: docker compose $compose_args logs -f"
    fi
}

# PM2 部署
deploy_pm2() {
    local apps=("$@")
    log_step "PM2 部署"
    
    # 先构建项目（使用 NX）
    if [ "$BUILD" = true ]; then
        log_info "通过 NX 构建项目..."
        cd "$PROJECT_ROOT"
        if [ ${#apps[@]} -gt 0 ]; then
            pnpm nx run-many -t build -p "${apps[*]}" --parallel
        else
            pnpm nx run-many -t build --all --parallel
        fi
    fi
    
    # 启动 PM2
    if [ ${#apps[@]} -gt 0 ]; then
        "$SCRIPT_DIR/pm2/start.sh" start "${apps[@]}"
    else
        "$SCRIPT_DIR/pm2/start.sh" start
    fi
}

# 构建模式（通过 NX）
deploy_build() {
    local apps=("$@")
    log_step "构建项目"
    cd "$PROJECT_ROOT"
    
    local nx_args="-t build"
    [ "$PARALLEL" = true ] && nx_args+=" --parallel"
    
    if [ ${#apps[@]} -gt 0 ]; then
        log_info "构建应用: ${apps[*]}"
        pnpm nx run-many $nx_args -p "${apps[*]}"
    else
        log_info "构建所有应用"
        pnpm nx run-many $nx_args --all
    fi
    
    log_success "构建完成"
}

# 开发模式（通过 NX）
deploy_dev() {
    local apps=("$@")
    cd "$PROJECT_ROOT"
    
    if [ ${#apps[@]} -eq 0 ]; then
        log_warn "未指定应用，使用默认: admin"
        apps=("admin")
    fi
    
    if [ ${#apps[@]} -eq 1 ]; then
        log_step "开发模式: ${apps[0]}"
        pnpm nx dev "${apps[0]}"
    else
        log_step "并行开发模式: ${apps[*]}"
        # 使用 NX 并行运行多个 dev
        pnpm nx run-many -t dev -p "${apps[*]}" --parallel
    fi
}

# 解析参数
DEPLOY_MODE=""
BUILD=false
DETACH=false
PARALLEL=false
PROD=false
ENV_FILE=""
APPS=()

while [[ $# -gt 0 ]]; do
    case $1 in
        docker|pm2|dev|build)
            DEPLOY_MODE="$1"
            shift
            ;;
        --build|-b)
            BUILD=true
            shift
            ;;
        --detach|-d)
            DETACH=true
            shift
            ;;
        --parallel|-P)
            PARALLEL=true
            shift
            ;;
        --prod)
            PROD=true
            shift
            ;;
        --env)
            ENV_FILE="$2"
            shift 2
            ;;
        -v|--version)
            echo "deploy.sh version $VERSION"
            exit 0
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        -*)
            log_error "未知选项: $1"
            show_help
            exit 1
            ;;
        *)
            # 应用名称
            APPS+=("$1")
            shift
            ;;
    esac
done

main() {
    print_header "Seed Monorepo 部署"
    
    # 显示部署信息
    log_info "模式: $DEPLOY_MODE"
    [ ${#APPS[@]} -gt 0 ] && log_info "应用: ${APPS[*]}"
    [ "$BUILD" = true ] && log_info "选项: --build"
    [ "$DETACH" = true ] && log_info "选项: --detach"
    [ "$PARALLEL" = true ] && log_info "选项: --parallel"
    [ "$PROD" = true ] && log_info "选项: --prod"
    echo ""
    
    case "$DEPLOY_MODE" in
        docker)
            deploy_docker
            ;;
        pm2)
            deploy_pm2 "${APPS[@]}"
            ;;
        build)
            deploy_build "${APPS[@]}"
            ;;
        dev)
            deploy_dev "${APPS[@]}"
            ;;
        "")
            log_error "请指定部署方式"
            echo ""
            show_help
            exit 1
            ;;
        *)
            log_error "未知部署方式: $DEPLOY_MODE"
            show_help
            exit 1
            ;;
    esac
}

main
