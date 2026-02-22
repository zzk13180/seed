#!/bin/bash
set -euo pipefail

# ============================================================
# Server API 部署脚本
# 用途：构建镜像、启动服务、管理 Docker Compose
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJECT_ROOT="$(cd "$APP_DIR/../.." && pwd)"
DOCKER_DIR="$APP_DIR/docker"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info()    { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn()    { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error()   { echo -e "${RED}[ERROR]${NC} $1" >&2; }
log_step()    { echo -e "${BLUE}[STEP]${NC} $1"; }
log_success() { echo -e "${GREEN}[✓]${NC} $1"; }

print_header() {
    echo ""
    echo -e "${CYAN}============================================${NC}"
    echo -e "${CYAN}  $1${NC}"
    echo -e "${CYAN}============================================${NC}"
    echo ""
}

show_help() {
    cat << EOF
${CYAN}Server API 部署脚本${NC}

用法: $0 <命令> [选项]

${GREEN}命令:${NC}
    up          启动服务 (docker compose up)
    down        停止服务 (docker compose down)
    build       构建 Docker 镜像
    logs        查看日志
    ps          查看容器状态
    db          仅启动数据库 (postgres + redis)
    pm2         使用 PM2 启动

${GREEN}选项:${NC}
    --prod      使用生产环境配置
    --build     启动时重新构建
    -d          后台运行
    --tag TAG   镜像标签 (默认: latest)

${GREEN}示例:${NC}
    $0 db                       # 启动数据库
    $0 up --build -d            # 构建并后台启动
    $0 up --prod -d             # 生产环境启动
    $0 build --tag v1.0.0       # 构建指定版本镜像
    $0 logs                     # 查看日志
    $0 pm2                      # PM2 模式启动
EOF
}

# 检查 Docker
check_docker() {
    if ! command -v docker &>/dev/null; then
        log_error "Docker 未安装"
        exit 1
    fi
    if ! docker info &>/dev/null; then
        log_error "Docker 服务未运行"
        exit 1
    fi
}

# 检查环境变量文件
check_env() {
    local env_file="$DOCKER_DIR/.env"
    if [ ! -f "$env_file" ]; then
        if [ -f "$DOCKER_DIR/.env.example" ]; then
            log_warn "未找到 .env 文件，从 .env.example 复制..."
            cp "$DOCKER_DIR/.env.example" "$env_file"
            log_warn "请编辑 $env_file 配置正确的环境变量"
        else
            log_error "未找到 .env 或 .env.example"
            exit 1
        fi
    fi
}

# 获取 compose 命令
get_compose_cmd() {
    local compose_file="docker-compose.yml"
    [ "$PROD" = true ] && compose_file="docker-compose.prod.yml"
    echo "docker compose -f $DOCKER_DIR/$compose_file --env-file $DOCKER_DIR/.env"
}

# 启动服务
cmd_up() {
    check_docker
    check_env
    
    log_step "启动 Server API 服务"
    
    local compose_cmd=$(get_compose_cmd)
    local up_args=""
    [ "$BUILD" = true ] && up_args+=" --build"
    [ "$DETACH" = true ] && up_args+=" -d"
    
    $compose_cmd up $up_args
    
    if [ "$DETACH" = true ]; then
        log_success "服务已在后台启动"
        log_info "查看状态: $0 ps"
        log_info "查看日志: $0 logs"
    fi
}

# 停止服务
cmd_down() {
    check_docker
    log_step "停止服务"
    local compose_cmd=$(get_compose_cmd)
    $compose_cmd down
    log_success "服务已停止"
}

# 仅启动数据库
cmd_db() {
    check_docker
    check_env
    
    log_step "启动数据库服务 (PostgreSQL + Redis)"
    local compose_cmd=$(get_compose_cmd)
    
    local up_args=""
    [ "$DETACH" = true ] && up_args+=" -d"
    
    $compose_cmd up postgres redis $up_args
    
    if [ "$DETACH" = true ]; then
        log_success "数据库服务已启动"
        log_info "PostgreSQL: localhost:5432"
        log_info "Redis: localhost:6379"
    fi
}

# 构建镜像
cmd_build() {
    check_docker
    
    log_step "构建 Server Docker 镜像"
    
    local image_name="seed-server"
    local tag="${TAG:-latest}"
    
    docker build \
        -f "$APP_DIR/Dockerfile" \
        -t "$image_name:$tag" \
        -t "$image_name:latest" \
        --build-arg BUILD_DATE="$(date -u +'%Y-%m-%dT%H:%M:%SZ')" \
        --build-arg VCS_REF="$(git rev-parse --short HEAD 2>/dev/null || echo 'unknown')" \
        --build-arg VERSION="$tag" \
        "$PROJECT_ROOT"
    
    log_success "镜像构建完成: $image_name:$tag"
}

# 查看日志
cmd_logs() {
    check_docker
    local compose_cmd=$(get_compose_cmd)
    $compose_cmd logs -f "${SERVICES[@]:-}"
}

# 查看状态
cmd_ps() {
    check_docker
    local compose_cmd=$(get_compose_cmd)
    $compose_cmd ps
}

# PM2 启动
cmd_pm2() {
    log_step "使用 PM2 启动 Server API"
    
    if ! command -v pm2 &>/dev/null; then
        log_error "PM2 未安装，请运行: pnpm add -g pm2"
        exit 1
    fi
    
    cd "$APP_DIR"
    
    if [ ! -f "ecosystem.config.cjs" ]; then
        log_error "未找到 ecosystem.config.cjs"
        exit 1
    fi
    
    pm2 start ecosystem.config.cjs --update-env
    pm2 save
    log_success "Server API 已通过 PM2 启动"
    pm2 list
}

# 解析参数
COMMAND=""
PROD=false
BUILD=false
DETACH=false
TAG=""
SERVICES=()

while [[ $# -gt 0 ]]; do
    case $1 in
        up|down|build|logs|ps|db|pm2)
            COMMAND="$1"
            shift
            ;;
        --prod)
            PROD=true
            shift
            ;;
        --build)
            BUILD=true
            shift
            ;;
        -d|--detach)
            DETACH=true
            shift
            ;;
        --tag)
            TAG="$2"
            shift 2
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            SERVICES+=("$1")
            shift
            ;;
    esac
done

# 主流程
main() {
    case "$COMMAND" in
        up)     cmd_up ;;
        down)   cmd_down ;;
        build)  cmd_build ;;
        logs)   cmd_logs ;;
        ps)     cmd_ps ;;
        db)     cmd_db ;;
        pm2)    cmd_pm2 ;;
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
