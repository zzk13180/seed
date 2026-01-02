#!/bin/bash
set -euo pipefail

# ============================================================
# Mobile 应用部署脚本
# 支持 Docker 构建和 PM2 管理
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJECT_ROOT="$(cd "$APP_DIR/../.." && pwd)"

# 颜色
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
RED='\033[0;31m'
NC='\033[0m'

log_info()    { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn()    { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error()   { echo -e "${RED}[ERROR]${NC} $1" >&2; }
log_step()    { echo -e "${BLUE}[STEP]${NC} $1"; }
log_success() { echo -e "${GREEN}[✓]${NC} $1"; }

show_help() {
    cat << EOF
${CYAN}Mobile 应用部署脚本${NC}

用法: $0 <命令> [选项]

${GREEN}命令:${NC}
    docker:build    构建 Docker 镜像
    pm2:start       使用 PM2 启动
    pm2:stop        停止 PM2 进程
    pm2:logs        查看 PM2 日志

${GREEN}选项:${NC}
    --tag TAG       镜像标签 (默认: latest)
    --push          构建后推送镜像

${GREEN}示例:${NC}
    $0 docker:build              # 构建镜像
    $0 docker:build --tag v1.0.0 # 指定版本
    $0 pm2:start                 # PM2 启动
EOF
}

cmd_docker_build() {
    log_step "构建 Mobile Docker 镜像"
    
    local image_name="seed-mobile"
    local tag="${TAG:-latest}"
    
    docker build \
        -f "$APP_DIR/Dockerfile" \
        -t "$image_name:$tag" \
        -t "$image_name:latest" \
        --build-arg BUILD_DATE="$(date -u +'%Y-%m-%dT%H:%M:%SZ')" \
        "$PROJECT_ROOT"
    
    log_success "镜像构建完成: $image_name:$tag"
    
    if [ "$PUSH" = true ]; then
        docker push "$image_name:$tag"
        docker push "$image_name:latest"
        log_success "镜像推送完成"
    fi
}

cmd_pm2_start() {
    log_step "使用 PM2 启动 Mobile"
    
    if ! command -v pm2 &>/dev/null; then
        log_error "PM2 未安装"
        exit 1
    fi
    
    cd "$APP_DIR"
    pm2 start ecosystem.config.cjs --update-env
    pm2 save
    log_success "Mobile 已启动"
}

cmd_pm2_stop() {
    pm2 stop mobile 2>/dev/null || log_warn "mobile 可能未运行"
}

cmd_pm2_logs() {
    pm2 logs mobile
}

# 解析参数
COMMAND=""
TAG="latest"
PUSH=false

while [[ $# -gt 0 ]]; do
    case $1 in
        docker:build|pm2:start|pm2:stop|pm2:logs)
            COMMAND="$1"
            shift
            ;;
        --tag)
            TAG="$2"
            shift 2
            ;;
        --push)
            PUSH=true
            shift
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo "未知选项: $1"
            show_help
            exit 1
            ;;
    esac
done

case "$COMMAND" in
    docker:build) cmd_docker_build ;;
    pm2:start)    cmd_pm2_start ;;
    pm2:stop)     cmd_pm2_stop ;;
    pm2:logs)     cmd_pm2_logs ;;
    *)            show_help ;;
esac
