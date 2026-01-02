#!/bin/bash
set -euo pipefail

# ============================================================
# Admin 前端 Docker 构建脚本
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJECT_ROOT="$(cd "$APP_DIR/../.." && pwd)"

# 颜色
GREEN='\033[0;32m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

log_info()    { echo -e "${GREEN}[INFO]${NC} $1"; }
log_step()    { echo -e "${BLUE}[STEP]${NC} $1"; }
log_success() { echo -e "${GREEN}[✓]${NC} $1"; }

show_help() {
    cat << EOF
${CYAN}Admin 前端 Docker 构建脚本${NC}

用法: $0 [选项]

${GREEN}选项:${NC}
    --tag TAG       镜像标签 (默认: latest)
    --push          构建后推送镜像
    --no-cache      不使用缓存构建
    -h, --help      显示帮助

${GREEN}示例:${NC}
    $0                      # 构建 latest 镜像
    $0 --tag v1.0.0         # 构建指定版本
    $0 --tag v1.0.0 --push  # 构建并推送
EOF
}

# 解析参数
TAG="latest"
PUSH=false
NO_CACHE=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --tag)      TAG="$2"; shift 2 ;;
        --push)     PUSH=true; shift ;;
        --no-cache) NO_CACHE="--no-cache"; shift ;;
        -h|--help)  show_help; exit 0 ;;
        *)          echo "未知选项: $1"; show_help; exit 1 ;;
    esac
done

main() {
    log_step "构建 Admin 前端 Docker 镜像"
    
    local image_name="seed-admin"
    
    log_info "标签: $TAG"
    
    docker build \
        -f "$APP_DIR/Dockerfile" \
        -t "$image_name:$TAG" \
        -t "$image_name:latest" \
        --build-arg BUILD_DATE="$(date -u +'%Y-%m-%dT%H:%M:%SZ')" \
        --build-arg VCS_REF="$(git rev-parse --short HEAD 2>/dev/null || echo 'unknown')" \
        $NO_CACHE \
        "$PROJECT_ROOT"
    
    log_success "镜像构建完成: $image_name:$TAG"
    
    if [ "$PUSH" = true ]; then
        log_step "推送镜像"
        docker push "$image_name:$TAG"
        docker push "$image_name:latest"
        log_success "镜像推送完成"
    fi
}

main
