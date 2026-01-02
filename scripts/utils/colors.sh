#!/bin/bash
# ============================================================
# 通用工具函数库
# 用途：提供所有脚本共用的辅助函数
# ============================================================

# 防止重复加载
if [ -n "${_COLORS_SH_LOADED:-}" ]; then
    return 0 2>/dev/null || exit 0
fi
readonly _COLORS_SH_LOADED=1

# ============================================================
# 颜色定义
# ============================================================
readonly RED='\033[0;31m'
readonly GREEN='\033[0;32m'
readonly YELLOW='\033[1;33m'
readonly BLUE='\033[0;34m'
readonly CYAN='\033[0;36m'
readonly MAGENTA='\033[0;35m'
readonly BOLD='\033[1m'
readonly DIM='\033[2m'
readonly NC='\033[0m' # No Color

# ============================================================
# 日志函数
# ============================================================
log_info()    { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn()    { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error()   { echo -e "${RED}[ERROR]${NC} $1" >&2; }
log_step()    { echo -e "${BLUE}[STEP]${NC} $1"; }
log_success() { echo -e "${GREEN}[✓]${NC} $1"; }
log_debug()   { [ "${DEBUG:-}" = "true" ] && echo -e "${DIM}[DEBUG]${NC} $1"; }

# ============================================================
# UI 函数
# ============================================================

# 打印分隔线
print_separator() {
    echo -e "${CYAN}============================================${NC}"
}

# 打印标题
print_header() {
    echo ""
    print_separator
    echo -e "${CYAN}${BOLD}  $1${NC}"
    print_separator
    echo ""
}

# 确认操作（默认 N）
confirm() {
    local message="${1:-是否继续？}"
    local default="${2:-n}"
    local prompt
    
    if [ "$default" = "y" ]; then
        prompt="$message [Y/n]: "
    else
        prompt="$message [y/N]: "
    fi
    
    read -p "$prompt" -n 1 -r
    echo ""
    
    if [ -z "$REPLY" ]; then
        [ "$default" = "y" ] && return 0 || return 1
    fi
    
    [[ $REPLY =~ ^[Yy]$ ]]
}

# 确认操作（默认 Y）
confirm_yes() {
    confirm "$1" "y"
}

# 进度条（简单版本）
show_spinner() {
    local pid=$1
    local delay=0.1
    local spinstr='|/-\\'
    while ps -p "$pid" > /dev/null 2>&1; do
        local temp=${spinstr#?}
        printf " [%c]  " "$spinstr"
        spinstr=$temp${spinstr%"$temp"}
        sleep $delay
        printf "\b\b\b\b\b\b"
    done
    printf "    \b\b\b\b"
}

# ============================================================
# 工具函数
# ============================================================

# 检查命令是否存在
command_exists() {
    command -v "$1" &> /dev/null
}

# 检查多个命令是否存在
check_dependencies() {
    local missing=()
    for cmd in "$@"; do
        if ! command_exists "$cmd"; then
            missing+=("$cmd")
        fi
    done
    
    if [ ${#missing[@]} -gt 0 ]; then
        log_error "缺少依赖: ${missing[*]}"
        return 1
    fi
    return 0
}

# 获取脚本所在目录（支持符号链接）
get_script_dir() {
    cd "$(dirname "${BASH_SOURCE[1]}")" && pwd
}

# 获取项目根目录
get_project_root() {
    local script_dir="$(get_script_dir)"
    cd "$script_dir/.." && pwd
}

# 获取 shell 配置文件路径
get_shell_rc() {
    if [ -n "${ZSH_VERSION:-}" ]; then
        echo "$HOME/.zshrc"
    elif [ -n "${BASH_VERSION:-}" ]; then
        echo "$HOME/.bashrc"
    else
        echo "$HOME/.profile"
    fi
}

# 添加路径到 PATH（避免重复）
add_to_path() {
    local dir="$1"
    local shell_rc="$(get_shell_rc)"
    
    # 当前 session 添加
    if [[ ":$PATH:" != *":$dir:"* ]]; then
        export PATH="$dir:$PATH"
    fi
    
    # 持久化到 shell 配置
    if ! grep -q "$dir" "$shell_rc" 2>/dev/null; then
        echo "export PATH=\"$dir:\$PATH\"" >> "$shell_rc"
        log_info "已添加 $dir 到 $shell_rc"
    fi
}

# ============================================================
# 版本比较函数
# ============================================================

# 比较版本号 (v1.2.3 格式)
# 返回: 0=相等, 1=第一个大, 2=第二个大
version_compare() {
    local v1="${1#v}"
    local v2="${2#v}"
    
    if [ "$v1" = "$v2" ]; then
        return 0
    fi
    
    local IFS=.
    local i v1_parts=($v1) v2_parts=($v2)
    
    for ((i=0; i<${#v1_parts[@]} || i<${#v2_parts[@]}; i++)); do
        local v1_part=${v1_parts[i]:-0}
        local v2_part=${v2_parts[i]:-0}
        
        if ((v1_part > v2_part)); then
            return 1
        elif ((v1_part < v2_part)); then
            return 2
        fi
    done
    
    return 0
}

# 检查版本是否满足最低要求
version_gte() {
    local current="$1"
    local required="$2"
    version_compare "$current" "$required"
    [ $? -ne 2 ]
}

# ============================================================
# 网络工具
# ============================================================

# 检查端口是否被占用
check_port() {
    local port="$1"
    if command_exists lsof; then
        lsof -Pi :"$port" -sTCP:LISTEN -t >/dev/null 2>&1
    elif command_exists ss; then
        ss -tuln | grep -q ":$port "
    elif command_exists netstat; then
        netstat -tuln | grep -q ":$port "
    else
        return 1
    fi
}

# 获取可用端口
get_available_port() {
    local start_port="${1:-3000}"
    local port=$start_port
    
    while check_port "$port"; do
        ((port++))
        if [ $port -gt 65535 ]; then
            log_error "无法找到可用端口"
            return 1
        fi
    done
    
    echo "$port"
}

# 等待服务启动
wait_for_service() {
    local host="$1"
    local port="$2"
    local timeout="${3:-30}"
    local count=0
    
    log_info "等待服务 $host:$port 启动..."
    
    while ! nc -z "$host" "$port" 2>/dev/null; do
        sleep 1
        count=$((count + 1))
        if [ $count -ge $timeout ]; then
            log_error "服务 $host:$port 启动超时 (${timeout}s)"
            return 1
        fi
    done
    
    log_success "服务 $host:$port 已就绪"
    return 0
}

# HTTP 健康检查
http_health_check() {
    local url="$1"
    local timeout="${2:-5}"
    
    if command_exists curl; then
        curl -sf --connect-timeout "$timeout" "$url" >/dev/null 2>&1
    elif command_exists wget; then
        wget -q --timeout="$timeout" -O /dev/null "$url" 2>/dev/null
    else
        log_warn "无法执行健康检查 (curl/wget 未安装)"
        return 1
    fi
}

# ============================================================
# 文件工具
# ============================================================

# 安全删除（移到回收站）
safe_rm() {
    local target="$1"
    local trash_dir="${HOME}/.Trash"
    
    if [ ! -e "$target" ]; then
        return 0
    fi
    
    if [ -d "$trash_dir" ]; then
        mv "$target" "$trash_dir/$(basename "$target").$(date +%s)"
    else
        rm -rf "$target"
    fi
}

# 获取文件大小（人类可读）
human_size() {
    local bytes="$1"
    if [ "$bytes" -lt 1024 ]; then
        echo "${bytes}B"
    elif [ "$bytes" -lt 1048576 ]; then
        echo "$((bytes / 1024))KB"
    elif [ "$bytes" -lt 1073741824 ]; then
        echo "$((bytes / 1048576))MB"
    else
        echo "$((bytes / 1073741824))GB"
    fi
}
