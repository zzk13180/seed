#!/bin/bash

# 检查 Node.js 是否安装，如果没有则自动使用 NodeSource 官方源安装
if ! command -v node &> /dev/null
then
    echo "未检测到 Node.js，正在下载并安装 Node.js v${NODE_VERSION:-20}..."
    NODE_VERSION="${NODE_VERSION:-20.0.0}"
        ARCH_RAW="$(uname -m)"
        case "$ARCH_RAW" in
            x86_64)
                ARCH="linux-x64" ;;
            aarch64|arm64)
                ARCH="linux-arm64" ;;
            armv7l|armv6l)
                ARCH="linux-armv7l" ;;
            *)
                echo "不支持的架构: $ARCH_RAW" ; exit 1 ;;
        esac
    INSTALL_DIR="$HOME/.local/node"
    mkdir -p "$INSTALL_DIR"
    curl -fsSL \
      "https://nodejs.org/dist/v$NODE_VERSION/node-v$NODE_VERSION-$ARCH.tar.xz" \
      -o /tmp/node.tar.xz
    tar -xJf /tmp/node.tar.xz -C "$INSTALL_DIR" --strip-components=1
    rm /tmp/node.tar.xz
    export PATH="$INSTALL_DIR/bin:$PATH"
    # 持久化到当前 shell 配置（可选）
    SHELL_RC=""
    if [ -n "$ZSH_VERSION" ]; then
      SHELL_RC="$HOME/.zshrc"
    elif [ -n "$BASH_VERSION" ]; then
      SHELL_RC="$HOME/.bashrc"
    else
      SHELL_RC="$HOME/.profile"
    fi
    if ! grep -q 'export PATH=.*\.local/node/bin' "$SHELL_RC"; then
      echo 'export PATH="$HOME/.local/node/bin:$PATH"' >> "$SHELL_RC"
    fi

    # 验证安装
    if ! command -v node &> /dev/null; then
        echo "错误：Node.js 安装失败。"
        exit 1
    fi
    echo "Node.js 安装完成，版本：$(node --version)"
fi

# 自动检测并设置 npm 全局安装目录到用户目录，避免 sudo
NPM_PREFIX=$(npm config get prefix)
if [[ "$NPM_PREFIX" == "/usr" || "$NPM_PREFIX" == "/usr/local" || "$NPM_PREFIX" == "/usr/lib/node_modules" ]]; then
    echo "检测到 npm 全局安装目录为系统目录，正在切换到用户目录 ~/.npm-global..."
    mkdir -p ~/.npm-global
    npm config set prefix ~/.npm-global
    # 检查 shell 类型并自动加入 PATH
    SHELL_RC=""
    if [ -n "$ZSH_VERSION" ]; then
        SHELL_RC="$HOME/.zshrc"
    elif [ -n "$BASH_VERSION" ]; then
        SHELL_RC="$HOME/.bashrc"
    else
        SHELL_RC="$HOME/.profile"
    fi
    if ! grep -q 'export PATH=~/.npm-global/bin:$PATH' "$SHELL_RC"; then
        echo 'export PATH=~/.npm-global/bin:$PATH' >> "$SHELL_RC"
        export PATH=~/.npm-global/bin:$PATH
        source "$SHELL_RC"
        echo "已将 ~/.npm-global/bin 加入 PATH 并 'source $SHELL_RC'。"
    fi
fi

# 检查并自动安装 pnpm
if ! command -v pnpm &> /dev/null
then
    echo "未检测到 pnpm，正在全局安装 pnpm……"
    npm install -g pnpm
    if [ $? -ne 0 ]; then
        echo "错误：pnpm 安装失败，请检查 npm 配置或网络。"
        exit 1
    fi
    echo "pnpm 安装完成。"
fi

# 安装项目依赖
echo "正在安装项目依赖..."
pnpm install
if [ $? -ne 0 ]; then
    echo "错误：依赖安装失败。"
    exit 1
fi

echo "正在打包 server-static..."
pnpm run --filter server-static build
if [ $? -ne 0 ]; then
    echo "错误：server-static 打包失败。"
    exit 1
fi

if [ -f ./apps/server-static/.env ]; then
    echo "正在复制 server-static 的 .env 文件到 dist 目录..."
    cp ./apps/server-static/.env ./apps/server-static/dist/.env
    if [ $? -ne 0 ]; then
        echo "错误：复制 .env 文件失败。"
        exit 1
    fi
else
    echo "警告：.env 文件不存在，跳过复制。"
fi

# TODO:将产出移动到 server-static 的静态目录

# 询问用户是否继续运行 pm2 部署
read -p "是否继续运行 pm2 部署？(y/n): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "已取消 pm2 部署，脚本结束。"
    exit 0
fi

# 检查并自动安装 pm2
if ! command -v pm2 &> /dev/null
then
    echo "未检测到 pm2，正在全局安装 pm2……"
    npm install -g pm2
    if [ $? -ne 0 ]; then
        echo "错误：pm2 安装失败，请检查 npm 配置或网络。"
        exit 1
    fi
    echo "pm2 安装完成。"
fi

ECOSYSTEM_CONFIG="ecosystem.config.cjs"

if [ ! -f "$ECOSYSTEM_CONFIG" ]; then
    echo "错误：未找到 $ECOSYSTEM_CONFIG"
    exit 1
fi

echo "正在使用 pm2 部署（ecosystem.config.cjs）..."

# 启动或重载 pm2 进程
pm2 startOrReload "$ECOSYSTEM_CONFIG"

if [ $? -ne 0 ]; then
    echo "错误：pm2 操作失败。"
    exit 1
fi

# 询问用户是否设置 pm2 开机自启
SERVICE_NAME="pm2-$USER.service"
if ! systemctl is-enabled $SERVICE_NAME &> /dev/null; then
    echo "正在配置 pm2 开机自启并保存进程列表…"
    pm2 save
    pm2 startup
    echo "请复制上面的命令到终端执行以完成 pm2 开机自启配置。"
else
    echo "pm2 开机自启已配置。"
fi

echo "pm2 日志文件位于 ~/.pm2/logs/ 目录，可使用 'pm2 logs \$PM2_APP_NAME' 查看实时日志。"
