# syntax=docker/dockerfile:1.3

# ===========================================
# Stage 1: Install dependencies
# ===========================================
FROM node:lts-alpine AS deps
WORKDIR /app

# 仅复制依赖相关文件（利用 Docker 缓存层）
COPY package.json pnpm-lock.yaml pnpm-workspace.yaml nx.json ./
COPY packages/common/package.json ./packages/common/
COPY packages/http/package.json ./packages/http/
COPY packages/tsconfig/package.json ./packages/tsconfig/
COPY packages/types/package.json ./packages/types/
COPY packages/tailwind-config/package.json ./packages/tailwind-config/
COPY apps/admin/package.json ./apps/admin/

RUN npm install -g pnpm && \
    pnpm install --frozen-lockfile --prefer-offline

# ===========================================
# Stage 2: Build application
# ===========================================
FROM node:lts-alpine AS builder
WORKDIR /app

# 从 deps 阶段复制已安装的依赖
COPY --from=deps /app/node_modules ./node_modules
COPY --from=deps /app/packages/*/node_modules ./packages/

# 复制源代码
COPY packages/ ./packages/
COPY apps/admin/ ./apps/admin/
COPY package.json pnpm-lock.yaml pnpm-workspace.yaml nx.json tsconfig*.json ./

RUN npm install -g pnpm && \
    pnpm --filter @seed/admin build

# ===========================================
# Stage 3: Production deployment
# ===========================================
FROM nginx:stable-alpine AS deploy

COPY --from=builder /app/apps/admin/dist /usr/share/nginx/html
COPY apps/admin/nginx.conf /etc/nginx/nginx.conf

HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
  CMD wget --no-verbose --tries=1 --spider http://localhost/health || exit 1

EXPOSE 80
CMD ["nginx", "-g", "daemon off;"]

# Build command:
# docker build -f apps/admin/Dockerfile -t seed-admin:latest .
# Run command:
# docker run -d --name seed-admin -p 8080:80 seed-admin:latest
