# syntax=docker/dockerfile:1.3

# ===========================================
# Stage 1: Install dependencies
# ===========================================
FROM node:lts-alpine AS deps
WORKDIR /app

COPY package.json pnpm-lock.yaml pnpm-workspace.yaml nx.json ./
COPY packages/tsconfig/package.json ./packages/tsconfig/
COPY apps/docs/package.json ./apps/docs/

RUN npm install -g pnpm && \
    pnpm install --frozen-lockfile --prefer-offline

# ===========================================
# Stage 2: Build application
# ===========================================
FROM node:lts-alpine AS builder
WORKDIR /app

COPY --from=deps /app/node_modules ./node_modules
COPY --from=deps /app/packages/*/node_modules ./packages/
COPY --from=deps /app/apps/docs/node_modules ./apps/docs/

COPY packages/ ./packages/
COPY apps/docs/ ./apps/docs/
COPY package.json pnpm-lock.yaml pnpm-workspace.yaml nx.json tsconfig*.json ./

RUN npm install -g pnpm && \
    pnpm --filter @seed/docs build

# ===========================================
# Stage 3: Production deployment
# ===========================================
FROM nginx:stable-alpine AS deploy

COPY --from=builder /app/apps/docs/dist /usr/share/nginx/html

# 简单的 nginx 配置
RUN echo 'server { \
    listen 80; \
    root /usr/share/nginx/html; \
    index index.html; \
    location /health { return 200 "ok"; add_header Content-Type text/plain; } \
    location / { \
        try_files $uri $uri/ /index.html; \
    } \
    gzip on; \
    gzip_types text/plain text/css application/json application/javascript text/xml application/xml; \
}' > /etc/nginx/conf.d/default.conf

HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
  CMD wget --no-verbose --tries=1 --spider http://localhost/health || exit 1

EXPOSE 80
CMD ["nginx", "-g", "daemon off;"]

# Build: docker build -f apps/docs/Dockerfile -t seed-docs:latest .
# Run: docker run -d --name seed-docs -p 4000:80 seed-docs:latest
