# syntax=docker/dockerfile:1.3

# ===========================================
# Stage 1: Install dependencies
# ===========================================
FROM node:lts-alpine AS deps
WORKDIR /app

COPY package.json pnpm-lock.yaml pnpm-workspace.yaml nx.json ./
COPY packages/common/package.json ./packages/common/
COPY packages/http/package.json ./packages/http/
COPY packages/tsconfig/package.json ./packages/tsconfig/
COPY packages/types/package.json ./packages/types/
COPY packages/tailwind-config/package.json ./packages/tailwind-config/
COPY apps/mobile/package.json ./apps/mobile/

RUN npm install -g pnpm && \
    pnpm install --frozen-lockfile --prefer-offline

# ===========================================
# Stage 2: Build application
# ===========================================
FROM node:lts-alpine AS builder
WORKDIR /app

COPY --from=deps /app/node_modules ./node_modules
COPY --from=deps /app/packages/*/node_modules ./packages/
COPY --from=deps /app/apps/mobile/node_modules ./apps/mobile/

COPY packages/ ./packages/
COPY apps/mobile/ ./apps/mobile/
COPY package.json pnpm-lock.yaml pnpm-workspace.yaml nx.json tsconfig*.json ./

RUN npm install -g pnpm && \
    pnpm --filter @seed/mobile build:prod

# ===========================================
# Stage 3: Production deployment
# ===========================================
FROM nginx:stable-alpine AS deploy

COPY --from=builder /app/apps/mobile/dist /usr/share/nginx/html

RUN echo 'server { \
    listen 80; \
    root /usr/share/nginx/html; \
    index index.html; \
    location /health { return 200 "ok"; add_header Content-Type text/plain; } \
    location / { try_files $uri $uri/ /index.html; } \
    gzip on; \
    gzip_types text/plain text/css application/json application/javascript text/xml application/xml; \
}' > /etc/nginx/conf.d/default.conf

HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
  CMD wget --no-verbose --tries=1 --spider http://localhost/health || exit 1

EXPOSE 80
CMD ["nginx", "-g", "daemon off;"]

# Build: docker build -f apps/mobile/Dockerfile -t seed-mobile:latest .
# Run: docker run -d --name seed-mobile -p 3002:80 seed-mobile:latest
