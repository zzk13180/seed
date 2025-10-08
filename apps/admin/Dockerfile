# syntax=docker/dockerfile:1.3

FROM node:lts-alpine AS builder
WORKDIR /app

COPY . .

RUN --mount=type=cache,target=/root/.pnpm-store \
  --mount=type=cache,target=/app/node_modules \
  npm install -g pnpm && \
  pnpm install --frozen-lockfile --prefer-offline && \
  pnpm --filter @seed/admin build

FROM nginx:stable-alpine AS deploy

COPY --from=builder /app/apps/admin/dist /usr/share/nginx/html
COPY apps/admin/nginx.conf /etc/nginx/nginx.conf

EXPOSE 80
CMD ["nginx", "-g", "daemon off;"]

# docker build -f .docker/Dockerfile.admin -t seed-admin:latest .
# docker run -d --name seed-admin -p 8080:80 seed-admin:latest
