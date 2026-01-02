FROM node:lts-alpine AS builder

WORKDIR /app

# Enable pnpm
RUN corepack enable && corepack prepare pnpm@latest --activate

# Copy source
COPY . .

# Install dependencies and build
RUN pnpm install --frozen-lockfile
RUN pnpm --filter @seed/server build

# Use pnpm deploy to generate a pruned production package
# This converts symlinked monorepo packages into physical files in node_modules
RUN pnpm --filter @seed/server --prod deploy pruned

# ========================================
# Production Stage
# ========================================
FROM node:lts-alpine AS runner

WORKDIR /app
ENV NODE_ENV=production

# Copy the pruned lockfile and install prod dependencies (if deploy didn't fully install)
# Or simply copy the deployed folder which includes node_modules
COPY --from=builder /app/pruned .

# Copy the built dist artifacts (overlaying the source code)
COPY --from=builder /app/apps/server/dist ./dist

# Health check
HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
  CMD wget --no-verbose --tries=1 --spider http://localhost:3003/api/health || exit 1

EXPOSE 3003
CMD ["node", "dist/main"]

