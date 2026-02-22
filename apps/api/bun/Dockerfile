# ========================================
# Build Stage
# ========================================
FROM oven/bun:latest AS builder

WORKDIR /app

# Copy workspace root files
COPY package.json pnpm-workspace.yaml pnpm-lock.yaml ./

# Copy workspace packages needed by server
COPY packages/contracts/ ./packages/contracts/
COPY packages/kit/ ./packages/kit/
COPY packages/db/ ./packages/db/
COPY packages/services/ ./packages/services/
COPY packages/libs/types/ ./packages/libs/types/
COPY packages/configs/ ./packages/configs/
COPY apps/api/bun/ ./apps/api/bun/

# Install dependencies
RUN bun install --frozen-lockfile

# ========================================
# Production Stage
# ========================================
FROM oven/bun:latest AS runner

WORKDIR /app
ENV NODE_ENV=production

# Copy everything from builder (Bun runs TS directly, no build step needed)
COPY --from=builder /app .

WORKDIR /app/apps/api/bun

# Health check
HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
  CMD bun -e "fetch('http://localhost:3003/api/health').then(r => r.ok ? process.exit(0) : process.exit(1)).catch(() => process.exit(1))"

EXPOSE 3003
CMD ["bun", "run", "src/main.ts"]

