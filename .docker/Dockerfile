FROM node:lts AS base
# WORKDIR /app

# FROM base AS deps
# COPY package.json pnpm-lock.yaml .npmrc ./
# RUN npm install -g pnpm
# RUN pnpm i


# FROM base AS build
# COPY --from=deps /app/node_modules ./node_modules
# COPY . .
# RUN npm run build

# FROM nginx:stable-alpine AS deploy
# COPY --from=build /app/dist /usr/share/nginx/html
# COPY ./nginx.conf /etc/nginx/nginx.conf

# EXPOSE 80

# CMD ["nginx", "-g", "daemon off;"]

# docker build -f ./Dockerfile -t xxxxx:v1 .

# docker run -d -p 4173:80 xxxxx:v1
