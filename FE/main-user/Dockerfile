# Basic Image
FROM node:22.13.1-alpine

# Set Working Path
WORKDIR /app

# Copy Code
COPY ./express-server /app/express-server
COPY ./public /app/public
COPY ./src /app/src
COPY ./eslint.config.js /app/eslint.config.js
COPY ./index.html /app/index.html
COPY ./package.json /app/package.json
COPY ./package-lock.json /app/package-lock.json
COPY ./vite.config.js /app/vite.config.js

# Set Time
RUN apk add --no-cache chrony
RUN echo "server time.google.com iburst" > /etc/chrony/chrony.conf
CMD ["chronyd", "-d", "-s", "-f", "/etc/chorny/chorny.conf"]

# Set Webpages Requirements
RUN npm install

# Build Webpages
RUN npm run build
RUN mkdir ./express-server/public
RUN cp -r ./dist/* ./express-server/public

# Set Express Working Path
WORKDIR express-server

# Set Server Requirements
RUN npm install

# Set Run
EXPOSE 3000
CMD ["npm", "run", "start"]


