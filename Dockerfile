FROM ubuntu:20.04 AS base

ENV TZ=Europe/Lisbon
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
RUN apt-get update
RUN apt-get install -y apache2
RUN apt-get install -y cmake
RUN apt-get install -y git
RUN apt-get install -y nlohmann-json3-dev
RUN apt-get install -y npm
RUN apt-get install -y python3
RUN apt-get install -y libwebsocketpp-dev libasio-dev

## Configure Apache2
RUN a2enmod rewrite
RUN a2enmod cgi
RUN a2enmod mime

## Redirect to swagger
WORKDIR /var/www/html
RUN ln -s /swagger/node_modules/swagger-ui-dist swagger-ui

COPY run.sh run.dev.sh /
RUN chmod +x /run.sh /run.dev.sh

FROM base as dev

FROM base AS prod

## Install web stuff
COPY web/config/000-default.conf /etc/apache2/sites-available/000-default.conf

## Install app
COPY app /app
RUN mkdir -p /tmp/app/build/
WORKDIR /tmp/app/build/
RUN cmake /app
RUN cmake --build . --target install
RUN rm -rf /tmp/app/

## Generate swagger docs
WORKDIR /app
RUN chmod 755 script/docs-swagger.py
RUN script/docs-swagger.py > /var/www/html/swagger.yaml
RUN chmod 755 /var/www/html/swagger.yaml

## Install swagger
COPY web/swagger /swagger
WORKDIR /swagger/
RUN npm install
COPY web/swagger/swagger-initializer.js /swagger/node_modules/swagger-ui-dist/swagger-initializer.js

## Cleanup as much as possible
RUN apt-get clean
RUN rm -rf /var/cache/apt/archives /var/lib/apt/lists/*

CMD /run.sh
