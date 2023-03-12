FROM php:7.2-apache AS dev

RUN apt-get update
RUN apt-get install -y \
    cmake \
    git \
    npm \
    python3

## Configure Apache2
RUN a2enmod rewrite
RUN a2enmod cgi
RUN a2enmod mime

## Install json
WORKDIR /tmp/
RUN git clone https://github.com/nlohmann/json.git
WORKDIR /tmp/json/
RUN cmake -Bbuild -H. -DBUILD_TESTING=OFF
RUN cmake --build build/ --target install
RUN rm -rf /tmp/json/

## Install Catch2
WORKDIR /tmp/
RUN git clone https://github.com/catchorg/Catch2.git
WORKDIR /tmp/Catch2/
RUN cmake -Bbuild -H. -DBUILD_TESTING=OFF
RUN cmake --build build/ --target install
RUN rm -rf /tmp/Catch2/

## Redirect to swagger
WORKDIR /var/www/html
RUN ln -s /swagger/node_modules/swagger-ui-dist swagger-ui

COPY run.dev.sh /
RUN chmod +x /run.dev.sh

FROM dev AS prod

## Install web stuff
COPY web/config/.htaccess /var/www/html/
# TODO: Remove this line, since swagger.yaml will be dynamically generated
COPY web/html/swagger.yaml /var/www/html/

## Install app
COPY app /app
RUN mkdir -p /tmp/app/build/
WORKDIR /tmp/app/build/
RUN cmake /app
RUN cmake --build . --target install
RUN rm -rf /tmp/app/

## Generate swagger docs
WORKDIR /app
RUN script/docs-swagger.py > /var/www/html/swagger.yaml && chmod 755 /var/www/html/swagger.yaml

## Install swagger
COPY web/swagger /swagger
WORKDIR /swagger/
RUN npm install
COPY web/swagger/swagger-initializer.js /swagger/node_modules/swagger-ui-dist/swagger-initializer.js

COPY web/html /var/www/html
