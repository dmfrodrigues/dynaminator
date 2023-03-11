FROM php:7.2-apache AS dev

RUN apt-get update
RUN apt-get install -y \
    cmake \
    git

## Configure Apache2
RUN a2enmod rewrite
RUN a2enmod cgi

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

COPY run.dev.sh /
RUN chmod +x /run.dev.sh

FROM dev AS prod

## Install app
COPY app /app
RUN mkdir -p /tmp/app/build/
WORKDIR /tmp/app/build/
RUN cmake /app
RUN cmake --build . --target install
RUN rm -rf /tmp/app/

COPY web/html /var/www/html
