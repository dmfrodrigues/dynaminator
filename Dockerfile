FROM php:7.2-apache AS dev

RUN apt-get update
RUN apt-get install -y \
    cmake \
    git

## Install Catch2
WORKDIR /tmp/
RUN git clone https://github.com/catchorg/Catch2.git
WORKDIR /tmp/Catch2/
RUN cmake -Bbuild -H. -DBUILD_TESTING=OFF
RUN cmake --build build/ --target install
RUN rm -rf /tmp/Catch2/

## Enable CGI
RUN a2enmod cgi

## Install app
COPY app /app
WORKDIR /app
RUN mkdir build
WORKDIR /app/build
RUN cmake ..
RUN cmake --build .
RUN cmake --build . --target install

FROM dev AS prod

COPY web/html /var/www/html
