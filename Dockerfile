FROM ubuntu:20.04 AS base

ENV TZ=Europe/Lisbon
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
RUN apt-get update
RUN apt-get install -y nlohmann-json3-dev
RUN apt-get install -y libwebsocketpp-dev libasio-dev

## Redirect to swagger
WORKDIR /var/www/html
RUN ln -s /swagger/node_modules/swagger-ui-dist swagger-ui

COPY run.sh run.dev.sh /
RUN chmod +x /run.sh /run.dev.sh

FROM base AS base-with-makers

RUN apt-get install -y cmake
RUN apt-get install -y npm
RUN apt-get install -y python3

FROM base-with-makers as dev

FROM base-with-makers AS prod-build

## Install app
COPY app /app
RUN mkdir -p /tmp/app/build/
WORKDIR /tmp/app/build/
RUN cmake /app -DCMAKE_BUILD_TYPE=Release
RUN cmake --build . --target install
RUN rm -rf /tmp/app/

## Install swagger and generate docs
COPY web/swagger /swagger
WORKDIR /swagger/
RUN chmod 755 docs-swagger.py
RUN ./docs-swagger.py > /var/www/html/swagger.yaml
RUN chmod 755 /var/www/html/swagger.yaml
RUN npm install
COPY web/swagger/swagger-initializer.js /swagger/node_modules/swagger-ui-dist/swagger-initializer.js

FROM base as prod

COPY --from=prod-build /var/www/html/swagger.yaml /var/www/html/swagger.yaml
COPY --from=prod-build /swagger/node_modules/swagger-ui-dist /swagger/node_modules/swagger-ui-dist
COPY --from=prod-build /usr/local/bin/dynaminator /usr/local/bin/dynaminator
COPY --from=prod-build /usr/lib/cgi-bin/script.cgi /usr/lib/cgi-bin/script.cgi

## Cleanup as much as possible
RUN apt-get clean
RUN rm -rf /var/cache/apt/archives /var/lib/apt/lists/*

CMD /run.sh
