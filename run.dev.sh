#!/bin/bash

set -e

cd /swagger
npm install
rm node_modules/swagger-ui-dist/swagger-initializer.js
cp swagger-initializer.js node_modules/swagger-ui-dist/

cd /app
script/docs-swagger.py > /var/www/html/swagger.yaml
chmod 755 /var/www/html/swagger.yaml

cd /tmp/app/build
cmake /app
cmake --build . --target install -j 8

cd /data
dynaminator 2>&1 | sed "s/^/[simulator] /" &

tail -f /var/log/apache2/error.log | sed "s/^/[apache:error] /" &
/usr/sbin/apache2ctl -DFOREGROUND | sed "s/^/[apache:acces] /"
