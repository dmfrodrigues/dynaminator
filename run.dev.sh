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

apache2-foreground
