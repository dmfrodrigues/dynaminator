#!/bin/bash

set -e

cd /app
/swagger/docs-swagger.py > /var/www/html/swagger.yaml
chmod 755 /var/www/html/swagger.yaml

cd /swagger
npm install
rm node_modules/swagger-ui-dist/swagger-initializer.js
cp swagger-initializer.js node_modules/swagger-ui-dist/

cd /tmp/app/build
cmake /app -DCMAKE_BUILD_TYPE=Release
cmake --build . --target install -j 8

cd /data
dynaminator
