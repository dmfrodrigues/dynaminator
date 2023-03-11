#!/bin/bash

cd /tmp/app/build
cmake /app
cmake --build . --target install -j 8

apache2-foreground
