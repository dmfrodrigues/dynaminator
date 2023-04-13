#!/bin/bash

set -e

cd /data
dynaminator 2>&1 | sed "s/^/[simulator] /" &

tail -f /var/log/apache2/error.log | sed "s/^/[apache:error] /" &
/usr/sbin/apache2ctl -DFOREGROUND | sed "s/^/[apache:acces] /"
