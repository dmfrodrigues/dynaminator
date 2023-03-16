#!/bin/bash

set -e

cd /data
dynaminator &

/usr/sbin/apache2ctl -DFOREGROUND
