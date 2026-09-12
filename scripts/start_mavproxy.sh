#!/bin/bash

set -e

echo "======================================"
echo " Starting MAVProxy"
echo "======================================"

mavproxy.py \
    --master=tcp:127.0.0.1:5760 \
    --out=127.0.0.1:14550 \
    --out=127.0.0.1:14551 \
    --console

