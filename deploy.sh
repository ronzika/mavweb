#!/bin/bash
set -e

# Build the Docker image
docker build -t mavweb .

docker stop mavweb 2>/dev/null || true
docker rm mavweb 2>/dev/null || true

docker run -d --name mavweb -p 8500:8500 -p 14551:14551/udp --env-file .env mavweb
