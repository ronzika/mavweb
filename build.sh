#!/bin/bash
set -e

# Build the Docker image
docker build -t mavweb -f Dockerfile.

docker stop mavweb 2>/dev/null || true
docker rm mavweb 2>/dev/null || true

docker run -d --name mavweb  --network proxy -p 8500:8500 --env-file .env mavweb
docker network connect bridge mavweb || true
