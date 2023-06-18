#!/bin/bash
xhost +
docker compose up -d visualization && docker compose logs -f visualization
docker compose kill visualization
docker compose rm -f visualization
