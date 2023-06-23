#!/bin/bash
xhost +
docker compose -f compose_dev.yaml up -d visualization && docker compose -f compose_dev.yaml logs -f visualization
docker compose -f compose_dev.yaml kill visualization
docker compose -f compose_dev.yaml rm -f visualization
