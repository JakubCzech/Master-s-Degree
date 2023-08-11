#!/bin/bash
xhost +
docker compose -f compose_dev.yaml up -d simulation navigation  && docker compose -f compose_dev.yaml logs -f simulation navigation
docker compose -f compose_dev.yaml kill simulation navigation
docker compose -f compose_dev.yaml rm -f simulation navigation
