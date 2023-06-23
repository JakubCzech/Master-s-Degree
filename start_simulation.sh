#!/bin/bash
xhost +
docker compose -f compose_dev.yaml up -d simulation  && docker compose -f compose_dev.yaml logs -f simulation 
docker compose -f compose_dev.yaml kill simulation 
docker compose -f compose_dev.yaml rm -f simulation 
