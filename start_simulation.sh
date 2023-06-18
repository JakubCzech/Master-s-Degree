#!/bin/bash
xhost +
docker compose up -d simulation && docker compose logs -f simulation
docker compose kill simulation
docker compose rm -f simulation
