#!/bin/bash
docker compose up -d controller navigation servo && docker compose logs -f controller navigation servo
docker compose kill controller navigation servo
docker compose rm -f