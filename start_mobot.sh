#!/bin/bash
docker compose up -d controller navigation servo && docker compose logs -f controller navigation servo
docker compose kill
docker compose rm -f
