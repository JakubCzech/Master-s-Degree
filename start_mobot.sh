#!/bin/bash
docker compose -f compose_dev.yaml up -d controller navigation servo && docker compose logs -f controller navigation servo
docker compose -f compose_dev.yaml kill controller navigation servo
docker compose -f compose_dev.yaml rm -f