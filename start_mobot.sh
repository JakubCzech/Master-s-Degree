#!/bin/bash
docker compose -f compose_dev.yaml rm
docker compose -f compose_dev.yaml up -d controller navigation && docker compose logs -f  navigation controller
docker compose -f compose_dev.yaml kill controller navigation 
docker compose -f compose_dev.yaml rm -f
