#!/bin/bash
docker compose -f compose_dev.yaml rm -f
docker compose -f compose_dev.yaml up -d controller navigation && docker compose logs -f  navigation controller |grep -e error -e warn -e fatal
docker compose -f compose_dev.yaml kill 
docker compose -f compose_dev.yaml rm -f
