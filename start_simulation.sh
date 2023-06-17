#!/bin/bash
docker compose up -d simulation navigation && docker compose logs -f simulation navigation
docker compose kill simulation navigation
docker compose rm -f