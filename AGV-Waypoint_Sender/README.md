# AGV-Navigation

## Run locally:

```bash
./run_local.sh
```

## Run from web:

```bash
docker run -it --rm --privileged \
 --network=host ghcr.io/jakubczech/agv-navigation:main
```

## Launch mapping:

```bash
export MAPPING=True
```

## Use simulation time:

```bash
export SIMULATION=True
```

## Save map (works only if you use mapping started from compose file):

```bash
./save_map.sh
```

This command also copy maps dir from container to host.

## Save map (works only if you use locally without compose):

```bash
./save_map_local.sh
```

This command also copy maps dir from container to host.