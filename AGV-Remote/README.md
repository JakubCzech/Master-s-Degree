# AGV-Remote


## For use only one tool:

```bash
export TOOL=[rviz,rqt,plot_juggler]
```

## Run locally:

```bash
./run_local.sh
```

## Run from web:

```bash
docker run -it --rm --privileged \
--env="DISPLAY=$DISPLAY" --env="TOOL=$TOOL" \
 --network=host ghcr.io/jakubczech/agv-remote:main
```