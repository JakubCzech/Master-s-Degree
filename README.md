# Master-s-Degree


## Update images:

```sh
docker compose pull
```

## Configure:

Kinco servo driver port:
   
```sh
export SERVO_PORT=/dev/ttyUSB0
```

Enable gazebo gui for simulation:


```sh
export GAZEBO_GUI=1
```

Select only one tool for visualization:

```bash
export TOOL=[rviz,rqt,plot_juggler]
```

Launch mapping:

```bash
export MAPPING=True
```

Use simulation time:

```bash
export SIMULATION=True
```

## Start

### Simulation

This command start simulation with navigation/mapping and visualization tools:

```sh
./start_simulation.sh
```

### Actual robot

This command start navigation/mapping, servo driver and agv controller:

```sh
./start_mobot.sh
```

On remote pc start visualization tools:

```sh
./start_visualization.sh
```

### Others

Save map:

```bash
./save_map.sh
```

This command also copy maps dir from container to host.
