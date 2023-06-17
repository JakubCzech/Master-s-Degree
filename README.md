# Master-s-Degree

1. Pull all images:

```sh
docker compose pull
```

2. Start compose for simulation: 
 
```sh
./start_simulation.sh
```

3. Start compose for actual robot with servo driver: 
 
```sh
./start_mobot.sh
```

3. Kinco servo driver:
   
```sh
export SERVO_PORT=/dev/ttyUSB0
```

4. Enable gazebo gui for simulation:


```sh
export GAZEBO_GUI=1
```