# Master-s-Degree

## 1. Introduction

This repository contains the code for the Master's Degree in Automation and Robotics at the Poznan University of Technology. 

## 2. Installation

### 2.1. Build docker images locally

```bash
./build_images.sh
```

### 2.2. Pull docker images from Docker Hub

```bash
docker compose pull
``` 

## 3. Usage

### 3.1. Run docker containers

#### Simulation:

```bash
export GAZEBO_GUI=1 # Default: blank (no GUI)
docker compose up -d simulation
```

#### Navigation:

```bash
export MAPPING=True # Default: False (no mapping)
export SIMULATION=True # Default: False (real robot)
docker compose up -d navigation
```
#### Actual robot (real robot):

```bash
docker compose up -d controller     
```

#### Servo driver (real robot):

```bash
docker compose up -d servo
```
#### Visualization:

```bash
export TOOL=[rviz|rqt|plot_juggler] # Default: blank (all tools)
docker compose up -d visualization
```