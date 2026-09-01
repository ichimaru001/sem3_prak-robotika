# Robotics TurtleBot3 - Modul 1 Development Environment

This repository contains a reproducible, Dockerized ROS 2 Humble development environment.

## Prerequisites
- Ubuntu 22.04
- Docker Engine and Docker Compose plugin

## Build and Run Instructions
Run the following commands to build the image and automatically start the talker and listener nodes:

1. `git clone <your-repository-url>`
2. `cd robotics-turtlebot3/docker`
3. `docker compose build`
4. `docker compose up`

To stop the nodes and tear down the environment, run:
`docker compose down`
