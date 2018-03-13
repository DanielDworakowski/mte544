# MTE544

C++ implementation for turtlebot control, occupancy grid creation, motion planning and state estimation. 

# Install

Install docker

Install nvidia-docker

Clone this repo

```cd MTE544```

# Run

```xhost +local:root ```

```docker build . -t mte544 ```

```docker stop mte544_c; docker rm mte544_c; nvidia-docker run -it --ipc=host --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -v $(pwd)/workspace:/home/user/workspace --privileged --net=host --name mte544_c mte544

```

- to run additional terminal

```docker exec -it mte544_c bash```

- for lazy, to kill all docker

```docker stop $(docker ps -aq); docker rm $(docker ps -aq)```
