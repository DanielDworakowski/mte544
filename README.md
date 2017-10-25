# MTE544

# Install

Install docker

Install nvidia-docker

Clone this repo

```cd MTE544```

# Run

```xhost +local:root ```

```sudo docker build . -t mte544 ```

```sudo docker stop mte544_c; sudo docker rm mte544_c; sudo nvidia-docker run -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -v $(pwd)/workspace:/home/user/workspace --privileged --net=host --name mte544_c mte544```

- to run additional terminal

```sudo docker exec -it mte544_c bash```

- for lazy, to kill all docker

```sudo docker stop $(sudo docker ps -aq); sudo docker rm $(sudo docker ps -aq)```
