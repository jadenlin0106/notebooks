# Docker common instructions

### 1. Generating container from image

```shell
$ docker run --name jaden-work -p 45901:5901 -v ~/sharefolder:/home/ros/sharefolder -it jaden/work /bin/bash
```

### 2. Executing container

```shell
$ docker exec -it jaden-ros /bin/bash
```

