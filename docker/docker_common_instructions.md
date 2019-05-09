# Docker common instructions

### 1. Generating container from image

```shell
$ docker run --name jaden-work -p 45901:5901 -v ~/sharefolder:/home/ros/sharefolder -it jaden/work /bin/bash
```

### 2. Executing container

```shell
$ docker exec -it jaden-ros /bin/bash
```

### 3.Generating container with CUDA

```shell
docker run --runtime=nvidia --name jaden-work -p 49152:5901 -v ~/sharefolder:/home/ros/sharefolder -it jaden/ros-vnc-ubuntu:kinetic /bin/bash
```

### 4.TensorFlow example

First of all, check your Nvidia driver version:

```shell
$ nvidia-smi
```

no need CUDA tool but Nvidia driver version should match CUDA version. Compatibility table:https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/index.html

```shell
$ docker run --runtime=nvidia --name jaden-tensorflow -p 49152:8888 -p49153:22 -p 49154:6006 -p 49155:5901 -it tensorflow/tensorflow:1.12.0-gpu-py3 bash
```

`--runtime=nvidia` (nvidia docker v2) run with GPU, `-p host:container` 

