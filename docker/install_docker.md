# Install docker on Ubuntu

### My environment:

- Distributor ID:	Ubuntu
  Description:	Ubuntu 16.04.5 LTS
  Release:	16.04
  Codename:	xenial

```shell
 $ lsb_release -a   #type this to check your ubuntu version
```



### Install steps:

#### 1.Set up the repository

Update the `apt` package index:

```
$ sudo apt-get update
```

Install packages to allow `apt` to use a repository over HTTPS:

```
$ sudo apt-get install \
    apt-transport-https \
    ca-certificates \
    curl \
    software-properties-common
```

Add Docker’s official GPG key:

```
$ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
```

Verify that you now have the key with the fingerprint
`9DC8 5822 9FC7 DD38 854A  E2D8 8D81 803C 0EBF CD88`, by searching for the
last 8 characters of the fingerprint.

```
$ sudo apt-key fingerprint 0EBFCD88
```

you will get:

```
pub   4096R/0EBFCD88 2017-02-22
      Key fingerprint = 9DC8 5822 9FC7 DD38 854A  E2D8 8D81 803C 0EBF CD88
uid                  Docker Release (CE deb) <docker@docker.com>
sub   4096R/F273FCD8 2017-02-22
```

for my environment amd64:

```
$ sudo add-apt-repository \
   "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
   $(lsb_release -cs) \
   stable"
```



#### 2.Install Docker CE

Update the `apt` package index.

```
$ sudo apt-get update
```

Install the *latest version* of Docker CE, or go to the next step to install a specific version:

```
$ sudo apt-get install docker-ce
```

To install a *specific version* of Docker CE, list the available versions in the repo, then select and install:

a. List the versions available in your repo:

```
$ apt-cache madison docker-ce
```

pick one version that you want to install from the list in command window.

for example:

```
docker-ce | 18.09.0~ce-0~ubuntu | https://download.docker.com/linux/ubuntu xenial/stable amd64 Packages
```

the <VERSION> is:

`18.03.0~ce-0~ubuntu`.

```
$ sudo apt-get install docker-ce=<VERSION>
```

The Docker daemon starts automatically.

Verify that Docker CE is installed correctly by running the `hello-world` image.

```
$ sudo docker run hello-world
```

See also: [using_docker_without_sudo](./using_docker_without_sudo.md)



#### 3.Docker for GPU

Install nvidia-docker:

```shell
# Add nvidia-docker library
$ curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | \
 sudo apt-key add -

$ distribution=$(. /etc/os-release;echo $ID$VERSION_ID)

$ curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
 sudo tee /etc/apt/sources.list.d/nvidia-docker.list

$ sudo apt-get update

# Install nvidia-docker2 and reload Docker daemon configuration
$ sudo apt-get install -y nvidia-docker2
$ sudo pkill -SIGHUP dockerd

# Test nvidia-smi command
docker run --runtime=nvidia --rm nvidia/cuda nvidia-smi
```