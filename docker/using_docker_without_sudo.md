# Using docker command without sudo

#### Create docker group and add specific user in the group:

```shell
$ sudo usermod -aG docker <your_username>
```

then Logout!

#### Check you permission 

```shell
$ docker run hello-world
```

done.