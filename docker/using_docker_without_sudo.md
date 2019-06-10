# Using docker command without sudo

#### Create docker group and add specific user in the group:

```shell
$ sudo usermod -aG docker [Your_username]
# Or, for current user
$ sudo usermod -aG docker ${USER}
```

then don't forget to **Logout** and login again.

#### Check you permission 

```shell
$ docker run hello-world
```

done.