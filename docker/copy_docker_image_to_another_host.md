# Copy docker image to another host

### Export your image to a file:

```shell
$ docker save -o <*path/NewFileName> <image name>
```

### Copy to another computer:

switch to the directory:

```shell
$ cd <save path>
```

copy:

```shell
$ scp <file name> IP:<*path/file name> 
```

key in password. If you want copy entire folder, add -r like "$ scp -r ..............."

### Load the image file:

At the destination computer:

```shell
$ docker load -i <*path/image file>
```

done.