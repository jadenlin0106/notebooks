# Edit ubuntu environment $PATH

### Check current $PATH

```shell
$ echo $PATH
```

### Append a path

Append at the end of original path:

```shell
$ export PATH=$PATH:<new path>
```

Append at the front of original path:

```shell
$ export PATH=<new path>:$PATH
```

### Edit $PATH

```shell
$ export PATH=<new entire PATH>
```

Fill it, you can copy from the result of "$ echo $PATH".