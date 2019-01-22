# Introduction to tf

### Set Up the Demo

The nodes for this tutorial are released for Ubuntu, so go ahead and install them: 

```shell
$ sudo apt-get install ros-kinetic-ros-tutorials ros-kinetic-geometry-tutorials ros-kinetic-rviz ros-kinetic-rosbash ros-kinetic-rqt-tf-tree
```

### Running the Demo

Now that we're done getting the turtle_tf tutorial package, let's run the demo. 

```shell
$ roslaunch turtle_tf turtle_tf_demo.launch
```

*Make sure that you are using python2, instead python3. Type ""$ python --version" to check.

You will see the turtlesim start with two turtles. 

![1547535862951](/home/jaden/.config/Typora/typora-user-images/1547535862951.png)

### Using rqt_tf_tree

Usage: 

```
rosrun rqt_tf_tree rqt_tf_tree
```

Or simply: 

```
rqt &
```

Then choose [rqt_tf_tree](http://wiki.ros.org/rqt_tf_tree) from `Plugins` tab. 

- ![snapshot_rqt_tree_turtle_tf.png](http://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf?action=AttachFile&do=get&target=snapshot_rqt_tree_turtle_tf.png)

### Using tf_echo

Usage: 

```shell
rosrun tf tf_echo [reference_frame] [target_frame]
```

Let's look at the transform of the turtle2 frame with respect to turtle1 frame which is equivalent to ![\large{$$\mathbf{T}_{turtle1\_turtle2} =\mathbf{T}_{turtle1\_world} *\mathbf{T}_{world\_turtle2}$$}](http://wiki.ros.org/tf/Tutorials/Introduction%20to%20tf?action=AttachFile&do=get&target=latex_986fed48f3e5bc349f7ca256812c18ebe46a588b_p1.png) : 

```shell
$ rosrun tf tf_echo turtle1 turtle2
```

## rviz and tf

[rviz](http://wiki.ros.org/rviz) is a visualization tool that is useful for examining tf frames. Let's look at our turtle frames using `rviz`. Let's start `rviz` with the turtle_tf configuration file using the `-d` option for `rviz`: 

```shell
$ rosrun rviz rviz -d `rospack find turtle_tf`/rviz/turtle_rviz.rviz
```

![1547536474378](/home/jaden/.config/Typora/typora-user-images/1547536474378.png)



#### Broadcaster, listener and frame

tf::TransformBroadcaster br;      //透過 tf 發送座標軸的工具

tf::Transform transform;     //描述設定座標軸的欄



tf::TransformListener listener;    //接收 tf 上兩個node關係的工具

​	用法 

```
listener.lookupTransform("/turtle2", "/turtle1", ros::Time(0), transform)
```

​	或 

```
listener.lookupTransform("/turtle2", "/carrot1",ros::Time(0), transform);
```

