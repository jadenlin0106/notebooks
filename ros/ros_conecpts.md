## Robotic Operating System(ROS)

### Introduction

ROS is an open-source, meta-operating system for your robot.

hardware abstraction, low-level device control, commonly-used functionality, message-passing between processes, and package management.



### Concepts

1. #### ROS Filesystem Level

   ![「ros 架構」的圖片搜尋結果](https://van23li.github.io/ROS%E5%85%A5%E9%97%A8%E5%8F%8A%E6%A6%82%E8%AE%BA/ROS.png)

   - Packages: Contain ROS runtime processes (*nodes*), a ROS-dependent library, datasets, configuration files.
   - Repositories: A collection of packages which share a common VCS system. 
   - Message (msg) types: Message descriptions, stored in `my_package/msg/MyMessageType.msg`, define the data structures for [messages](http://wiki.ros.org/Messages) sent in ROS. 
   - Service (srv) types: Service descriptions, stored in `my_package/srv/MyServiceType.srv`, define the request and response data structures for [services](http://wiki.ros.org/Services) in ROS. 

2. #### ROS Computation Graph Level

   - Nodes: For example, one node controls a laser range-finder, one node controls the wheel motors, one node performs localization.

   - Master: Name registration, to find each other, exchange messages. (Manage nodes)

   - Messages: A message is simply a data structure, comprising typed fields.  Standard
   - primitive types (integer, floating point, boolean, etc.) are supported,
   - as are arrays of primitive types.  Messages can include arbitrarily 
   - nested structures and arrays (much like C structs). 

   - Topics: publish / subscribe. A node sends out a message by *publishing* it to a given topic.

   - Services: one or the request and one for the reply.A providing node offers a service under a [name](http://wiki.ros.org/Names) and a client uses the service by sending the request message and awaiting the reply.

   - Bags: Bags are a format for saving and playing back ROS message data.

   Nodes connect to other nodes directly. the Master only provides lookup information, much like a DNS server. The most common protocol used in a ROS is called [TCPROS](http://wiki.ros.org/ROS/TCPROS), which uses standard TCP/IP sockets.  

3. #### ROS Community Level

   - Distributions: ROS Distributions are collections of versioned [stacks](http://wiki.ros.org/Stacks) that you can install. 

   - Repositories: ROS relies on a federated network of code repositories, where different 
     institutions can develop and release their own robot software 
     components.  
   - The ROS Wiki: The ROS community Wiki is the main forum for documenting information 
     about ROS. Anyone can sign up for an account and contribute their own 
     documentation, provide corrections or updates, write tutorials, and 
     more. 

4. #### Names

   1. ##### Graph Resource Names

      Used in [Nodes](http://wiki.ros.org/Nodes), [Parameters](http://wiki.ros.org/Parameter%20Server), [Topics](http://wiki.ros.org/Topics), and [Services](http://wiki.ros.org/Services). 

      Graph Resource Names provide a hierarchical naming structure that is used for all resources in a ROS Computation Graph

      Name space

      1. Valid Nams

      2. Resolving

         - base

         - relative/name

         - /global/name

         - ~private/name

           examples:

           | **Node**    | **Relative** (default)     | **Global**               | **Private**                       |
           | ----------- | -------------------------- | ------------------------ | :-------------------------------- |
           | `/node1`    | `bar` -> `/bar`            | `/bar` -> `/bar`         | `~bar` -> `/node1/bar`            |
           | `/wg/node2` | `bar` -> `/wg/bar`         | `/bar` -> `/bar`         | `~bar` -> `/wg/node2/bar`         |
           | `/wg/node3` | `foo/bar` -> `/wg/foo/bar` | `/foo/bar` -> `/foo/bar` | `~foo/bar` -> `/wg/node3/foo/bar` |

   2. ##### Package Resource Names

      1. used in ROS with Filesystem-Level concepts 

         [Message (msg) types](http://wiki.ros.org/msg) 

         [Service (srv) types](http://wiki.ros.org/srv) 

         [Node types](http://wiki.ros.org/Nodes)

         Example: Package/[Resource Name]

### ROS Tutorials

1. #### Core ROS Tutorials

   1. Beginner Level

      1. install

      2. create workspace

         1. ```shell
            $ mkdir -p ~/catkin_ws/src
            $ cd ~/catkin_ws/
            $ catkin_make
            $ source devel/setup.bash
            ```

      3. Navigating the ROS Filesystem

         1. ```shell
            $ rospack find [package_name]
            ```

         2. cd to package

            ```shell
            $ roscd [locationname[/subdir]]
            ```

            then you don't need to use the full path name

         3. ```shell
            $ rosls [locationname[/subdir]]
            ```

      4. Creating Package

         1. ```shell
            catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
            ```

            and then edit "package.xml" to customize 

      5. Build Package

         1. ```shell
            catkin_make
            ```

      6. Nodes

         1. 節點實際上只不過是ROS包中的可執行文件。 ROS節點使用ROS客戶端庫與其他節點通信。 節點可以發布或訂閱主題。 節點還可以提供或使用服務。

         2. 先開機

            ```shell
            roscore
            ```

         3. 在另一個Terminal

            ```
            rosnode list
            ```

            ```
            rosnode info /rosout
            ```

         4. 可以run  node

            ```
            rosrun [package_name] [node_name]
            ```

            example:

            ```
            rosrun turtlesim turtlesim_node
            ```

         5. 可以在run之前改名字

            ```
            rosrun turtlesim turtlesim_node __name:=my_turtle
            ```

            Review:

            roscore = ros+core : master (provides name service for ROS) + rosout  (stdout/stderr) + parameter server (parameter server will be introduced  later) 

            rosnode = ros+node : ROS tool to get information about a node. 

            rosrun = ros+run :  runs a node from a given package. 

      7. Topics

         1. ```
            $ sudo apt-get install ros-<distro>-rqt
            $ sudo apt-get install ros-<distro>-rqt-common-plugins
            ```

            replacing <distro> with the name of your [ROS distribution](http://wiki.ros.org/Distributions) (e.g. indigo, jade, **kinetic**, lunar ...) 

         2. ```
            rosrun rqt_graph rqt_graph
            ```

            ![rqt_graph_turtle_key2.png](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=rqt_graph_turtle_key2.png)

         3. ```
            rostopic echo [topic]
            ```

            可以顯示publisher更新的資料, example:

            ```
            rostopic echo /turtle1/cmd_vel
            ```

            ![rqt_graph_echo.png](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics?action=AttachFile&do=get&target=rqt_graph_echo.png)

            echo 也算一個訂閱者

         4. 列出所有Topic

            1. ```
               rostopic list -v
               ```

         5. 看Topic 的型態

            1. ```
               rostopic type [topic]
               ```

               在用show 看型態的架構

               ```
               rosmsg show [type]
               ```

               或show Topic

         6. Public Message

            ```
            rostopic pub [topic] [msg_type] [args]
            ```

            example:

            ```
            rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]'
            ```

            拆解來看：

            ```
            rostopic pub
            ```



            This option (dash-one) causes rostopic to only publish one message then exit: 
    
            ```
             -1 
            ```



            This is the name of the topic to publish to: 
    
            ```
            /turtle1/cmd_vel
            ```



            This is the message type to use when publishing to the topic: 
    
            ```
            geometry_msgs/Twist
            ```



​        This  option (double-dash) tells the option parser that none of the following  arguments is an option.  This is required in cases where your arguments  have a leading dash `-`, like negative numbers.

```
--

'[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]' 
```



 7. 畫曲線

    ```
    rosrun rqt_plot rqt_plot
    ```

  8. Services and Parameters

     1. ```
        rosservice list         #print information about active services
        rosservice call         #call the service with the provided args
        rosservice type         #print service type
        rosservice find         #find services by service type
        rosservice uri          #print service ROSRPC uri
        ```

        看看/spawn的格式

        ```
        rosservice type /spawn | rossrv show
        ```

        得到

        ```
        float32 x
        float32 y
        float32 theta
        string name
        ---
        string name
        ```

        ```
        rosservice call /spawn 2 2 0.2 "turtle2"
        ```

     2. rosparam

        ```
        rosparam set            set parameter
        rosparam get            get parameter
        rosparam load           load parameters from file
        rosparam dump           dump parameters to file
        rosparam delete         delete parameter
        rosparam list           list parameter names
        ```

        example:

        ```
        rosparam set /background_r 150
        ```

        Usage: 

        ```
        rosparam dump [file_name] [namespace]
        rosparam load [file_name] [namespace]
        ```



​        	Here we write all the parameters to the file params.yaml 

```

​```
    ```

​```

$ rosparam dump params.yaml
        ```
```



​        You can even load these yaml files into new namespaces, e.g. `copy`: 

```
        $ rosparam load params.yaml copy
        $ rosparam get /copy/background_b
```



  9. rqt_console and roslaunch

     1. ```
        $ rosrun rqt_console rqt_console
        ```

        ```
        $ rosrun rqt_logger_level rqt_logger_level
        ```

        兩種console

     2. roslaunch

        ```
        roslaunch [package] [filename.launch]
        ```

        example:

        ```
        roslaunch beginner_tutorials turtlemimic.launch
        ```

        須先寫好.launch檔（xml格式）：

        ```
        <launch>
        
          <group ns="turtlesim1">
            <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
          </group>
        
          <group ns="turtlesim2">
            <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
          </group>
        
          <node pkg="turtlesim" name="mimic" type="mimic">
            <remap from="input" to="turtlesim1/turtle1"/>
            <remap from="output" to="turtlesim2/turtle1"/>
          </node>
        
        </launch>
        ```

  10. rosed

            1. 打開package並編輯，可以不用打整個路徑名

        ```
        rosed [package_name] [filename]
        ```

            2. 可以指定編輯器：

        ```
        export EDITOR='nano -w'
        ```

        確認當前編輯器：

        ```
        echo $EDITOR
        ```

  11. msg and srv

      [msg](http://wiki.ros.org/msg): msg  files are simple text files that describe the fields of a ROS message.  They are used to generate source code for messages in different  languages. 

      [srv](http://wiki.ros.org/srv): an srv file describes a service. It is composed of two parts: a request and a response. 就是get & set

      1. msg

         ```
         $ roscd beginner_tutorials
         $ mkdir msg
         $ echo "int64 num" > msg/Num.msg
         ```

         uncomment package.xml裡的：

         ```
           <build_depend>message_generation</build_depend>
           <exec_depend>message_runtime</exec_depend>
         ```

         ```
         # Do not just add this to your CMakeLists.txt, modify the existing text to add message_generation before the closing parenthesis
         find_package(catkin REQUIRED COMPONENTS
            roscpp
            rospy
            std_msgs
            message_generation
         )
         ```

         ```
         catkin_package(
           ...
           CATKIN_DEPENDS message_runtime ...
           ...)
         ```

         ```
         add_message_files(
           FILES
           Num.msg
         )
         ```

         ```
         generate_messages(
           DEPENDENCIES
           std_msgs
         )
         ```

         用rosmsg

         ```
         rosmsg show [message type]
         ```

         example:

         ```
         $ rosmsg show beginner_tutorials/Num
         ```

      2. srv

         ```
         roscp [想複製的package_name] [複製出來的檔名] [要複製哪個路徑]
         ```

         Unless you have done so already, open `package.xml`, and make sure these two lines are in it and [uncommented](http://www.htmlhelp.com/reference/wilbur/misc/comment.html): 

         ```
           <build_depend>message_generation</build_depend>
           <exec_depend>message_runtime</exec_depend>
         ```

         ```
         # Do not just add this line to your CMakeLists.txt, modify the existing line
         find_package(catkin REQUIRED COMPONENTS
           roscpp
           rospy
           std_msgs
           message_generation
         )
         ```

         ```
         add_service_files(
           FILES
           AddTwoInts.srv
         )
         ```

         用rossrv

         ```
         $ rossrv show <service type>
         ```

         Unless you have already done this in the previous steps, change in `CMakeLists.txt`. : 

         ```
         generate_messages(
           DEPENDENCIES
           std_msgs
         )
         ```

         ```
         # In your catkin workspace
         $ roscd beginner_tutorials
         $ cd ../..
         $ catkin_make install
         $ cd -
         ```

  12. Writing a Simple Publisher and Subscriber (C++)

        1. 建立Publisher Node

           在package下的src裡建立c++：

           Now, let's break the code down. 



           切换行号显示

           ```
             27 #include "ros/ros.h"
             28 
           ```



            `ros/ros.h` is a convenience include that includes all the headers necessary to use the most common public pieces of the ROS system. 



           切换行号显示

           ```
             28 #include "std_msgs/String.h"
             29 
           ```



            This includes the [std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html) message, which resides in the [std_msgs](http://wiki.ros.org/std_msgs) package.  This is a header generated automatically from the `String.msg` file in that package.  For more information on message definitions, see the [msg](http://wiki.ros.org/msg) page. 



           切换行号显示

           ```
             47   ros::init(argc, argv, "talker");
           ```



            Initialize  ROS.  This allows ROS to do name remapping through the command line --  not important for now. This is also where we specify the name of our  node.  Node names must be unique in a running system. 

           The name used here must be a [base name](http://wiki.ros.org/Names#Graph), ie. it cannot have a `/` in it. 



           切换行号显示

           ```
             54   ros::NodeHandle n;
           ```



            Create a handle to this process' node.  The first `NodeHandle`  created will actually do the initialization of the node, and the last  one destructed will cleanup any resources the node was using. 



           切换行号显示

           ```
             73   ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
           ```



            Tell the master that we are going to be publishing a message of type [std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html) on the topic `chatter`.  This lets the master tell any nodes listening on `chatter`  that we are going to publish data on that topic.  The second argument  is the size of our publishing queue.  In this case if we are publishing  too quickly it will buffer up a maximum of 1000 messages before  beginning to throw away old ones. 

           `NodeHandle::advertise()` returns a `ros::Publisher` object, which serves two purposes: 1) it contains a `publish()`  method that lets you publish messages onto the topic it was created  with, and 2) when it goes out of scope, it will automatically  unadvertise. 



           切换行号显示

           ```
             75   ros::Rate loop_rate(10);
           ```



            A `ros::Rate`  object allows you to specify a frequency that you would like to loop  at.  It will keep track of how long it has been since the last call to `Rate::sleep()`, and sleep for the correct amount of time. 

           In this case we tell it we want to run at 10Hz. 



           切换行号显示

           ```
             81   int count = 0;
             82   while (ros::ok())
             83   {
           ```



            By default roscpp will install a SIGINT handler which provides Ctrl-C handling which will cause `ros::ok()` to return false if that happens. 

           `ros::ok()` will return false if: 

           - a SIGINT is received (Ctrl-C) 
           - we have been kicked off the network by another node with the same name 
           - `ros::shutdown()` has been called by another part of the application.   
           - all ros::[NodeHandles](http://wiki.ros.org/NodeHandles) have been destroyed 

           Once `ros::ok()` returns false, all ROS calls will fail. 



           切换行号显示

           ```
             87     std_msgs::String msg;
             88 
             89     std::stringstream ss;
             90     ss << "hello world " << count;
             91     msg.data = ss.str();
           ```



            We broadcast a message on ROS using a message-adapted class, generally generated from a [msg file](http://wiki.ros.org/msg).  More complicated datatypes are possible, but for now we're going to use the standard `String` message, which has one member: "data". 



           切换行号显示

           ```
            101     chatter_pub.publish(msg);
           ```



            Now we actually broadcast the message to anyone who is connected. 



           切换行号显示

           ```
             93     ROS_INFO("%s", msg.data.c_str());
           ```



            `ROS_INFO` and friends are our replacement for `printf`/`cout`.  See the [rosconsole documentation](http://wiki.ros.org/rosconsole) for more information. 



           切换行号显示

           ```
            103     ros::spinOnce();
           ```



            Calling `ros::spinOnce()`  here is not necessary for this simple program, because we are not  receiving any callbacks.  However, if you were to add a subscription  into this application, and did not have `ros::spinOnce()` here, your callbacks would never get called.  So, add it for good measure. 



           切换行号显示

           ```
            105     loop_rate.sleep();
           ```



            Now we use the `ros::Rate` object to sleep for the time remaining to let us hit our 10Hz publish rate. 

           Here's the condensed version of what's going on: 

           - Initialize the ROS system 
           - Advertise that we are going to be publishing [std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html) messages on the `chatter` topic to the master 
           - Loop while publishing messages to `chatter` 10 times a second 

        2. Subscriber Node

           在package下的src裡建立c++：

           Now, let's break it down piece by piece, ignoring some pieces that have already been explained above. 



           切换行号显示

           ```
             34 void chatterCallback(const std_msgs::String::ConstPtr& msg)
             35 {
             36   ROS_INFO("I heard: [%s]", msg->data.c_str());
             37 }
           ```



            This is the callback function that will get called when a new message has arrived on the `chatter` topic. The message is passed in a [boost shared_ptr](http://www.boost.org/doc/libs/1_37_0/libs/smart_ptr/shared_ptr.htm),  which means you can store it off if you want, without worrying about it  getting deleted underneath you, and without copying the underlying  data. 



           切换行号显示

           ```
             75   ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
           ```



            Subscribe to the `chatter` topic with the master.  ROS will call the `chatterCallback()`  function whenever a new message arrives.  The 2nd argument is the queue  size, in case we are not able to process messages fast enough.  In this  case, if the queue reaches 1000 messages, we will start throwing away  old messages as new ones arrive. 

           `NodeHandle::subscribe()` returns a `ros::Subscriber`  object, that you must hold on to until you want to unsubscribe.  When  the Subscriber object is destructed, it will automatically unsubscribe  from the `chatter` topic. 

           There are versions of the `NodeHandle::subscribe()` function which allow you to specify a class member function, or even anything callable by a Boost.Function object.  The [roscpp overview](http://wiki.ros.org/roscpp/Overview) contains more information. 



           切换行号显示

           ```
             82   ros::spin();
           ```



            `ros::spin()`  enters a loop, calling message callbacks as fast as possible.  Don't  worry though, if there's nothing for it to do it won't use much CPU.  `ros::spin()` will exit once `ros::ok()` returns false, which means `ros::shutdown()` has been called, either by the default Ctrl-C handler, the master telling us to shutdown, or it being called manually. 

           There are other ways of pumping callbacks, but we won't worry about those here.  The [roscpp_tutorials](http://wiki.ros.org/roscpp_tutorials) package has some demo applications which demonstrate this.  The [roscpp overview](http://wiki.ros.org/roscpp/Overview) also contains more information. 

           Again, here's a condensed version of what's going on: 

           - Initialize the ROS system 
           - Subscribe to the `chatter` topic 
           - Spin, waiting for messages to arrive 
           - When a message arrives, the `chatterCallback()` function is called 

        3. Build Nodes

           把這些加到CMakeLists.txt最下面

           ```
           add_executable(talker src/talker.cpp)
           target_link_libraries(talker ${catkin_LIBRARIES})
           add_dependencies(talker beginner_tutorials_generate_messages_cpp)
           
           add_executable(listener src/listener.cpp)
           target_link_libraries(listener ${catkin_LIBRARIES})
           add_dependencies(listener beginner_tutorials_generate_messages_cpp)
           ```

           再重新build work space一次

           ```
           # In your catkin workspace
           $ cd ~/catkin_ws
           $ catkin_make  
           ```

           ＊不過每當build work space後，都要執行

           ```
           source ~/[work space]/devel/setup.bash
           ```

  13. 檢查Pub跟Subs

        1. Run Publisher 

           Make sure that a roscore is up and running: 

           ```
           $ roscore
           ```





           catkin specific If you are using catkin, make sure you have sourced your workspace's setup.sh file after calling `catkin_make` but before trying to use your applications: 



           ```
           # In your catkin workspace
           $ cd ~/catkin_ws
           $ source ./devel/setup.bash
           ```





           In the last tutorial we made a publisher called "talker". Let's run it: 

           ```
           $ rosrun beginner_tutorials talker      (C++)
           ```

        2. Run Subscriber

           在另一個Terminal：

           ```
           $ rosrun beginner_tutorials listener     (C++)
           ```

           如果是新開的Terminal就要執行

           ```
           source ~/[work space]/devel/setup.bash
           ```

  14. Writing a Simple Service and Client (C++)

        1. Writing a Service Server Node

           Now, let's break the code down. 



           切换行号显示

           ```
           #include "ros/ros.h"
           #include "beginner_tutorials/AddTwoInts.h"
           ```



            `beginner_tutorials/AddTwoInts.h` is the header file generated from the srv file that we created earlier. 



           切换行号显示

           ```
              4 bool add(beginner_tutorials::AddTwoInts::Request  &req,
              5          beginner_tutorials::AddTwoInts::Response &res)
           ```



            This  function provides the service for adding two ints, it takes in the  request and response type defined in the srv file and returns a boolean.   



           切换行号显示

           ```
              6 {
              7   res.sum = req.a + req.b;
              8   ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
              9   ROS_INFO("sending back response: [%ld]", (long int)res.sum);
             10   return true;
             11 }
           ```



            Here  the two ints are added and stored in the response. Then some  information about the request and response are logged. Finally the  service returns true when it is complete. 



           切换行号显示

           ```
             18   ros::ServiceServer service = n.advertiseService("add_two_ints", add);
           ```



            Here the service is created and advertised over ROS.  

        2. Wirting a Service Client Node

           Now, let's break the code down. 



           切换行号显示

           ```
             15   ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
           ```



            This creates a client for the `add_two_ints` service.  The `ros::ServiceClient` object is used to call the service later on. 



           切换行号显示

           ```
             16   beginner_tutorials::AddTwoInts srv;
             17   srv.request.a = atoll(argv[1]);
             18   srv.request.b = atoll(argv[2]);
           ```



            Here  we instantiate an autogenerated service class, and assign values into  its request member.  A service class contains two members, `request` and `response`.  It also contains two class definitions, `Request` and `Response`.   



           切换行号显示

           ```
             19   if (client.call(srv))
           ```



            This  actually calls the service.  Since service calls are blocking, it will  return once the call is done.  If the service call succeeded, `call()` will return true and the value in `srv.response` will be valid.  If the call did not succeed, `call()` will return false and the value in `srv.response` will be invalid. 

        3. Build nodes

           一樣要在CMakeList.txt加入些指令，讓它在執行 $ catkin_make  指令時可以被build

           ```
           add_executable(add_two_ints_server src/add_two_ints_server.cpp)
           target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
           add_dependencies(add_two_ints_server beginner_tutorials_gencpp)
           
           add_executable(add_two_ints_client src/add_two_ints_client.cpp)
           target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
           add_dependencies(add_two_ints_client beginner_tutorials_gencpp)
           ```

  15. 驗證 Service Server and Clinet

      1. Run起來

         ```
         roscore
         ```

      2. Start by running the server. Open a new shell and type: 

         ```
         rosrun beginner_tutorials add_two_ints_server
         ```

         You should see something similar to: 

         ```
         Ready to add two ints.
         ```

      3. Now let's run the client with the necessary arguments, in another shell: 

         ```
         $ rosrun beginner_tutorials add_two_ints_client 1 3
         
         ```

         In the client's shell, you should see something similar to: 

         ```
         Sum: 4
         ```

         In the server's shell, instead, you should see something similar to: 

         ```
         request: x=1, y=3
         sending back response: [4]
         ```

  16. 紀錄＆回放

        1. 建立bag資料夾，然後紀錄

           ```
           mkdir ~/bagfiles
           cd ~/bagfiles
           rosbag record -a
           ```

           -a 表示把所有資訊集中一起

        2. 回放

           ```
           rosbag play <your bagfile>
           ```

           播放倍速

           ```
           rosbag play -r <number of speed> <your bagfile>
           ```

           控制指令可以播放得很快，但烏龜跟不上，所以走得路線會不同