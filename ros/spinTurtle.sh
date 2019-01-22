#!/bin/bash
# Program:
# 	spin ${Turtle name} from 0 to 360
# History:
# 2019/01/18	Jaden Lin	First release
PATH=/bin:/sbin:/usr/bin:/usr/sbin:/usr/local/bin:/usr/local/sbin:~/bin
export PATH
source ~/catkin_ws/devel/setup.bash

read -p "Please input the turtle name that you want to spin: " turtleName

#s=0
#for (( deg=1; deg<=${nu}; deg=deg+10 ))
#do
	rosservice call /spawn 10 10 0 ${turtleName}
    rosservice call /spawn 6 7 0 "turtle4"
#done
#echo "The result of '1+2+3+...+${nu}' is ==> ${s}"