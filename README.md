# Robotics programming with rospy

 - Visit [https://www.theconstructsim.com/rds-ros-development-studio/](https://www.theconstructsim.com/rds-ros-development-studio/) and create free account.
 - Create new *Rosject*
 -  create new package inside *catkin_ws* using following command
 `cd catkin_ws/src`
` catkin_create_pkg workshop rospy std_msgs`
 
 ## Source Code
 - topic_publisher.py
 ```
 #!/usr/bin/env python
import rospy
from std_msgs.msg import Int32

# initiating ros node
rospy.init_node('publisher')

# creating publisher object
pub = rospy.Publisher('counter', Int32, queue_size=1)
rate = rospy.Rate(1)  # defines frequency of publishing in Hz

count = 0
while not rospy.is_shutdown():
    pub.publish(count)  # publish data over topic
    count += 1
    rate.sleep()
 ```
 - topic_subscriber.py
 ```
 #!/usr/bin/env python
import rospy
from std_msgs.msg import Int32


def callback(msg):
    print msg

# initiaite ROS node
rospy.init_node('topic_subscriber')

# create subscriber object
sub = rospy.Subscriber('counter', Int32, callback)

rospy.spin()  # avoids exiting from this script until node is stopped
 ```
 - pub_sub.py
 ```
 #!/usr/bin/env python
import rospy
import random
from std_msgs.msg import Int32, Bool


rospy.init_node('pub_sub')

pub = rospy.Publisher('led', Bool, queue_size=1)
rate = rospy.Rate(1)

def callback(msg):
    """
    turns LED ON if topic data is multiple of 5 or 7 else turn LED Off
    :param msg: <Int32> ROS topic data 
    :return: None
    """
    if msg.data % 5 == 0 or msg.data % 7 == 0:
        # turn LED On
        pub.publish(True)
    else:
        # turn LED Off
        pub.publish(False)

def subscriber():
    sub = rospy.Subscriber('counter', Int32, callback)
    rospy.spin()
    

if __name__ == "__main__":
    subscriber()
 ```
 - service_server.py
 ```
 #!/usr/bin/env python
import rospy
from mybot.srv import WordCount, WordCountResponse

def callback(request):
    return WordCountResponse(len(request.words.split(' ')))

rospy.init_node('service_server')

serv = rospy.Service('word_count', WordCount, callback)
rospy.spin()
```
 - service_client.py
```
#!/usr/bin/env python
import rospy
from mybot.srv import WordCount

rospy.init_node('service_client')

rospy.wait_for_service('word_count')
word_counter = rospy.ServiceProxy('word_count', WordCount)

inp = "Welcome to scipy"
print "Input is : %s" % inp

count = word_counter(inp)
print count
```
 - wander.py
```
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('wander')
rospy.loginfo("Node Initiated")

pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=2)
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    forward = Twist()
    forward.linear.x = 0.1
    pub.publish(forward)
    rate.sleep()

```
 - scipybot.py 
```
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

obstacle_distance = 5

def callback(msg):
    global obstacle_distance
    obstacle_distance = min(msg.ranges)
    # rospy.loginfo("min range : %f" % obstacle_distance)

def subscriber():
    sub = rospy.Subscriber('scan', LaserScan, callback)

def publisher():
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=2)
    rate = rospy.Rate(1)
    moving_forward = True

    while not rospy.is_shutdown():
        global obstacle_distance
        moving_forward = True if obstacle_distance > 0.5 else False
        twist = Twist()
        if moving_forward:
            twist.linear.x = 0.2
        else:
            rospy.loginfo("turning robot")
            twist.angular.z = 0.2
        pub.publish(twist)
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node('scipybot')
    rospy.loginfo("Node initiated")
    subscriber()
    publisher()
```

## Create new service
- Define the service call inputs and outputs in a service defination `srv/WordCount.srv` file
```
string words
---
int32 count
```
- Make addition to `package.xml` to reflect dependencies on message system
```
 <build_depend>message_generation</build_depend>
 <exec_depend>message_runtime</exec_depend>
```
- In order to run `catkin_make` command, we have to update `CMakeLists.txt` as follows
```
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)
.
.
add_service_files(
  FILES
  WordCount.srv
)
.
.
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

## Shell Commands
- To create new package 
`catkin_create_pkg {package_name} [dependecies]`
- To start keyboard teleop for turtlebot 
`roslaunch turtlebot_teleop keyboard_teleop.launch`
- To make a python file executable 
`chmod +x filename.py`

## ROS Commands
- rostopic
[rostopic](http://wiki.ros.org/rostopic) displays run-time information about topics and also lets you print out messages being sent to a topic.
```
rostopic bw     display bandwidth used by topic
rostopic delay display delay for topic which has header
rostopic echo   print messages to screen
rostopic find   find topics by type
rostopic hz     display publishing rate of topic
rostopic info   print information about active topic
rostopic list   print information about active topics
rostopic pub    publish data to topic
rostopic type   print topic type
```
- rosservice
[rosservice](http://wiki.ros.org/rosservice) displays run-time information about [Services](http://wiki.ros.org/Services) and also lets you print out messages being sent to a topic.
```
rosservice call call the service with the provided args
rosservice find find services by service type
rosservice info print information about service
rosservice list list active services
rosservice type print service type
rosservice uri  print service ROSRPC uri
```
- rosrun
rosrun  allows you to run an executable in an arbitrary package without having to cd (or roscd) there first.
```
Usage:
rosrun package executable

Example:
rosrun rospy_tutorials talker
```
- rosmsg and rossrv
rosmsg and rossrv are handy command-line tools that provide reference information for developers and also serve as a powerful introspection tool for learning more about data being transmitted in ROS.
```
rosmsg show   Display the fields in a ROS message type
rosmsg list   Display a list of all messages
rossrv show   Show service description
rossrv list   List all services
```
