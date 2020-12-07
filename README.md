<h3>ROS tutorials</h3>

<a href="http://wiki.ros.org/">ROS Tutorials</a>

<h5> Notes </h5>

<ul> Creating a ROS Package
  <li>
    <p> Using catkin to create a new package. Requirements: package.xml file, CMakeLists.txt file, own folder. To create a new package 'cd' into 'catkin_ws/src' and run the 'catkin_create_pkg [name] [deps]' script. This creates a folder which contains a package.xml and CMakeLists.txt files, which are partially filled out (if you included deps). To build the packages in your catkin_ws run the 'catkin_make' script.
  </li>
  <li>Analyze first order dependencies of a package by running the 'rospack depends1 [package name]' command<li>
  <li>Dependencies are split into build_depend, buildtool_depend, exec_depend, and test_depend. If you want your dependencies to be available at build and runtime, you must add the exec_depend tag.</li>
  <li>Now we will build our package using catkin_make. catkin_make is a command line tool which adds some convenience to the standard catkin workflow. catkin_make combines the calls to cmake and make in the standard CMake workflow.</li>
  <li>The 'build' folder is the default location of the build space and is where cmake and make are called to configure and build your packages. The 'devel' folder is the default location of the devel space, which is where your executables and libraries go before you install your pacakges.</li>
</ul>

<ul> Understanding ROS Nodes
  <li>Node: an executable that uses ROS to communicate with other nodes. A node isn't much more than an executable file within a ROS package</li>
  <li>Message: ROS data type used when subscribing or publishing to a topic</li>
  <li>Topic: Nodes can publish messages to a topic as well as subscribe to a topic to receive messages</li>
  <li>Master: Name service for ROS (the service that helps nodes find each other)</li>
  <li>rosout: ROS equivalent to stdout/stderr</li>
  <li>roscore: Master + rosout + parameter server</li>
  <li>Client libraries: ROS client libraries allow nodes written in different programming languages to communicate: rospy = python client library, roscpp = c++ client library</li>
  <li>'roscore' is the first command you should run when using ROS.</li>
  <li>'rosnode list': lists the active nodes. 'rosout' node is always running as it collects and logs nodes' debugging output. 'rosnode info [/node_name]' returns information about a specific node.</li>
  <li>'rosrun [package_name] [node_name]': command that allows you to use the package name to directly run a node within a package (without having to know the package path).'</li>
  <li>'rosrun [package_name] [node_name] __name:=new_name': allows you to reassign a nodes name from the command line.__</li>
  <li>'rosnode ping [node_name]': command to ping a node to test that it is running</li>
  <li>'rosrun rqt_graph rqt_graph': rqt_graph creates a dynamic graph of what's going on in the system</li>
  <li>One way of communication between nodes is through topics. Nodes can subscribe and publish to topics.</li>
  <li>'rostopic': allows you to get information about topics. 'rostopic echo': shows the data published on a topic. 'rostopic list': returns a list of all topics currently subscribed to and published.</li>
  <li>Communication on topcis happens by sending ROS messages between nodes. For the publisher and subscriber to communicate, the publisher and subscriber must send and receive the same type of message.</li>
  <li>A topic type is defined by the message type published on it. The type of the message sent on a topic can be determined using the 'rostopic type' command.</li>
  <li>'rostopic pub [topic] [msg_type] [args]': command used to publish data on to a topic currently advertised.</li>
  <li>Example: ~~~~ rostopic pub -1 /turtle1/cmd_vel geometry_msgs/Twist -- '[2.0, 0.0, 0.0]' '[0.0, 0.0, 1.8]' ~~~~ IMPORTANT: must use -- before listing negative numbers, etc. or else the parser will think it is a flag. Inputs are in YAML.</li>
  <li>'rostopic hz [topic]': reports the rate at which data is being published to a topic.</li>
  <li>rqt_plot: displays a scrolling time plot of the data published on topics. Use the following command to run rqt_plot: 'rosrun rqt_plot rqt_plot'</li>
</ul>

<ul> Nodes continued...
  <li>ROS Services: another way that nodes can communicate with each other. Services allow nodes to send a request and receive a response.</li>
  <li>'rosservice list': prints information about active services</li>
  <li>'rosservice type [service_name]': prints the service type of the given service</li>
  <li>'rosservice call [service] [args]': Calls the service with the provided arguments</li>
</ul>

<ul> ROS Parameter Server
  <li>'rosparam': allows you to store and manipulate data on the ROS Parameter Server. The Parameter Server can store intgers, floats, boolean, dictionaries, and lists.</li>
  <li>'rosparam list': Lists all of the parameter names on the Parameter Server</li>
  <li>'rosparam set [parameter]': Set a parameter on the Parameter Server</li>
  <li>'rosparam get [parameter]': Get a parameter's value from the Parameter Server</li>
  <li>'rosparam load [file_name] [namespace]': Load parameters onto the Parameter Server with a file</li>
  <li>'rosparam dump [file_name] [namespace]': Dump parameters from the Parameter Server into a file</li>
</ul>

<ul> Using rqt_console and roslaunch
  <li>'rqt_console': attaches to ROS's logging framework to display output from nodes.</li>
  <li>'rqt_logger_level': allows you to change the verbosity level (DEBUG, WARN, INFO, and ERROR) of nodes as they run.</li>
  <li>'roslaunch': roslaunch starts nodes as defined in a launch file.</li>
</ul>

<ul> Using 'rosed' to edit files in ROS
  <li>'rosed [package_name] [filename]': rosed is part of the rosbash suite. It allows you to directly edit a file within a pakcage by using the package name rather than having to type the entire path to the package</li>
  <li>Default editor for 'rosed' is the one you provide in your .bashrc file</li>
</ul>

<ul> Creating a ROS msg and srv
  <li>msg: msg files are simple text files that describe the fields of a ROS message. They are used to generate source code for messages in different languages. Stored in the 'msg' directory of a package</li>
  <li>srv: a srv file describes a service. It is composed of two parts: a request and a response. Stored in the 'srv' directory of a package</li>
  <li>msgs are simple text files with a field type and field name per line, included field types are: int8, int16, int32, int64 (plus uint), float32, float64, string, time, duration, other msg files, variable-length array[] and fixed-length array[C], Header</li>
  <li>'Header': A special type in ROS, the header contains a timestamp and coordinate frame information that are commonly used in ROS. You will frequently see the first line in a msg file have Header header</li>
  <li>srv files are just like msg files with the exception of containing request types and response types. The two parts are separated by a '---' line. The request types/names are above the '---' and the response types/names are below the '---'</li>
  <li>Converting msg files to source code: To make sure that the msg files are turned into source code for C++, Python, and other languages: (1) Open your package.xml file, (2) make sure the following two lines are included: '<build_depend>message_generation</build_depend>' and '<exec_depend>message_runtime</exec_depend>' (3) Open your CMakeLists.txt package file and add the (4) 'message_generation' dependency to the 'find_package' call, (5) add the 'message_runtime' dependency to your catkin_package macro after CATKIN_DEPENDS, (6) add the msg file to the 'add_message_files' macro, (7) uncomment the 'generate_messages' macro</li>
  <li>'rosmsg show [package_name/msg_name]': Outputs the definition of the given message</li>
  <li>To generate the source files go to the root of your workspace and run catkin_make</li>
</ul>

<ul> Creating a srv
  <li>Create the 'srv' directory in your package (if it does not exist)</li>
  <li>'roscp [package_name] [file_to_copy_path] [copy_path]': Tool to copy files from one package to another</li>
  <li>Converting srv files to source code: (1) Add the following two lines to your package.xml file '<build_depend>message_generation</build_depend>' and '<exec_depend>message_runtime</exec_depend>' (2) add 'message_generation' dependency to the find_package macro in CMakeLists.txt (3) Add the file to the 'add_service_files' macro in CMakeLists.txt (4)</li>
  <li>'rossrv show [package_name/service_name]': Outputs the definition of the given service</li>
  <li>To generate the source files go to the root of your workspace and run catkin_make</li>
</ul>

<ul> Writing a simple publisher and subscriber (python)
  <li>1st: Create a 'scripts' folder in your package to store your Python scripts</li>
  <li>2nd: Add the following to the 'catkin_install_python' macro in your CMakeLists.txt file: 'PROGRAMS scripts/[name_of_file].py</li>
  <li>'#!/usr/bin/env python': will be at the top of every python ROS node, to make sure your script is executed as a python script</li>
  <li>'import rospy': ROS's python library</li>
  <li>'from std_msgs.msg import String': imports the std_msgs/String message type so we can reuse it for publishing</li>
  <li>'pub = rospy.Publisher('chatter', String, queue_size=10)': declares that your node is publishing to the chatter topic using the message type String (which is actually std_msgs.msg.String. The queue_size argument limits the amount of queued messages if any subscriber is not receiving them fast enough</li>
  <li>'rospy.init_node('talker', anonymous=True)': Tells rospy the name of your node, until rospy has this information, it cannot start communicating with the ROS Master. In this case the name of the node is 'talker'. NOTE: the name cannot have any '/'s. Anonymous = True ensures that your node has a unique name by adding random numbers to the end of NAME</li>
  <li>'rate = rospy.Rate(10)': Creates a Rate object named 'rate', with the help of the Rate object's 'sleep' method, it offers a convenient way for looping at the desired rate. With its argument of 10, we should expect to go through the loop 10 times per second (as long as our processing time does not exceed 1/10th of a second)</li>
</ul>

<ul> Writing the Subscriber node
  <li>In ROS, nodes are uniquely named. If two nodes with the same name are launched, the previous one is kicked off. The anonymous=True flag means that rospy will choose a unique name for our 'listener' node so that multiple listeners can run simultaneously</li>
  <li>'rospy.spin()': Keeps python from exiting until this node is stopped</li>
  <li>'rospy.init_node('listener', anonymous=True)': Creates a node named 'listener' + unique id</li>
  <li>'rospy.Subscriber('chatter', String, callback)': Subscribes the node created above to the 'chatter' topic which is of type String, and gives a 'callback' function to be called when data is published to the 'chatter' topic</li>
</ul>

<ul> Building your nodes
  <li>(1) add all the required fields, dependencies, etc. to your package.xml and CMakeLists.txt files</li>
  <li>(2) 'cd' into your catkin_ws directory</li>
  <li>(3) run 'catkin_make'</li>
</ul>

<ul> Running the publisher
  <li></li>
  <li></li>
  <li></li>
  <li></li>
  <li></li>
</ul>
