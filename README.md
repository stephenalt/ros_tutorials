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
