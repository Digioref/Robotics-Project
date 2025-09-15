# Robotics: Project

## Authors
This Project was developed by:
- Francesco Di Giore [@Digioref](https://github.com/Digioref)
- Federico de Introna [@federicodeintrona](https://github.com/federicodeintrona)  
- Carlo Arnone [@CarloArnone](https://github.com/CarloArnone)

## Introduction
This is the repository for the two Projects of Robotics in the academic year 2022/2023 at Polytechnic of Milan.

Subject: 089013 - Robotics

Professor: Matteucci Matteo

Academic Year: 2022/2023

## Description of the Project
The project is divided into two sub-projects, whose specifications are:
- First project: compute the odometry of an autonomous vehicle.
- Second project: mapping of an environment and then map navigation.

For the first project, please refer to [first project](Specifications/first_project_2023.pdf).

For the second project, please refer to [second project](Specifications/second_project_2023.pdf).

Both pojects were done in **C++** and using **ROS** as the robot framework.

### First project
In [first_project](first_project.zip), there are four folders: launch, msg, srv, src.

In the folder launch, there is only one file, the first_project.launch file, which is used to start the node odom_node. Inside this file, we specified:
- the node, named odom_node;
- 4 static transformation for the 4 robot's single plane lasers;
- set of some node parameters: starting_x, starting_y and starting_th, all set to 0.0 as a starting value;
- set use_sim_time to true;
- 
Using the command "roslaunch first_project first_project.launch", all the nodes should start.

In the folder msg, we defined the structure of the custom message Odom in the only file there, Odom.msg:
- 3 fields with type float representing x, y and th;
- a string named timestamp representing the time;

In the folder srv, we defined the structure of the service reset_odom in the file reset_odom.srv. Inside the file, we specified:
- a boolean resetted, which is the return value of the service, set to true when the odometry is resetted;
- no input;

In the folder src, there is the source code of the node, inside a file called odom_node.cpp, written in C++ language. In this file, we created:
- a class called odom_node;
- a main function;

In the main function, the node is initialized with "ros::init" and the odom_node is created.
The class has some private features and some methods.
The private features are:
- n, the NodeHandle;
- sub_odom, which is a subscriber subscribed to the topic "speed_steer";
- pub_odom, a publisher which publishes messages about the topic "odometry";
- pub_custom, a publisher which publishes messages about the topic "custom_odometry";
- starting_x, starting_y and starting_th are the values used for the computation of the odometry;
- t_prev is the previous time;
- transform_broadcaster publishes the transformation;
- odomTransform is a TransformStamped message published by the transform_broadcaster;
- reset_service is the service which, when it's called, resets the starting values (sets each of them to 0.0);

Then, there is the method odom_node(), which:
- takes the starting values for starting_x, starting_y and starting_th from the first_project.launch file;
- sub_odom subscribes to the "speed_steer" topic;
- pub_odom and pub_custom are connected to the corresponding topic ("odometry" and "custom_odometry", respectively);
- reset_service is connected to the callback (reset_odom);

After this method, we defined the methods used for the computation of the odometry and the publication of the messages.

The method calculate_odometry, as its name underlines, calculates the odometry of the robot. We considered the kinematics of the robot as an Ackermann ssteering, using the bycicle approximation. In the calculus, we used the 2nd order Runge-Kutta integration for a better approximation.

First of all, we computed the angular velocity of the robot using the data provided by the "speed_steer" topic and the following formula:

**ω = V ∙ tan(α) / d**, 

where d = 2.8 and α is the steering angle.

Then, the pose is computed:

**xk+1 = xk + vkT cos(θk + ωkT/2)**

**yk+1 = yk + vkT sin(θk + ωkT/2)**

**θk+1 = θk + ωkT**

**T = tk+1 - tk**

where (xk+1, yk+1, θk+1) is the next pose, while (xk, yk, θk) is the previous one, and T is the delta time.
After the computation, the methods publishTFOdometry and publishOdometry are called and the new pose is assigned to the old one.

The method publishTFOdometry takes in input the new pose (x_next, y_next, theta_next) and the time and publishes the odomTransform (a TransformStamped message), which has several features:
- stamp, which is the time, which the input value is assigned to;
- header.frame_id, which is the parent's reference frame, so it is set to "odom";
- child_frame_id, which is the robot's reference frame, so it is set to "base_link";

Then, we set odomTranform's translation according to the values in input and we created a Quaternion, setting it using the provided theta. Finally, we set odomTransform's rotation using the Quaternion and, using the transform_broadcaster, the odomTransform message is published.

In the publishOdometry, we did similar steps. We created two messages, an Odom message and an Odometry message. The Odom message is the custom one, so we set its features (x, y, theta, timestamp) using the values in input. While, for the Odometry message, we proceeded in the same way as for the tf aforementioned, adding the velocity, linear and angular, set according to the input values. Finally, both messages are published using the corresponding publisher.

The last method is reset_odom, the callback method of the service. It sets to 0.0 all the starting features and set to true the boolean resetted, returning true.

### Second Project
For mapping we decided to use slam toolbox instead of gmapping mainly for the ease of use.
We decided to clean the map a little for navigation even though we probably didn't need to. 
We also decided to move the robot's starting position to the bottom-left corner to make it easier to plan the path with the waypoints.
During mapping we used a node to publish the tf.

For localization we used amcl as written in the specifics.
For path planning we used the base global planner and teb local planner with clearing rotation.
To move the robot we used move_base and used our node as an action client to publish the goals.

For the navigation node we decided to read 4 variables from the csv file x and y for the position and z and w of a quaternion for the heading. (We could have easily used radiants then convert them in a quaternion but we preferred to use the quaternion values directly).

To load the waypoints.csv file, we used "getParam" in the node navigation to get the path of the file, which is set in the launch file as a parameter, and then the file is opened using ifstream.

For the code, check the zip folder [second_project](second_project.zip)

## Final Considerations
First Project Mark: 3.0/3.0

Second Project Mark: 3.0/3.0
