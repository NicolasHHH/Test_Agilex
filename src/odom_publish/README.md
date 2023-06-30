# Odom Publish Package

## 0. Quick Start
    
    # 1. launch gazebo simulation ===========

    cd project_folder
    catkin_make 
    source devel/setup.bash
    roslaunch scout_gazebo_sim scout_empty_world.launch
    
    # 2. launch odom_publish node ===========

    rosrun odom_publish odom_publish_transform.py

    # or 
    roslaunch odom_publish odom_publish.launch

    # or (recommended)
    cd src/odom_publish/scripts
    python odom_publish_transform.py
    


## 1. Objective & Main Function

**Motivation** : The original gazebo simulation of the robot does not contain
the fixed global frame `/world`, whose existence should facilitate the 
installation of sensor stations and global positioning. Originally, all links point to the `/base_link` frame.
Therefore, we would like to establish a ROS node that links the `/world` frame and 
the `/base_link` frame in the `tf_tree`.

**Main Function** :  Establish a ROS node that publishes the tf transform 
between `/world` and `/base_link`, and the odometry information of the robot as a by-product.

_Before_ : 
![before.svg](doc%2Fbefore.svg)
_After_ : 
![after.svg](doc%2Fafter.svg)

## 2. Realisation Details

1. Call ROS service to get information from gazebo simulation 
(model state). The same could be done by subscribing to the `/gazebo/model_state` topic.

       # service registration

       rospy.wait_for_service("/gazebo/get_model_state")
       get_model_state = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)

       # model state request

       model = GetModelStateRequest()
       model.model_name = "scout/"

       # get model state

       result = get_model_state(model)


2. Publish the tf transform between `/world` and `/base_link`:

        br = TransformBroadcaster()
        br.sendTransform((result.pose.position.x, result.pose.position.y, result.pose.position.z),
                         (result.pose.orientation.x, result.pose.orientation.y, result.pose.orientation.z,
                          result.pose.orientation.w),
                         rospy.Time.now(),
                         "base_link",
                         "world")
        

3. Publish the odometry information of the robot:

        odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)
        odom = Odometry()
        odom.header.frame_id = "world"
        odom.child_frame_id = "base_link"

        result = get_model_state(model)
        odom.pose.pose = result.pose
        odom.twist.twist = result.twist
        odom_pub.publish(odom)