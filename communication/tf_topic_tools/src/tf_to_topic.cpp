// This node transform a given tf transformation into an string and publish in given topic
// String format: source_frame target_frame x y z qx qy qz qw
// where x,y,z are in meters and qx,qy,qz,qw are the quaternion components

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include "std_msgs/String.h"
#include <sstream>
#include <stdlib.h>

int main(int argc, char **argv){

	if (argc < 5) {
		std::cout << "Unexpected number of parameters. Please use the following format: \n" << "tf_to_topic origin target topic hz\n" 
			  << "String format: source_frame target_frame x y z qx qy qz qw\n" << "where x,y,z are in meters and qx,qy,qz,qw are the quaternion components\n";
		return 1;
	}

	ros::init(argc, argv, "tf_to_topic");
  	ros::NodeHandle n;

	ros::Publisher publisher = n.advertise<std_msgs::String>(argv[3], 10);
	ros::Rate loop_rate(atoi(argv[4]));

	tf::TransformListener listener;

	while (ros::ok()){

		tf::StampedTransform transform;
		try{

			// http://mirror.umd.edu/roswiki/doc/diamondback/api/tf/html/c++/classtf_1_1Transformer.html
			// lookupTransform it is a little bit confusing, target is the source and source is the target
			// Parameters:
    			// target_frame The frame to which data should be transformed
    			// source_frame The frame where the data originated
    			// time 	The time at which the value of the transform is desired. (0 will get the latest)
    			// transform 	The transform reference to fill.

			listener.lookupTransform(argv[1], argv[2], ros::Time(0), transform);

			double x = transform.getOrigin().x();
			double y = transform.getOrigin().y();
			double z = transform.getOrigin().z();

			double qx = transform.getRotation().x();
			double qy = transform.getRotation().y();
			double qz = transform.getRotation().z();
			double qw = transform.getRotation().w();
		
			std_msgs::String msg;
			std::stringstream ss;
			ss << argv[1] << " " << argv[2] << " " << x << " " << y << " " << z << " " << qx <<  " " << qy <<  " " << qz << " " << qw;
			msg.data = ss.str();

			publisher.publish(msg);

		} catch (tf::TransformException ex){
			//ROS_ERROR("%s",ex.what());
			//ros::Duration(1.0).sleep();
		}

		
		ros::spinOnce();
		loop_rate.sleep();
	}

  return 0;
}
