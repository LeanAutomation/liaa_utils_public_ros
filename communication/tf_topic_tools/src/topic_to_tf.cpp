// This node transforms a given string topic into a tf graph
// String format: source_frame target_frame x y z qx qy qz qw
// where x,y,z are in meters and qx,qy,qz,qw are the quaternion components

#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "std_msgs/String.h"
#include <sstream>

void callback(const std_msgs::String::ConstPtr& msg){

	static tf::TransformBroadcaster br;

	tf::Transform transform;

	char origin[1024];
	char target[1024];

	double x,y,z,qx,qy,qz,qw;
	sscanf(msg->data.c_str(), "%s %s %lf %lf %lf %lf %lf %lf %lf", origin, target, &x, &y, &z, &qx, &qy, &qz, &qw);

	transform.setOrigin(tf::Vector3(x, y, z));
	tf::Quaternion q(qx,qy,qz,qw);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), origin, target));

}

int main(int argc, char **argv)
{

	if (argc < 2) {
		std::cout << "Unexpected number of parameters. Please use the following format: \n" << "topic_to_tf topic\n"
			  << "String format: source_frame target_frame x y z qx qy qz qw\n" << "where x,y,z are in meters and qx,qy,qz,qw are the quaternion components\n";
		return 1;
	}

	ros::init(argc, argv, "topic_to_tf");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe(argv[1], 10, callback);
	ros::spin();

  	return 0;
}
