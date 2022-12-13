#include "../include/BaseDetector.hpp"
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"

BaseDetector baseDetector;


ros::Publisher pub, pubBaseSize;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

	try
	{
		baseDetector.setImage(cv_bridge::toCvShare(msg, "bgr8")->image);
		baseDetector.applyMask();

		std_msgs::Float64MultiArray array;
		std_msgs::Float64MultiArray baseSize;
		array.data.resize(4);
		baseSize.data.resize(2);

        baseDetector.processImage();

		// Verifica se achou alguma elipse
		 if (baseDetector.findSquare()) {
			baseDetector.drawClosestBase();
		// 	baseDetector.setClosestBaseDistance();

			if (baseDetector.success){
				/* array.data[0] = baseDetector.dx.data;
				array.data[1] = baseDetector.dy.data; */
				array.data[0] = baseDetector.x;
				array.data[1] = baseDetector.y;
				array.data[2] = float(baseDetector.cameraImage.centerX)*2;
				array.data[3] = float(baseDetector.cameraImage.centerY)*2;
				pub.publish(array);
				
				baseSize.data[0] = baseDetector.closestBase.width;
				baseSize.data[1] = baseDetector.closestBase.height;
				pubBaseSize.publish(baseSize);
			}
			
		}  

		// Mostra a imagem
		baseDetector.show();
		// imshow("Final", baseDetector.imageFinalC1);
		//imshow("Blue", baseDetector.imageBlueC1);
		//imshow("Yellow", baseDetector.imageYellowC1);
			
		int key = waitKey(20);

		// if (key == 32) {
		// 	imwrite("/tmp/debug.jpeg", baseDetector.imageMainC3);

		// 	key = 255;
		// }
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

int main(int argc, char *argv[])
{
 	ros::init(argc, argv, "image_listener");

	ros::NodeHandle n;

	pub = n.advertise<std_msgs::Float64MultiArray>("/base_detector/px_py", 3);
	pubBaseSize = n.advertise<std_msgs::Float64MultiArray>("/base_detector/base_size", 3);
	
	string packagePath = ros::package::getPath("FURG-fase1");
	packagePath.append("/src/mask_no_gripper.png");
	baseDetector.loadMask(packagePath);
	cout << packagePath << "\n\n\n";

	image_transport::ImageTransport it(n);
	image_transport::Subscriber sub = it.subscribe("/uav1/bluefox_optflow/image_raw", 1, imageCallback);
	
	ros::spin();
		
	//destroyAllWindows();
}
