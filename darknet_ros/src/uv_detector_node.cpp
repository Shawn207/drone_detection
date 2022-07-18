#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sstream>
#include <math.h>
#include <vector>
#include <time.h>
#include <./darknet_ros/UV_detector.h>
#include <./darknet_ros/kalman_filter.h>
#include <Eigen/Dense>
#include <queue>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <librealsense2/rs.hpp>

#include "darknet_ros_msgs/BoundingBox3D.h"
#include "darknet_ros_msgs/BoundingBox3DArray.h"
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

using namespace cv; 
using namespace std;

class my_detector
{  
	public:  
		tf2_ros::Buffer tfBuffer;
  		tf2_ros::TransformListener tfListener;
		// create point stamped for point coords transformation
		geometry_msgs::PointStamped point_camera;
		geometry_msgs::PointStamped point_world;
		my_detector():tfListener(tfBuffer)  
		{  	
			image_transport::ImageTransport it(nh);
			
			//Topic subscribed 
			// depsub = it.subscribe("/camera/depth/image_rect_raw", 1, &my_detector::depthCallback,this);
			// imgsub = it.subscribe("/camera/color/image_raw", 1, &my_detector::imageCallback,this);
			// rectsub = nh.subscribe("/person_detector/human_rect",1,&my_detector::rectCallback,this);
			depsub = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &my_detector::depthCallback,this);
			imgsub = it.subscribe("/camera/color/image_raw", 1, &my_detector::imageCallback,this);
			rectsub = nh.subscribe("/person_detector/human_rect",1,&my_detector::rectCallback,this);
			
			ros::Time time = ros::Time::now();
			ros::Duration duration(5.0);
			while(ros::Time::now() - time < duration){
				ROS_INFO("Wating. Give some tfBuffer some time to populate transformations");
			}

			// Topic published
			marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
			bboxes_pub = nh.advertise<darknet_ros_msgs::BoundingBox3DArray>("Bounding_Box3D", 1);
			person_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("person_visualization_marker", 1);
			// obstacles = n.advertise<std_msgs::Float64MultiArray>("Obstacles", 1000); // working on

		}  

		void rectCallback(const darknet_ros_msgs::BoundingBoxes& msg){
			// ROS_INFO("get rects");
			bboxes_human = msg;
			// ROS_INFO("num of rects, %d", bboxes_human.bounding_boxes.size()-1);
		}

		void imageCallback(const sensor_msgs::ImageConstPtr& msg){
			// ROS_INFO("get rgb");
			cv_bridge::CvImagePtr cv_ptr;
			try{
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
			}
			catch (cv_bridge::Exception& e){
				ROS_ERROR("Could not convert from '%s' to 'bgr8'.", e.what());
			return;
			}
			cv::Mat RGB = cv_ptr->image;
			this->uv_detector.readrgb(RGB);
		}

		void depthCallback(const sensor_msgs::ImageConstPtr& msg){
			// ROS_INFO("get depth");
			cv_bridge::CvImagePtr cv_ptr;
			try{
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1); 
			}
			catch (cv_bridge::Exception& e){
				ROS_ERROR("Could not convert from '%s' to 'TYPE_16UC1'.", e.what());
			return;
			}
			
			this->point_camera.header.frame_id = "camera";
			this->point_camera.header.stamp = ros::Time::now();

			cv::Mat depth = cv_ptr->image;
			if (depthq.size()>2) {
				depthq.pop();
			}

			depthq.push(depth);

			this->uv_detector.readdata(depthq);
			this->uv_detector.detect();
			this->uv_detector.track();
			this->uv_detector.display_U_map();
			// this->uv_detector.display_bird_view();
			this->uv_detector.extract_3Dbox();
			this->uv_detector.display_depth();

			
// // point coordinate
// //											5 _ _ _ 6
// //											| \     | \
// //											|   4 _ _ _ 7
// //											|   |   |   |
// //											|   |   |   |
// //											|   |   |   |
// //											|   |   |   |
// //											|   |   |   |
// //											1 _ | _ 2   |
// //											\ |     \ |
// //					   	 						0 _ _ _ 3
// //
// //															^z
// //															|/y
// //															--->x

			// visualization using bounding boxes
			visualization_msgs::Marker line;
			visualization_msgs::MarkerArray lines;
			line.header.frame_id = "world";
			line.type = visualization_msgs::Marker::LINE_LIST;
			line.action = visualization_msgs::Marker::ADD;
			
			line.scale.x = 0.1;

			// Line list is red
			line.color.r = 1.0;
			line.color.a = 1.0;
			line.lifetime = ros::Duration(0.05);

			// vision msgs
			darknet_ros_msgs::BoundingBox3D BBox;
			darknet_ros_msgs::BoundingBox3DArray BBoxes;
			BBoxes.header.stamp = ros::Time::now();
			BBoxes.header.frame_id = 'world';

			for(int i = 0; i < this->uv_detector.box3Ds.size(); i++){
				
				// visualization msgs

				float x = uv_detector.box3Ds[i].x / 1000.; // convert from mm to m
				float y = uv_detector.box3Ds[i].y / 1000.;
				float z = uv_detector.box3Ds[i].z / 1000.;

				float x_width = uv_detector.box3Ds[i].x_width / 1000.; // convert from mm to m
				float y_width = uv_detector.box3Ds[i].y_width / 1000.;
				float z_width = uv_detector.box3Ds[i].z_width / 1000.;

				// vector<geometry_msgs::Point> verts;
				// geometry_msgs::Point p;
				// // vertice 0
				// p.x = x-x_width / 2.; p.y = y-y_width / 2.; p.z = z-z_width / 2.;
				// verts.push_back(p);
				// // vertice 1
				// p.x = x-x_width / 2.; p.y = y+y_width / 2.; p.z = z-z_width / 2.;
				// verts.push_back(p);
				// // vertice 2
				// p.x = x+x_width / 2.; p.y = y+y_width / 2.; p.z = z-z_width / 2.;
				// verts.push_back(p);
				// // vertice 3
				// p.x = x+x_width / 2.; p.y = y-y_width / 2.; p.z = z-z_width / 2.;
				// verts.push_back(p);
				// // vertice 4
				// p.x = x-x_width / 2.; p.y = y-y_width / 2.; p.z = z+z_width / 2.;
				// verts.push_back(p);
				// // vertice 5
				// p.x = x-x_width / 2.; p.y = y+y_width / 2.; p.z = z+z_width / 2.;
				// verts.push_back(p);
				// // vertice 6
				// p.x = x+x_width / 2.; p.y = y+y_width / 2.; p.z = z+z_width / 2.;
				// verts.push_back(p);
				// // vertice 7
				// p.x = x+x_width / 2.; p.y = y-y_width / 2.; p.z = z+z_width / 2.;
				// verts.push_back(p);
				
				vector<geometry_msgs::Point> verts;
				geometry_msgs::Point p;
				// vertice 0
				p.x = x-x_width / 2.; p.y = y-y_width / 2.; p.z = z-z_width / 2.;
				point_camera.point.x = p.x; point_camera.point.y = p.y; point_camera.point.z = p.z;
				while (!tfBuffer.canTransform("world","camera",point_camera.header.stamp,ros::Duration(5.0))){
				ROS_INFO("waiting for transform");
				}
				// ROS_INFO("transform is ready!!");

				point_world = tfBuffer.transform(point_camera, "world");
				p.x = point_world.point.x; p.y =  point_world.point.y; p.z = point_world.point.z;
				verts.push_back(p);
				// vertice 1
				p.x = x-x_width / 2.; p.y = y+y_width / 2.; p.z = z-z_width / 2.;
				point_camera.point.x = p.x; point_camera.point.y = p.y; point_camera.point.z = p.z;
				point_world = tfBuffer.transform(point_camera, "world");
				p.x = point_world.point.x; p.y =  point_world.point.y; p.z = point_world.point.z;
				verts.push_back(p);
				// vertice 2
				p.x = x+x_width / 2.; p.y = y+y_width / 2.; p.z = z-z_width / 2.;
				point_camera.point.x = p.x; point_camera.point.y = p.y; point_camera.point.z = p.z;
				point_world = tfBuffer.transform(point_camera, "world");
				p.x = point_world.point.x; p.y =  point_world.point.y; p.z = point_world.point.z;
				verts.push_back(p);
				// vertice 3
				p.x = x+x_width / 2.; p.y = y-y_width / 2.; p.z = z-z_width / 2.;
				point_camera.point.x = p.x; point_camera.point.y = p.y; point_camera.point.z = p.z;
				point_world = tfBuffer.transform(point_camera, "world");
				p.x = point_world.point.x; p.y =  point_world.point.y; p.z = point_world.point.z;
				verts.push_back(p);
				// vertice 4
				p.x = x-x_width / 2.; p.y = y-y_width / 2.; p.z = z+z_width / 2.;
				point_camera.point.x = p.x; point_camera.point.y = p.y; point_camera.point.z = p.z;
				point_world = tfBuffer.transform(point_camera, "world");
				p.x = point_world.point.x; p.y =  point_world.point.y; p.z = point_world.point.z;
				verts.push_back(p);
				// vertice 5
				p.x = x-x_width / 2.; p.y = y+y_width / 2.; p.z = z+z_width / 2.;
				point_camera.point.x = p.x; point_camera.point.y = p.y; point_camera.point.z = p.z;
				point_world = tfBuffer.transform(point_camera, "world");
				p.x = point_world.point.x; p.y =  point_world.point.y; p.z = point_world.point.z;
				verts.push_back(p);
				// vertice 6
				p.x = x+x_width / 2.; p.y = y+y_width / 2.; p.z = z+z_width / 2.;
				point_camera.point.x = p.x; point_camera.point.y = p.y; point_camera.point.z = p.z;
				point_world = tfBuffer.transform(point_camera, "world");
				p.x = point_world.point.x; p.y =  point_world.point.y; p.z = point_world.point.z;
				verts.push_back(p);
				// vertice 7
				p.x = x+x_width / 2.; p.y = y-y_width / 2.; p.z = z+z_width / 2.;
				point_camera.point.x = p.x; point_camera.point.y = p.y; point_camera.point.z = p.z;
				point_world = tfBuffer.transform(point_camera, "world");
				p.x = point_world.point.x; p.y =  point_world.point.y; p.z = point_world.point.z;
				verts.push_back(p);
				// printf("center %f, %f, %f\n",x,y,z);

				
				int vert_idx[12][2] = {
					{0,1},
					{1,2},
					{2,3},
					{0,3},
					{0,4},
					{1,5},
					{3,7},
					{2,6},
					{4,5},
					{5,6},
					{4,7},
					{6,7}
				};
				
				for (int i=0;i<12;i++){
					line.points.push_back(verts[vert_idx[i][0]]);
					line.points.push_back(verts[vert_idx[i][1]]);
				}
				
				lines.markers.push_back(line);
				line.id++;

				// vision msgs
				BBox.center.position.x = x;
				BBox.center.position.y = y;
				BBox.center.position.z = z;

				BBox.size.x = x_width;
				BBox.size.y = y_width;
				BBox.size.z = z_width;
				BBoxes.boxes.push_back(BBox);

				bool person = false;
				Rect uv_box2d = Rect( uv_detector.bounding_box_U[i].tl().x ,uv_detector.bounding_box_U[i].tl().y, uv_detector.bounding_box_U[i].br().x - uv_detector.bounding_box_U[i].tl().x, uv_detector.bounding_box_U[i].br().y -uv_detector.bounding_box_U[i].tl().y);
				for (int j = 1 ; j < bboxes_human.bounding_boxes.size() ; j++) // first box is meaningless.
				{
					human_rect = Rect(bboxes_human.bounding_boxes[i].xmin,bboxes_human.bounding_boxes[i].ymin, bboxes_human.bounding_boxes[i].xmax-bboxes_human.bounding_boxes[i].xmin, bboxes_human.bounding_boxes[i].ymax-bboxes_human.bounding_boxes[i].ymin);
					Rect overlap = human_rect & uv_box2d;
					// printf("uv_box x,y,w,h %d,%d,%d,%d",uv_box2d.x, uv_box2d.y, uv_box2d.width,uv_box2d.height);
					// printf("overlap x,y,w,h %d,%d,%d,%d",overlap.x, overlap.y, overlap.width,overlap.height);
					person = person ||(overlap.area() >= 0.5 * uv_box2d.area());
					// printf("overlap, uv_box2d %d, %d\n",overlap.area(), uv_box2d.area() );
				}
				if (person)
				{
					uv_detector.person_box3Ds.push_back(uv_detector.box3Ds[i]);
					ROS_INFO("person detected\n");
				}

			}
			marker_pub.publish(lines);
			bboxes_pub.publish(BBoxes);


			



			// visualization using sphere
			visualization_msgs::Marker marker;
			visualization_msgs::MarkerArray markers;
			
			marker.header.frame_id = "camera_link";
			marker.id = 0;
			marker.type = visualization_msgs::Marker::SPHERE;
			marker.action = visualization_msgs::Marker::ADD;
			double u_r, u_l, d_b, d_t;
			for(int i = 0; i < this->uv_detector.person_box3Ds.size(); i++)
			{
				// cout<<"----------------------------"<<endl;
				// cout<<"Object "<< i <<": "<<endl;
				// cout<<"x: " << uv_detector.person_box3Ds[i].x<<endl;
				// cout<<"y: " <<uv_detector.person_box3Ds[i].y<<endl;
				// cout<<"z: " <<uv_detector.person_box3Ds[i].z<<endl;

				marker.lifetime = ros::Duration(0.05);
				marker.pose.position.x = uv_detector.person_box3Ds[i].x / 1000.; // convert from mm to m
				marker.pose.position.y = uv_detector.person_box3Ds[i].y / 1000.;
				marker.pose.position.z = uv_detector.person_box3Ds[i].z / 1000.;

				marker.scale.x = uv_detector.person_box3Ds[i].x_width / 1000.;
				marker.scale.y = uv_detector.person_box3Ds[i].y_width / 1000.;
				marker.scale.z = uv_detector.person_box3Ds[i].z_width / 1000.;

				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = 0.0;
				marker.pose.orientation.w = 1.0;
				
				marker.color.a = 0.7; // Don't forget to set the alpha!
				marker.color.r = abs(sin(i));
				marker.color.g = abs(cos(i));
				marker.color.b = (abs(cos(i)) + abs(sin(i))) / 2;
				markers.markers.push_back(marker);
				marker.id++;
			}

			person_marker_pub.publish(markers);
			uv_detector.person_box3Ds.clear();
		}

	private:  
		queue<cv::Mat> depthq;
		ros::NodeHandle nh;   		// define node
    	image_transport::Subscriber depsub;		// define subscriber for depth image
		image_transport::Subscriber imgsub;
		ros::Subscriber rectsub;

		UVdetector uv_detector;
		ros::Publisher marker_pub;
		ros::Publisher bboxes_pub;
		ros::Publisher person_marker_pub;

		darknet_ros_msgs::BoundingBoxes bboxes_human;
		Rect human_rect;

		// ros::Publisher obstacles; // working on
};

int main(int argc, char **argv)  
{  
	//Initiate ROS  
	ros::init(argc, argv, "my_realsense_recorder");  
	ROS_INFO("after init");
	//Create an object of class SubscribeAndPublish that will take care of everything  
	my_detector SAPObject;

	ros::spin();  
	return 0;  
} 
