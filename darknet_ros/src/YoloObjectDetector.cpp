/*
 * YoloObjectDetector.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

// yolo object detector
#include "darknet_ros/YoloObjectDetector.hpp"
#include "darknet_ros/UV_detector.h"
#include "darknet_ros_msgs/BoundingBox3D.h"
#include "darknet_ros_msgs/BoundingBox3DArray.h"


#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
// Check for xServer
#include <X11/Xlib.h>

#ifdef DARKNET_FILE_PATH
std::string darknetFilePath_ = DARKNET_FILE_PATH;
#else
#error Path of darknet repository is not defined in CMakeLists.txt.
#endif

namespace darknet_ros {

char* cfg;
char* weights;
char* data;
char** detectionNames;

YoloObjectDetector::YoloObjectDetector(ros::NodeHandle nh)
    : nodeHandle_(nh), imageTransport_(nodeHandle_), numClasses_(0), classLabels_(0), rosBoxes_(0), rosBoxCounter_(0), tfListener(tfBuffer) {
  ROS_INFO("[YoloObjectDetector] Node started.");

  // Read parameters from config file.
  if (!readParameters()) {
    ros::requestShutdown();
  }
  // tf buffer
  
  // tf2_ros::TransformListener tfListener(tfBuffer);
  // while (!tfBuffer.canTransform("world","camera",ros::Time(),ros::Duration(5.0))){
  //   ROS_INFO("waiting for transform");
  // }

  init();
}

YoloObjectDetector::~YoloObjectDetector() {
  {
    boost::unique_lock<boost::shared_mutex> lockNodeStatus(mutexNodeStatus_);
    isNodeRunning_ = false;
  }
  yoloThread_.join();
}

bool YoloObjectDetector::readParameters() {
  // Load common parameters.
  nodeHandle_.param("image_view/enable_opencv", viewImage_, false);
  std::cout<<"+++++++++++++++++++++++++++"<<viewImage_<<"++++++++++++++++++++++++++++++";
  nodeHandle_.param("image_view/wait_key_delay", waitKeyDelay_, 3);
  nodeHandle_.param("image_view/enable_console_output", enableConsoleOutput_, false);
  // NO WINDOWS SHOW IMG ON DRONES
  // viewImage_ = false;
  // Check if Xserver is running on Linux.
  if (XOpenDisplay(NULL)) {
    // Do nothing!
    ROS_INFO("[YoloObjectDetector] Xserver is running.");
  } else {
    ROS_INFO("[YoloObjectDetector] Xserver is not running.");
    viewImage_ = false;
  }

  // Set vector sizes.
  nodeHandle_.param("yolo_model/detection_classes/names", classLabels_, std::vector<std::string>(0));

  numClasses_ = classLabels_.size();
  rosBoxes_ = std::vector<std::vector<RosBox_> >(numClasses_);
  rosBoxCounter_ = std::vector<int>(numClasses_);

  return true;
}

void YoloObjectDetector::init() {
  ROS_INFO("[YoloObjectDetector] init().");

  // Initialize deep network of darknet.
  std::string weightsPath;
  std::string configPath;
  std::string dataPath;
  std::string configModel;
  std::string weightsModel;

  // uv detector for 3d box

  // Threshold of object detection.
  float thresh;
  nodeHandle_.param("yolo_model/threshold/value", thresh, (float)0.3);

  // Path to weights file.
  nodeHandle_.param("yolo_model/weight_file/name", weightsModel, std::string("yolov2-tiny.weights"));
  nodeHandle_.param("weights_path", weightsPath, std::string("/default"));
  weightsPath += "/" + weightsModel;
  weights = new char[weightsPath.length() + 1];
  strcpy(weights, weightsPath.c_str());

  // Path to config file.
  nodeHandle_.param("yolo_model/config_file/name", configModel, std::string("yolov2-tiny.cfg"));
  nodeHandle_.param("config_path", configPath, std::string("/default"));
  configPath += "/" + configModel;
  cfg = new char[configPath.length() + 1];
  strcpy(cfg, configPath.c_str());

  // Path to data folder.
  dataPath = darknetFilePath_;
  dataPath += "/data";
  data = new char[dataPath.length() + 1];
  strcpy(data, dataPath.c_str());
  printf("DATAPATH is : %s \n", data);
  // Get classes.
  detectionNames = (char**)realloc((void*)detectionNames, (numClasses_ + 1) * sizeof(char*));
  for (int i = 0; i < numClasses_; i++) {
    detectionNames[i] = new char[classLabels_[i].length() + 1];
    strcpy(detectionNames[i], classLabels_[i].c_str());
  }

  // Load network.
  setupNetwork(cfg, weights, data, thresh, detectionNames, numClasses_, 0, 0, 1, 0.5, 0, 0, 0, 0);
  ROS_INFO("before yolo");
  yoloThread_ = std::thread(&YoloObjectDetector::yolo, this);
  ROS_INFO("after yolo");
  // Initialize publisher and subscriber.
  std::string cameraTopicName;
  std::string depthTopicName;
  int cameraQueueSize;
  std::string objectDetectorTopicName;
  int objectDetectorQueueSize;
  bool objectDetectorLatch;
  std::string boundingBoxesTopicName;
  int boundingBoxesQueueSize;
  bool boundingBoxesLatch;
  std::string detectionImageTopicName;
  int detectionImageQueueSize;
  bool detectionImageLatch;

  // nodeHandle_.param("/device_0/sensor_1/Color_0/image/data", cameraTopicName, std::string("/device_0/sensor_1/Color_0/image/data"));
  nodeHandle_.param("/camera/color/image_raw", cameraTopicName, std::string("/camera/color/image_raw"));
  nodeHandle_.param("/camera/aligned_depth_to_color/image_raw", depthTopicName, std::string("/camera/aligned_depth_to_color/image_raw"));
  nodeHandle_.param("subscribers/camera_reading/queue_size", cameraQueueSize, 1);
  nodeHandle_.param("publishers/object_detector/topic", objectDetectorTopicName, std::string("found_object"));
  nodeHandle_.param("publishers/object_detector/queue_size", objectDetectorQueueSize, 1);
  nodeHandle_.param("publishers/object_detector/latch", objectDetectorLatch, false);
  nodeHandle_.param("publishers/bounding_boxes/topic", boundingBoxesTopicName, std::string("bounding_boxes"));
  nodeHandle_.param("publishers/bounding_boxes/queue_size", boundingBoxesQueueSize, 1);
  nodeHandle_.param("publishers/bounding_boxes/latch", boundingBoxesLatch, false);
  nodeHandle_.param("publishers/detection_image/topic", detectionImageTopicName, std::string("detection_imagezzzz"));
  nodeHandle_.param("publishers/detection_image/queue_size", detectionImageQueueSize, 1);
  nodeHandle_.param("publishers/detection_image/latch", detectionImageLatch, true);
  ROS_INFO("before subscriber:");
  std::cout<<cameraTopicName<<std::endl;
  imageSubscriber_ = imageTransport_.subscribe(cameraTopicName, cameraQueueSize, &YoloObjectDetector::cameraCallback, this);
  depthSubscriber_ = imageTransport_.subscribe(depthTopicName, cameraQueueSize, &YoloObjectDetector::depthCallback,this);
  ROS_INFO("after subscrber");
  objectPublisher_ =
      nodeHandle_.advertise<darknet_ros_msgs::ObjectCount>(objectDetectorTopicName, objectDetectorQueueSize, objectDetectorLatch);
  boundingBoxesPublisher_ =
      nodeHandle_.advertise<darknet_ros_msgs::BoundingBoxes>(boundingBoxesTopicName, boundingBoxesQueueSize, boundingBoxesLatch);
  detectionImagePublisher_ =
      nodeHandle_.advertise<sensor_msgs::Image>(detectionImageTopicName, detectionImageQueueSize, detectionImageLatch);
  
  // publis 3d bbox with uv-detector
  marker_pub = nodeHandle_.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
	bboxes_pub = nodeHandle_.advertise<darknet_ros_msgs::BoundingBox3DArray>("Bounding_Box3D", 1);
  // Action servers.
  std::string checkForObjectsActionName;
  nodeHandle_.param("actions/camera_reading/topic", checkForObjectsActionName, std::string("check_for_objects"));
  checkForObjectsActionServer_.reset(new CheckForObjectsActionServer(nodeHandle_, checkForObjectsActionName, false));
  checkForObjectsActionServer_->registerGoalCallback(boost::bind(&YoloObjectDetector::checkForObjectsActionGoalCB, this));
  checkForObjectsActionServer_->registerPreemptCallback(boost::bind(&YoloObjectDetector::checkForObjectsActionPreemptCB, this));
  checkForObjectsActionServer_->start();
}

void YoloObjectDetector::cameraCallback(const sensor_msgs::ImageConstPtr& msg) {
  // std::cout<<"in cameracall back"<<std::endl;
  ROS_DEBUG("[YoloObjectDetector] USB image received.");

  cv_bridge::CvImagePtr cam_image;

  try {
    if (msg->encoding == "mono8" || msg->encoding == "bgr8" || msg->encoding == "rgb8") {
      cam_image = cv_bridge::toCvCopy(msg, msg->encoding);
    } else if ( msg->encoding == "bgra8") {
      cam_image = cv_bridge::toCvCopy(msg, "bgr8");
    } else if ( msg->encoding == "rgba8") {
      cam_image = cv_bridge::toCvCopy(msg, "rgb8");
    } else if ( msg->encoding == "mono16") {
      ROS_WARN_ONCE("Converting mono16 images to mono8");
      cam_image = cv_bridge::toCvCopy(msg, "mono8");
    } else {
      ROS_ERROR("Image message encoding provided is not mono8, mono16, bgr8, bgra8, rgb8 or rgba8.");
    }
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (cam_image) {
    {
      boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
      imageHeader_ = msg->header;
      camImageCopy_ = cam_image->image.clone();
      std::cout<<"size of image(Mat): "<<camImageCopy_.size<<std::endl;
    }
    {
      boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_);
      imageStatus_ = true;
    }
    frameWidth_ = cam_image->image.size().width;
    frameHeight_ = cam_image->image.size().height;
  }
  return;
}

void YoloObjectDetector::depthCallback(const sensor_msgs::ImageConstPtr&msg)
{
  // ROS_INFO(" in depthCall back ");
  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1); 
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("Could not convert from '%s' to 'TYPE_16UC1'.", e.what());
  return;
  }
  if (cv_ptr){
    {
      boost::unique_lock<boost::shared_mutex> lockDepthCallback(mutexDepthCallback_);
      camDepthCopy_ = cv_ptr->image.clone();
    }
    {
      boost::unique_lock<boost::shared_mutex> lockDepthStatus(mutexDepthStatus_);
      depthStatus_ = true;
    }
    // cv::imshow("Depth",camDepthCopy_);
  }

}

void YoloObjectDetector::checkForObjectsActionGoalCB() {
  ROS_DEBUG("[YoloObjectDetector] Start check for objects action.");

  boost::shared_ptr<const darknet_ros_msgs::CheckForObjectsGoal> imageActionPtr = checkForObjectsActionServer_->acceptNewGoal();
  sensor_msgs::Image imageAction = imageActionPtr->image;

  cv_bridge::CvImagePtr cam_image;

  try {
    cam_image = cv_bridge::toCvCopy(imageAction, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (cam_image) {
    {
      boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
      camImageCopy_ = cam_image->image.clone();
    }
    {
      boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexActionStatus_);
      actionId_ = imageActionPtr->id;
    }
    {
      boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_);
      imageStatus_ = true;
    }
    frameWidth_ = cam_image->image.size().width;
    frameHeight_ = cam_image->image.size().height;
  }
  return;
}

void YoloObjectDetector::checkForObjectsActionPreemptCB() {
  ROS_DEBUG("[YoloObjectDetector] Preempt check for objects action.");
  checkForObjectsActionServer_->setPreempted();
}

bool YoloObjectDetector::isCheckingForObjects() const {
  return (ros::ok() && checkForObjectsActionServer_->isActive() && !checkForObjectsActionServer_->isPreemptRequested());
}

bool YoloObjectDetector::publishDetectionImage(const cv::Mat& detectionImage) {
  if (detectionImagePublisher_.getNumSubscribers() < 1) return false;
  cv_bridge::CvImage cvImage;
  cvImage.header.stamp = ros::Time::now();
  cvImage.header.frame_id = "detection_image";
  cvImage.encoding = sensor_msgs::image_encodings::BGR8;
  cvImage.image = detectionImage;
  detectionImagePublisher_.publish(*cvImage.toImageMsg());
  ROS_DEBUG("Detection image has been published.");
  return true;
}

// double YoloObjectDetector::getWallTime()
// {
//   struct timeval time;
//   if (gettimeofday(&time, NULL)) {
//     return 0;
//   }
//   return (double) time.tv_sec + (double) time.tv_usec * .000001;
// }

int YoloObjectDetector::sizeNetwork(network* net) {
  int i;
  int count = 0;
  for (i = 0; i < net->n; ++i) {
    layer l = net->layers[i];
    if (l.type == YOLO || l.type == REGION || l.type == DETECTION) {
      count += l.outputs;
    }
  }
  return count;
}

void YoloObjectDetector::rememberNetwork(network* net) {
  int i;
  int count = 0;
  for (i = 0; i < net->n; ++i) {
    layer l = net->layers[i];
    if (l.type == YOLO || l.type == REGION || l.type == DETECTION) {
      memcpy(predictions_[demoIndex_] + count, net->layers[i].output, sizeof(float) * l.outputs);
      count += l.outputs;
    }
  }
}

detection* YoloObjectDetector::avgPredictions(network* net, int* nboxes) {
  int i, j;
  int count = 0;
  fill_cpu(demoTotal_, 0, avg_, 1);
  for (j = 0; j < demoFrame_; ++j) {
    axpy_cpu(demoTotal_, 1. / demoFrame_, predictions_[j], 1, avg_, 1);
  }
  for (i = 0; i < net->n; ++i) {
    layer l = net->layers[i];
    if (l.type == YOLO || l.type == REGION || l.type == DETECTION) {
      memcpy(l.output, avg_ + count, sizeof(float) * l.outputs);
      count += l.outputs;
    }
  }
  detection* dets = get_network_boxes(net, buff_[0].w, buff_[0].h, demoThresh_, demoHier_, 0, 1, nboxes);
  return dets;
}

void* YoloObjectDetector::detectInThread() {
  point_camera.header.stamp = ros::Time::now();// make sure the 

  running_ = 1;
  float nms = .4;

  layer l = net_->layers[net_->n - 1];
  float* X = buffLetter_[(buffIndex_ + 2) % 3].data;
  float* prediction = network_predict(net_, X);

  rememberNetwork(net_);
  detection* dets = 0;
  int nboxes = 0;
  dets = avgPredictions(net_, &nboxes);

  if (nms > 0) do_nms_obj(dets, nboxes, l.classes, nms);

  if (enableConsoleOutput_) {
    printf("\033[2J");
    printf("\033[1;1H");
    printf("\nFPS:%.1f\n", fps_);
    printf("Objects:\n\n");
  }
  image display = buff_[(buffIndex_ + 2) % 3];
  draw_detections(display, dets, nboxes, demoThresh_, demoNames_, demoAlphabet_, demoClasses_);

  // extract the bounding boxes and send them to ROS
  int i, j;
  int count = 0;
  vector<box3D> bbox_from_uv;
  vector<int> id_from_uv;
  for (i = 0; i < nboxes; ++i) {
    float xmin = dets[i].bbox.x - dets[i].bbox.w / 2.;
    float xmax = dets[i].bbox.x + dets[i].bbox.w / 2.;
    float ymin = dets[i].bbox.y - dets[i].bbox.h / 2.;
    float ymax = dets[i].bbox.y + dets[i].bbox.h / 2.;

    if (xmin < 0) xmin = 0;
    if (ymin < 0) ymin = 0;
    if (xmax > 1) xmax = 1;
    if (ymax > 1) ymax = 1;

    // iterate through possible boxes and collect the bounding boxes
    for (j = 0; j < demoClasses_; ++j) {
      if (dets[i].prob[j]) {
        float x_center = (xmin + xmax) / 2;
        float y_center = (ymin + ymax) / 2;
        float BoundingBox_width = xmax - xmin;
        float BoundingBox_height = ymax - ymin;

        // define bounding box
        // BoundingBox must be 1% size of frame (3.2x2.4 pixels)
        if (BoundingBox_width > 0.01 && BoundingBox_height > 0.01) {
          roiBoxes_[count].x = x_center;
          roiBoxes_[count].y = y_center;
          roiBoxes_[count].w = BoundingBox_width;
          roiBoxes_[count].h = BoundingBox_height;
          roiBoxes_[count].Class = j;
          roiBoxes_[count].prob = dets[i].prob[j];
          count++;
        }
        // pass into uv detector
        if (dets[i].prob[j] > demoThresh_){
          // uv-detector for depth
          // x,y of bbox is percentage of width/heigh
          x_tl = xmin*frameWidth_;
          y_tl = ymin*frameHeight_;
          cv::Mat box = camDepthCopy_(cv::Range(int(ymin*frameHeight_),int(ymax*frameHeight_)), cv::Range(int(xmin*frameWidth_),int(xmax*frameWidth_)));
          call_uv_detector(box, j, bbox_from_uv, id_from_uv);
        }
      }
    }

    // visualize all bbox from all yolo boxed regions of a single image
    visualize_bbox(bbox_from_uv, id_from_uv);
     
    
  }

  // create array to store found bounding boxes  
  // if no object detected, make sure that ROS knows that num = 0
  if (count == 0) {
    roiBoxes_[0].num = 0;
  } else {
    roiBoxes_[0].num = count;
  }
  

  // for (int i = 0; i < count; i++)
  // {
  //   cv::Mat box = camImageCopy_()
  // }

  free_detections(dets, nboxes);
  demoIndex_ = (demoIndex_ + 1) % demoFrame_;
  running_ = 0;
  return 0;
}

void YoloObjectDetector::visualize_bbox(vector<box3D> &box3Ds, vector<int> &id){
  // visualization using bounding boxes
  visualization_msgs::Marker line;
  visualization_msgs::MarkerArray lines;
  line.header.frame_id = "world";
  line.type = visualization_msgs::Marker::LINE_LIST;
  line.action = visualization_msgs::Marker::ADD;
  
  line.scale.x = 0.1;

  // Line list is green if the box is generated from yolo box of the target id, otherwise red
  line.color.g = 1.0;
  line.color.a = 1.0;
  line.lifetime = ros::Duration(0.05);

  // vision msgs

  darknet_ros_msgs::BoundingBox3D BBox;
  darknet_ros_msgs::BoundingBox3DArray BBoxes;
  BBoxes.header.stamp = ros::Time::now();
  BBoxes.header.frame_id = 'world';

  for(int i = 0; i < box3Ds.size(); i++){
    
    // visualization msgs
    // point coordinate
    //											5 _ _ _ 6
    //											| \     | \
    //											|   4 _ _ _ 7
    //											|   |   |   |
    //											|   |   |   |
    //											|   |   |   |
    //											|   |   |   |
    //											|   |   |   |
    //											1 _ | _ 2   |
    //											\ |     \ |
    //					   	 						0 _ _ _ 3
    //
    //															^z
    //															|/y
    //															--->x
    // if (id[i] == 0){
    //   line.color.g = 1.0;
    //   line.color.r = 0.0;
    // }
    // else{
    //   line.color.r = 1.0;
    //   line.color.g = 0.0;
    // }

    // visualize person only
    if (id[i] != 0){
      break;
    }
    
    float x = box3Ds[i].x / 1000.; // convert from mm to m
    float y = box3Ds[i].y / 1000.;
    float z = box3Ds[i].z / 1000.;

    float x_width = box3Ds[i].x_width / 1000.; // convert from mm to m
    float y_width = box3Ds[i].y_width / 1000.;
    float z_width = box3Ds[i].z_width / 1000.;
    
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
  }
  marker_pub.publish(lines);
	bboxes_pub.publish(BBoxes);
}

void YoloObjectDetector::call_uv_detector(cv::Mat depth, int target_label, vector<box3D> &bbox, vector<int> &id)
{
  this->uv_detector.x0 = x_tl;
  this->uv_detector.y0 = y_tl;
  this->uv_detector.readdepth(depth);
  this->uv_detector.detect();
  // this->uv_detector.track();
  this->uv_detector.display_U_map();
  // this->uv_detector.display_bird_view();
  this->uv_detector.extract_3Dbox();
  for (int i=0;i<this->uv_detector.box3Ds.size();i++){
    bbox.push_back(this->uv_detector.box3Ds[i]);
    id.push_back(target_label);
  }
  this->uv_detector.display_depth();

}

void* YoloObjectDetector::fetchInThread() {
  {
    boost::shared_lock<boost::shared_mutex> lock(mutexImageCallback_);
    CvMatWithHeader_ imageAndHeader = getCvMatWithHeader();
    free_image(buff_[buffIndex_]);
    buff_[buffIndex_] = mat_to_image(imageAndHeader.image);
    headerBuff_[buffIndex_] = imageAndHeader.header;
    buffId_[buffIndex_] = actionId_;
  }
  rgbgr_image(buff_[buffIndex_]);
  letterbox_image_into(buff_[buffIndex_], net_->w, net_->h, buffLetter_[buffIndex_]); // image in buff_ is now in buffLetter, buff_ is dealocated
  return 0;
}

void* YoloObjectDetector::displayInThread(void* ptr) {
  int c = show_image(buff_[(buffIndex_ + 1) % 3], "YOLO", 1);
  if (c != -1) c = c % 256;
  if (c == 27) {
    demoDone_ = 1;
    return 0;
  } else if (c == 82) {
    demoThresh_ += .02;
  } else if (c == 84) {
    demoThresh_ -= .02;
    if (demoThresh_ <= .02) demoThresh_ = .02;
  } else if (c == 83) {
    demoHier_ += .02;
  } else if (c == 81) {
    demoHier_ -= .02;
    if (demoHier_ <= .0) demoHier_ = .0;
  }
  return 0;
}

void* YoloObjectDetector::displayLoop(void* ptr) {
  while (1) {
    displayInThread(0);
  }
}

void* YoloObjectDetector::detectLoop(void* ptr) {
  while (1) {
    detectInThread();
  }
}

void YoloObjectDetector::setupNetwork(char* cfgfile, char* weightfile, char* datafile, float thresh, char** names, int classes, int delay,
                                      char* prefix, int avg_frames, float hier, int w, int h, int frames, int fullscreen) {
  demoPrefix_ = prefix;
  demoDelay_ = delay;
  demoFrame_ = avg_frames;
  image** alphabet = load_alphabet_with_file(datafile);
  demoNames_ = names;
  demoAlphabet_ = alphabet;
  demoClasses_ = classes;
  demoThresh_ = thresh;
  demoHier_ = hier;
  fullScreen_ = fullscreen;
  printf("YOLO\n");
  net_ = load_network(cfgfile, weightfile, 0);
  set_batch_network(net_, 1);
}

void YoloObjectDetector::yolo() {
  const auto wait_duration = std::chrono::milliseconds(2000);
  point_camera.header.frame_id = "camera";
  while (!getImageStatus() && !getDepthStatus()) {
    
    printf("Waiting for image.\n");
    if (!isNodeRunning()) {
      return;
    }
    std::this_thread::sleep_for(wait_duration);
  }
  
  // std::stringstream ss;
  // ss << point_camera.header.stamp.sec << "." << point_camera.header.stamp.nsec;
  // std::cout << "time " <<ss.str() << std::endl;
  

  
  std::thread detect_thread;
  std::thread fetch_thread;

  srand(2222222);

  int i;
  demoTotal_ = sizeNetwork(net_);
  predictions_ = (float**)calloc(demoFrame_, sizeof(float*));
  for (i = 0; i < demoFrame_; ++i) {
    predictions_[i] = (float*)calloc(demoTotal_, sizeof(float));
  }
  avg_ = (float*)calloc(demoTotal_, sizeof(float));

  layer l = net_->layers[net_->n - 1];
  roiBoxes_ = (darknet_ros::RosBox_*)calloc(l.w * l.h * l.n, sizeof(darknet_ros::RosBox_));

  {
    boost::shared_lock<boost::shared_mutex> lock(mutexImageCallback_);
    CvMatWithHeader_ imageAndHeader = getCvMatWithHeader();
    buff_[0] = mat_to_image(imageAndHeader.image);
    headerBuff_[0] = imageAndHeader.header;
  }
  // {
  //   boost::shared_lock<boost::shared_mutex> lock(mutexDepthCallback_);

  // }
  buff_[1] = copy_image(buff_[0]);
  buff_[2] = copy_image(buff_[0]);
  headerBuff_[1] = headerBuff_[0];
  headerBuff_[2] = headerBuff_[0];
  buffLetter_[0] = letterbox_image(buff_[0], net_->w, net_->h);
  buffLetter_[1] = letterbox_image(buff_[0], net_->w, net_->h);
  buffLetter_[2] = letterbox_image(buff_[0], net_->w, net_->h);
  disp_ = image_to_mat(buff_[0]);

  int count = 0;
  if (!demoPrefix_ && viewImage_) {
    cv::namedWindow("YOLO", cv::WINDOW_NORMAL);
    if (fullScreen_) {
      cv::setWindowProperty("YOLO", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);
    } else {
      cv::moveWindow("YOLO", 0, 0);
      cv::resizeWindow("YOLO", 640, 480);
    }
  }

  demoTime_ = what_time_is_it_now();

  while (!demoDone_) {
    buffIndex_ = (buffIndex_ + 1) % 3;
    fetch_thread = std::thread(&YoloObjectDetector::fetchInThread, this);
    detect_thread = std::thread(&YoloObjectDetector::detectInThread, this);
    if (!demoPrefix_) {
      fps_ = 1. / (what_time_is_it_now() - demoTime_);
      demoTime_ = what_time_is_it_now();
      if (viewImage_) {
        displayInThread(0);
      } else {
        generate_image(buff_[(buffIndex_ + 1) % 3], disp_);
      }
      publishInThread();
    } else {
      char name[256];
      sprintf(name, "%s_%08d", demoPrefix_, count);
      save_image(buff_[(buffIndex_ + 1) % 3], name);
    }
    fetch_thread.join();
    detect_thread.join();
    ++count;
    if (!isNodeRunning()) {
      demoDone_ = true;
    }
  }
}

CvMatWithHeader_ YoloObjectDetector::getCvMatWithHeader() {
  CvMatWithHeader_ header = {.image = camImageCopy_, .header = imageHeader_};
  return header;
}

bool YoloObjectDetector::getImageStatus(void) {
  boost::shared_lock<boost::shared_mutex> lock(mutexImageStatus_);
  return imageStatus_;
}

bool YoloObjectDetector::getDepthStatus(void) {
  boost::shared_lock<boost::shared_mutex> lock(mutexDepthStatus_);
  return depthStatus_;
}

bool YoloObjectDetector::isNodeRunning(void) {
  boost::shared_lock<boost::shared_mutex> lock(mutexNodeStatus_);
  return isNodeRunning_;
}

void* YoloObjectDetector::publishInThread() {
  // Publish image.
  cv::Mat cvImage = disp_;
  if (!publishDetectionImage(cv::Mat(cvImage))) {
    ROS_DEBUG("Detection image has not been broadcasted.");
  }

  // Publish bounding boxes and detection result.
  int num = roiBoxes_[0].num;
  if (num > 0 && num <= 100) {
    for (int i = 0; i < num; i++) {
      for (int j = 0; j < numClasses_; j++) {
        if (roiBoxes_[i].Class == j) {
          rosBoxes_[j].push_back(roiBoxes_[i]);
          rosBoxCounter_[j]++;
        }
      }
    }

    darknet_ros_msgs::ObjectCount msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "detection";
    msg.count = num;
    objectPublisher_.publish(msg);

    for (int i = 0; i < numClasses_; i++) {
      if (rosBoxCounter_[i] > 0) {
        darknet_ros_msgs::BoundingBox boundingBox;

        for (int j = 0; j < rosBoxCounter_[i]; j++) {
          int xmin = (rosBoxes_[i][j].x - rosBoxes_[i][j].w / 2) * frameWidth_;
          int ymin = (rosBoxes_[i][j].y - rosBoxes_[i][j].h / 2) * frameHeight_;
          int xmax = (rosBoxes_[i][j].x + rosBoxes_[i][j].w / 2) * frameWidth_;
          int ymax = (rosBoxes_[i][j].y + rosBoxes_[i][j].h / 2) * frameHeight_;

          boundingBox.Class = classLabels_[i];
          boundingBox.id = i;
          boundingBox.probability = rosBoxes_[i][j].prob;
          boundingBox.xmin = xmin;
          boundingBox.ymin = ymin;
          boundingBox.xmax = xmax;
          boundingBox.ymax = ymax;
          boundingBoxesResults_.bounding_boxes.push_back(boundingBox);
        }
      }
    }
    boundingBoxesResults_.header.stamp = ros::Time::now();
    boundingBoxesResults_.header.frame_id = "detection";
    boundingBoxesResults_.image_header = headerBuff_[(buffIndex_ + 1) % 3];
    boundingBoxesPublisher_.publish(boundingBoxesResults_);
  } else {
    darknet_ros_msgs::ObjectCount msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "detection";
    msg.count = 0;
    objectPublisher_.publish(msg);
  }
  if (isCheckingForObjects()) {
    ROS_DEBUG("[YoloObjectDetector] check for objects in image.");
    darknet_ros_msgs::CheckForObjectsResult objectsActionResult;
    objectsActionResult.id = buffId_[0];
    objectsActionResult.bounding_boxes = boundingBoxesResults_;
    checkForObjectsActionServer_->setSucceeded(objectsActionResult, "Send bounding boxes.");
  }
  boundingBoxesResults_.bounding_boxes.clear();
  for (int i = 0; i < numClasses_; i++) {
    rosBoxes_[i].clear();
    rosBoxCounter_[i] = 0;
  }

  return 0;
}

} /* namespace darknet_ros*/
