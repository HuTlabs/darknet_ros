/*
 * YoloObjectDetector.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 *
 *
 *  Modified on: May 20, 2018
 *      Authors: Alejandro Díaz, Adrian Romero and Gonzalo Nuño
 *    Institute: UPM, Universidad Politécnica de Madrid
 */

// YOLO object detector
#include "darknet_ros/YoloObjectDetector.hpp"
#include <geometry_msgs/Pose.h>

// Check for xServer
#include <X11/Xlib.h>

#ifdef DARKNET_FILE_PATH
std::string darknetFilePath_ = DARKNET_FILE_PATH;
#else
#error Path of darknet repository is not defined in CMakeLists.txt.
#endif

namespace darknet_ros 
{
   char *cfg;
   char *weights;
   char *data;
   char **detectionNames;
   char **detectionProbability; //for coord inclussion

   YoloObjectDetector::YoloObjectDetector(ros::NodeHandle nh)
       : nodeHandle_(nh),
         imageTransport_(nodeHandle_),
         numClasses_(0),
         classLabels_(0),
         rosBoxes_(0),
         rosBoxCounter_(0),
         imagergb_sub(imageTransport_,"/kinect2/qhd/image_color",1),       //For depth inclussion
         imagedepth_sub(imageTransport_,"/kinect2/qhd/image_depth_rect",1),   //For depth inclussion
         sync_1(MySyncPolicy_1(5), imagergb_sub, imagedepth_sub)        //For depth inclussion

   {
      ROS_INFO("[YoloObjectDetector] Node started.");

      // Read parameters from config file.
      if (!readParameters())
      {
         ros::requestShutdown();
      }
      init();
   }

   YoloObjectDetector::~YoloObjectDetector()
   {
      {
         boost::unique_lock<boost::shared_mutex> lockNodeStatus(mutexNodeStatus_);
         isNodeRunning_ = false;
      }
   yoloThread_.join();
   }

   bool YoloObjectDetector::readParameters()
   {
      // Load common parameters.
      nodeHandle_.param("image_view/enable_opencv", viewImage_, true);
      nodeHandle_.param("image_view/wait_key_delay", waitKeyDelay_, 3);
      nodeHandle_.param("image_view/enable_console_output", enableConsoleOutput_, false);

      // Check if Xserver is running on Linux.
      if (XOpenDisplay(NULL))
      {
         // Do nothing!
         ROS_INFO("[YoloObjectDetector] Xserver is running.");
      }
      else
      {
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

   void YoloObjectDetector::init()
   {
      ROS_INFO("[YoloObjectDetector] init().");

      // Initialize deep network of darknet.
      std::string weightsPath;
      std::string configPath;
      std::string dataPath;
      std::string configModel;
      std::string weightsModel;

      // Threshold of object detection.
      float thresh;
      nodeHandle_.param("yolo_model/threshold/value", thresh, (float) 0.3);

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

      // Get classes.
      detectionNames = (char**) realloc((void*) detectionNames, (numClasses_ + 1) * sizeof(char*));
      for (int i = 0; i < numClasses_; i++)
      {
          detectionNames[i] = new char[classLabels_[i].length() + 1];
          strcpy(detectionNames[i], classLabels_[i].c_str());
      }

      // Load network.
      setupNetwork(cfg, weights, data, thresh, detectionNames, numClasses_, 0, 0, 1, 0.5, 0, 0, 0, 0);
      yoloThread_ = std::thread(&YoloObjectDetector::yolo, this);

      // Initialize publisher and subscriber.
      std::string cameraTopicName;
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


      std::string point1;
      int pointquesize;
      bool pointlatch;


      std::string depthTopicName;        //For depth inclussion
      int depthQueueSize;                //For depth inclussion

      nodeHandle_.param("subscribers/camera_reading/topic", cameraTopicName, std::string("/kinect2/qhd/image_color"));
      nodeHandle_.param("subscribers/camera_reading/queue_size", cameraQueueSize, 1);

      nodeHandle_.param("subscribers/camera_depth/topic", depthTopicName, std::string("/kinect2/qhd/image_depth_rect"));   //For depth inclussion
      nodeHandle_.param("subscribers/camera_depth/queue_size", depthQueueSize, 1);                            //For depth inclussion

      nodeHandle_.param("publishers/object_detector/topic", objectDetectorTopicName, std::string("found_object"));
      nodeHandle_.param("publishers/object_detector/queue_size", objectDetectorQueueSize, 1);
      nodeHandle_.param("publishers/object_detector/latch", objectDetectorLatch, false);

      nodeHandle_.param("publishers/bounding_boxes/topic", boundingBoxesTopicName, std::string("bounding_boxes"));
      nodeHandle_.param("publishers/bounding_boxes/queue_size", boundingBoxesQueueSize, 1);
      nodeHandle_.param("publishers/bounding_boxes/latch", boundingBoxesLatch, false);

      nodeHandle_.param("publishers/detection_image/topic", detectionImageTopicName, std::string("detection_image"));
      nodeHandle_.param("publishers/detection_image/queue_size", detectionImageQueueSize, 1);
      nodeHandle_.param("publishers/detection_image/latch", detectionImageLatch, true);
      
      
      nodeHandle_.param("publishers/point/topic", point1, std::string("points"));
      nodeHandle_.param("publishers/point/queue_size", pointquesize, 1);
      nodeHandle_.param("publishers/point/latch", pointlatch, false);


      sync_1.registerCallback(boost::bind(&YoloObjectDetector::cameraCallback,this,_1,_2));   //For depth inclussion

      objectPublisher_ = nodeHandle_.advertise<std_msgs::Int8>(objectDetectorTopicName, objectDetectorQueueSize, objectDetectorLatch);
      boundingBoxesPublisher_ = nodeHandle_.advertise<darknet_ros_msgs::BoundingBoxes>(boundingBoxesTopicName, boundingBoxesQueueSize, boundingBoxesLatch);
      detectionImagePublisher_ = nodeHandle_.advertise<sensor_msgs::Image>(detectionImageTopicName, detectionImageQueueSize, detectionImageLatch);
      
      points_v=nodeHandle_.advertise<geometry_msgs::Pose>(point1,pointquesize,pointlatch);


      // Action servers.
      std::string checkForObjectsActionName;
      nodeHandle_.param("actions/camera_reading/topic", checkForObjectsActionName, std::string("check_for_objects"));
      checkForObjectsActionServer_.reset(new CheckForObjectsActionServer(nodeHandle_, checkForObjectsActionName, false));
      checkForObjectsActionServer_->registerGoalCallback(boost::bind(&YoloObjectDetector::checkForObjectsActionGoalCB, this));
      checkForObjectsActionServer_->registerPreemptCallback(boost::bind(&YoloObjectDetector::checkForObjectsActionPreemptCB, this));
      checkForObjectsActionServer_->start();
   }

   void YoloObjectDetector::cameraCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::ImageConstPtr& msgdepth)
   {

      ROS_DEBUG("[YoloObjectDetector] USB image received.");
      cv_bridge::CvImagePtr cam_image;
      cv_bridge::CvImageConstPtr cam_depth;

      //if (msgdepth->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
      //   ROS_INFO("32FC1");
      //else if (msgdepth->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
      //   ROS_INFO("16UC1");

      try
      {
         cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
         cam_depth = cv_bridge::toCvCopy(msgdepth, sensor_msgs::image_encodings::TYPE_32FC1);
		 //cam_depth = cv_bridge::toCvCopy(msgdepth, sensor_msgs::image_encodings::MONO8);

         imageHeader_ = msg->header;
      }

      catch (cv_bridge::Exception& e)
      {
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
      }
    

      if (cam_image)
      {
         {
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
            camImageCopy_ = cam_image->image.clone();
         }
         {
            boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_);
            imageStatus_ = true;
         }
         frameWidth_ = cam_image->image.size().width;
         frameHeight_ = cam_image->image.size().height;
      }

      if (cam_depth)
      {
         DepthImageCopy_ = cam_depth->image.clone();
      }

      return;
   }

   void YoloObjectDetector::checkForObjectsActionGoalCB()
   {
      ROS_DEBUG("[YoloObjectDetector] Start check for objects action.");

      boost::shared_ptr<const darknet_ros_msgs::CheckForObjectsGoal> imageActionPtr = checkForObjectsActionServer_->acceptNewGoal();
      sensor_msgs::Image imageAction = imageActionPtr->image;

      cv_bridge::CvImagePtr cam_image;

      try
      {
         cam_image = cv_bridge::toCvCopy(imageAction, sensor_msgs::image_encodings::BGR8);
      }
      
      catch (cv_bridge::Exception& e)
      {
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
      }

      if (cam_image)
      {
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

   void YoloObjectDetector::checkForObjectsActionPreemptCB()
   {
      ROS_DEBUG("[YoloObjectDetector] Preempt check for objects action.");
      checkForObjectsActionServer_->setPreempted();
   }

   bool YoloObjectDetector::isCheckingForObjects() const
   {
      return (ros::ok() && checkForObjectsActionServer_->isActive() && !checkForObjectsActionServer_->isPreemptRequested());
   }

   bool YoloObjectDetector::publishDetectionImage(const cv::Mat& detectionImage)
   {
      if (detectionImagePublisher_.getNumSubscribers() < 1)
         return false;

      cv_bridge::CvImage cvImage;
      cvImage.header.stamp = ros::Time::now();
      cvImage.header.frame_id = "detection_image";
      cvImage.encoding = sensor_msgs::image_encodings::BGR8;
      cvImage.image = detectionImage;
      detectionImagePublisher_.publish(*cvImage.toImageMsg());

      ROS_DEBUG("Detection image has been published.");
      return true;
   }

   //double YoloObjectDetector::getWallTime()
   //{
      //struct timeval time;
      //if (gettimeofday(&time, NULL))
      //{
         //return 0;
      //}
      //return (double) time.tv_sec + (double) time.tv_usec * .000001;
   //}

   int YoloObjectDetector::sizeNetwork(network *net)
   {
      int i;
      int count = 0;
      for(i = 0; i < net->n; ++i)
      {
         layer l = net->layers[i];
         if(l.type == YOLO || l.type == REGION || l.type == DETECTION)
         {
            count += l.outputs;
         }
      }
      return count;
   }

   void YoloObjectDetector::rememberNetwork(network *net)
   {
      int i;
      int count = 0;
      for(i = 0; i < net->n; ++i)
      {
         layer l = net->layers[i];
         if(l.type == YOLO || l.type == REGION || l.type == DETECTION)
         {
            memcpy(predictions_[demoIndex_] + count, net->layers[i].output, sizeof(float) * l.outputs);
            count += l.outputs;
         }
      }
   }

   detection *YoloObjectDetector::avgPredictions(network *net, int *nboxes)
   {
      int i, j;
      int count = 0;
      fill_cpu(demoTotal_, 0, avg_, 1);

      for(j = 0; j < demoFrame_; ++j)
      {
         axpy_cpu(demoTotal_, 1./demoFrame_, predictions_[j], 1, avg_, 1);
      }

      for(i = 0; i < net->n; ++i)
      {
         layer l = net->layers[i];
         if(l.type == YOLO || l.type == REGION || l.type == DETECTION)
         {
            memcpy(l.output, avg_ + count, sizeof(float) * l.outputs);
            count += l.outputs;
         }
      }
      detection *dets = get_network_boxes(net, buff_[0].w, buff_[0].h, demoThresh_, demoHier_, 0, 1, nboxes);
      return dets;
   }

   void *YoloObjectDetector::detectInThread()
   {
      running_ = 1;
      float nms = .4;

      layer l = net_->layers[net_->n - 1];
      float *P = buffLetter_[(buffIndex_ + 2) % 3].data;
      float *prediction = network_predict(net_, P);

      rememberNetwork(net_);
      detection *dets = 0;
      int nboxes = 0;
      dets = avgPredictions(net_, &nboxes);

      if (nms > 0) do_nms_obj(dets, nboxes, l.classes, nms);

      if (enableConsoleOutput_)
      {
         printf("\033[2J");
         printf("\033[1;1H");
         printf("\nFPS:%.1f\n",fps_);
         printf("Objects:\n\n");
      }
      image display = buff_[(buffIndex_+2) % 3];
      draw_detections(display, dets, nboxes, demoThresh_, demoNames_, demoAlphabet_, demoClasses_);

      // Extract the bounding boxes and send them to ROS
      int i, j;
      int count = 0;
      for (i = 0; i < nboxes; ++i)
      {
         float xmin = dets[i].bbox.x - dets[i].bbox.w / 2.;
         float xmax = dets[i].bbox.x + dets[i].bbox.w / 2.;
         float ymin = dets[i].bbox.y - dets[i].bbox.h / 2.;
         float ymax = dets[i].bbox.y + dets[i].bbox.h / 2.;

         if (xmin < 0)
            xmin = 0;
         if (ymin < 0)
            ymin = 0;
         if (xmax > 1)
            xmax = 1;
         if (ymax > 1)
            ymax = 1;

         // Iterate through possible boxes and collect the bounding boxes
         for (j = 0; j < demoClasses_; ++j)
         {
            if (dets[i].prob[j])
            {
               float x_center = (xmin + xmax) / 2;
               float y_center = (ymin + ymax) / 2;
               float BoundingBox_width = xmax - xmin;
               float BoundingBox_height = ymax - ymin;

               // Define bounding box - BoundingBox must be 1% size of frame (3.2x2.4 pixels)
               if (BoundingBox_width > 0.01 && BoundingBox_height > 0.01)
               {
                  roiBoxes_[count].x = x_center;
                  roiBoxes_[count].y = y_center;
                  roiBoxes_[count].w = BoundingBox_width;
                  roiBoxes_[count].h = BoundingBox_height;
                  roiBoxes_[count].Class = j;
                  roiBoxes_[count].prob = dets[i].prob[j];
                  count++;
               }
            }
         }
      }

      // Create array to store found bounding boxes
      // If no object detected, make sure that ROS knows that num = 0
      if (count == 0) 
      {
         roiBoxes_[0].num = 0;
      }
      else
      {
         roiBoxes_[0].num = count;
      }

      free_detections(dets, nboxes);
      demoIndex_ = (demoIndex_ + 1) % demoFrame_;
      running_ = 0;
      return 0;
   }

   void *YoloObjectDetector::fetchInThread()
   {
      IplImage* ROS_img = getIplImage();
      ipl_into_image(ROS_img, buff_[buffIndex_]);
      {
         boost::shared_lock<boost::shared_mutex> lock(mutexImageCallback_);
         buffId_[buffIndex_] = actionId_;
      }
      rgbgr_image(buff_[buffIndex_]);
      letterbox_image_into(buff_[buffIndex_], net_->w, net_->h, buffLetter_[buffIndex_]);
      return 0;
   }

   void *YoloObjectDetector::displayInThread(void *ptr)
   {


      int c = cvWaitKey(waitKeyDelay_);
      if (c != -1) c = c%256;
      if (c == 27)
      {
         demoDone_ = 1;
         return 0;
      }
      else if (c == 82)
      {
         demoThresh_ += .02;
      }
      else if (c == 84)
      {
         demoThresh_ -= .02;
         if(demoThresh_ <= .02) demoThresh_ = .02;
      }
      else if (c == 83)
      {
         demoHier_ += .02;
      }
      else if (c == 81)
      {
         demoHier_ -= .02;
         if(demoHier_ <= .0) demoHier_ = .0;
      }

      return 0;

   }

   void *YoloObjectDetector::displayLoop(void *ptr)
   {
      while (1)
      {
         displayInThread(0);
      }
   }

   void *YoloObjectDetector::detectLoop(void *ptr)
   {
      while (1)
      {
         detectInThread();
      }
   }

   void YoloObjectDetector::setupNetwork(char *cfgfile, char *weightfile, char *datafile, float thresh, char **names, int classes, int delay, char *prefix, int avg_frames, float hier, int w, int h, int frames, int fullscreen)
   {
      demoPrefix_ = prefix;
      demoDelay_ = delay;
      demoFrame_ = avg_frames;
      image **alphabet = load_alphabet_with_file(datafile);
      demoNames_ = names;
      demoAlphabet_ = alphabet;
      demoClasses_ = classes;
      demoThresh_ = thresh;
      demoHier_ = hier;
      fullScreen_ = fullscreen;
      printf("YOLO V3\n");
      net_ = load_network(cfgfile, weightfile, 0);
      set_batch_network(net_, 1);
   }

  void YoloObjectDetector::yolo()
   {
      const auto wait_duration = std::chrono::milliseconds(2000);
      while (!getImageStatus())
      {
         printf("Waiting for image.\n");
         if (!isNodeRunning())
         {
            return;
         }
         std::this_thread::sleep_for(wait_duration);
      }

      std::thread detect_thread;
      std::thread fetch_thread;

      srand(2222222);

      int i;
      demoTotal_ = sizeNetwork(net_);
      predictions_ = (float **) calloc(demoFrame_, sizeof(float*));

      for (i = 0; i < demoFrame_; ++i)
      {
         predictions_[i] = (float *) calloc(demoTotal_, sizeof(float));
      }

      avg_ = (float *) calloc(demoTotal_, sizeof(float));

      layer l = net_->layers[net_->n - 1];
      roiBoxes_ = (darknet_ros::RosBox_ *) calloc(l.w * l.h * l.n, sizeof(darknet_ros::RosBox_));

      IplImage* ROS_img = getIplImage();
      buff_[0] = ipl_to_image(ROS_img);
      buff_[1] = copy_image(buff_[0]);
      buff_[2] = copy_image(buff_[0]);
      buffLetter_[0] = letterbox_image(buff_[0], net_->w, net_->h);
      buffLetter_[1] = letterbox_image(buff_[0], net_->w, net_->h);
      buffLetter_[2] = letterbox_image(buff_[0], net_->w, net_->h);
      ipl_ = cvCreateImage(cvSize(buff_[0].w, buff_[0].h), IPL_DEPTH_8U, buff_[0].c);

      int count = 0;

   /*   if (!demoPrefix_ && viewImage_)
      {
         cvNamedWindow("YOLO V3", CV_WINDOW_NORMAL);
         if (fullScreen_)
         {           
           cvSetWindowProperty("YOLO V3", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
         }

         else
         {
            cvMoveWindow("YOLO V3", 0, 0);
            cvResizeWindow("YOLO V3", 640, 480);
         }
      }
   */

      cvNamedWindow("YOLO", CV_WINDOW_NORMAL);
      if (fullScreen_)
      {
        cvSetWindowProperty("YOLO", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
      }

      else
      {
         cvMoveWindow("YOLO", 0, 0);
         cvResizeWindow("YOLO", 640, 480);
      }

      demoTime_ = what_time_is_it_now();

      while (!demoDone_)
      {
         buffIndex_ = (buffIndex_ + 1) % 3;
         fetch_thread = std::thread(&YoloObjectDetector::fetchInThread, this);
         detect_thread = std::thread(&YoloObjectDetector::detectInThread, this);

         if (!demoPrefix_)
         {
            fps_ = 1./(what_time_is_it_now() - demoTime_);
            demoTime_ = what_time_is_it_now();

            if (viewImage_)
            {
               displayInThread(0);               
            }
            cv::Mat image;
            cv::cvtColor(camImageCopy_, image, cv::COLOR_RGB2XYZ);
            publishInThread(image);
            cv::imshow("YOLO",image);

         }
         else
         {
            char name[256];
            sprintf(name, "%s_%08d", demoPrefix_, count);
            save_image(buff_[(buffIndex_ + 1) % 3], name);
         }
         fetch_thread.join();
         detect_thread.join();
         ++count;
         if (!isNodeRunning())
         {
            demoDone_ = true;
         }
      }
   }


   IplImage* YoloObjectDetector::getIplImage()
   {
      boost::shared_lock<boost::shared_mutex> lock(mutexImageCallback_);
      IplImage* ROS_img = new IplImage(camImageCopy_);
      return ROS_img;
   }

   bool YoloObjectDetector::getImageStatus(void)
   {
      boost::shared_lock<boost::shared_mutex> lock(mutexImageStatus_);
      return imageStatus_;
   }

   bool YoloObjectDetector::isNodeRunning(void)
   {
      boost::shared_lock<boost::shared_mutex> lock(mutexNodeStatus_);
      return isNodeRunning_;
   }

   void *YoloObjectDetector::publishInThread(cv::Mat image)
   {
      // Publish image.
      cv::Mat cvImage = cv::cvarrToMat(ipl_);
      if (!publishDetectionImage(cv::Mat(cvImage)))
      {
        ROS_DEBUG("Detection image has not been broadcasted.");
      }


      // Publish bounding boxes and detection result.
      int num = roiBoxes_[0].num;
      if (num > 0 && num <= 100)
      {
         for (int i = 0; i < num; i++)
         {
            for (int j = 0; j < numClasses_; j++)
            {
               if (roiBoxes_[i].Class == j)
               {
                  rosBoxes_[j].push_back(roiBoxes_[i]);
                  rosBoxCounter_[j]++;
               }
            }
         }

         std_msgs::Int8 msg;
         msg.data = num;
         objectPublisher_.publish(msg); 
         printf("hi");

         for (int i = 0; i < numClasses_; i++)             
         {
            if (rosBoxCounter_[i] > 0)
            {
               darknet_ros_msgs::BoundingBox boundingBox;

               for (int j = 0; j < rosBoxCounter_[i]; j++)
               {
                  int xmin = (rosBoxes_[i][j].x - rosBoxes_[i][j].w / 2) * frameWidth_;
                  int ymin = (rosBoxes_[i][j].y - rosBoxes_[i][j].h / 2) * frameHeight_;
                  int xmax = (rosBoxes_[i][j].x + rosBoxes_[i][j].w / 2) * frameWidth_;
                  int ymax = (rosBoxes_[i][j].y + rosBoxes_[i][j].h / 2) * frameHeight_;

                  YoloObjectDetector::Coordinates(i, xmin, ymin, xmax, ymax);

                  char *showx = new char[255];
                  char *showy = new char[255];
                  char *showz = new char[255];

                  sprintf(showx,"X = %f",X);
                  sprintf(showy,"Y = %f",Y);
                  sprintf(showz,"Z = %f",Z);

                  cv::Point pt1(xmin, ymin);
                  cv::Point pt2(xmax, ymax);
                  cv::Point pt3(xmin,ymin+10);
                  cv::Point pt4(xmin,ymin);
                  cv::Point pt5(xmin+60,ymin+16);
                  //Starting points for coords on bounding boxes
                  cv::Point pt6(((((xmax-xmin))/2)+xmin),(((ymax-ymin)/2)+ymin-15));
                  cv::Point pt7((((xmax-xmin))/2)+xmin,((ymax-ymin)/2)+ymin);
                  cv::Point pt8((((xmax-xmin))/2)+xmin,((ymax-ymin)/2)+ymin+15);

                  //Fill the rectangle with color
                  YoloObjectDetector::FillRectWithColor(image,i,xmin,ymin,xmax,ymax);

                  //Show coordinates on image
                   /*
                  cv::putText(image,showx,cv::Point(10,30),cv::FONT_HERSHEY_TRIPLEX,0.5,cv::Scalar(0,0,0),1);
                  cv::putText(image,showy,cv::Point(10,50),cv::FONT_HERSHEY_TRIPLEX,0.5,cv::Scalar(0,0,0),1);
                  cv::putText(image,showz,cv::Point(10,70),cv::FONT_HERSHEY_TRIPLEX,0.5,cv::Scalar(0,0,0),1);
                    */

                   //Show coordinates on bounding box
                  cv::putText(image,showx,pt6,cv::FONT_HERSHEY_TRIPLEX,0.5,cv::Scalar(0,255,0),1);
                  cv::putText(image,showy,pt7,cv::FONT_HERSHEY_TRIPLEX,0.5,cv::Scalar(0,255,0),1);
                  cv::putText(image,showz,pt8,cv::FONT_HERSHEY_TRIPLEX,0.5,cv::Scalar(0,255,0),1);

                  //Show the bounding box with the class name
                  cv::rectangle(image,pt1,pt2,cv::Scalar(255,0,255),2, 8, 0);
                  cv::rectangle(image,pt4,pt5,cv::Scalar(255,0,255),2,8,0);
                  cv::putText(image,classLabels_[i],pt3,cv::FONT_HERSHEY_TRIPLEX,0.5,cv::Scalar(0,0,0));



                  boundingBox.Class = classLabels_[i];
                  boundingBox.probability = rosBoxes_[i][j].prob;
                  boundingBox.xmin = xmin;
                  boundingBox.ymin = ymin;
                  boundingBox.xmax = xmax;
                  boundingBox.ymax = ymax;
                  boundingBox.X = X;
                  boundingBox.Y = Y;
                  boundingBox.Z = Z;
                  boundingBox.Invalid = Invalid;
                  boundingBoxesResults_.bounding_boxes.push_back(boundingBox);
                  printf("\n%d",i);
		  geometry_msgs::Pose msg23;	
		  msg23.position.x=X;
		  msg23.position.y=Y;
		  msg23.position.z=Z;
                  msg23.orientation.x=i;
		  points_v.publish(msg23);
         boundingBoxesResults_.header.stamp = ros::Time::now();
         boundingBoxesResults_.header.frame_id = "detection";
         boundingBoxesResults_.image_header = imageHeader_;



         boundingBoxesPublisher_.publish(boundingBoxesResults_);

               }
            }
         }
         //points_v.publish(msg23);
         boundingBoxesResults_.header.stamp = ros::Time::now();
         boundingBoxesResults_.header.frame_id = "detection";
         boundingBoxesResults_.image_header = imageHeader_;



         boundingBoxesPublisher_.publish(boundingBoxesResults_);
      }
      
      else
      {
         std_msgs::Int8 msg;
         msg.data = 0;
         objectPublisher_.publish(msg);
      }

      if (isCheckingForObjects())
      {
         ROS_DEBUG("[YoloObjectDetector] check for objects in image.");
         darknet_ros_msgs::CheckForObjectsResult objectsActionResult;
         objectsActionResult.id = buffId_[0];
         objectsActionResult.bounding_boxes = boundingBoxesResults_;
         checkForObjectsActionServer_->setSucceeded(objectsActionResult, "Send bounding boxes.");
      }

      boundingBoxesResults_.bounding_boxes.clear();
      for (int i = 0; i < numClasses_; i++)
      {
         rosBoxes_[i].clear();
         rosBoxCounter_[i] = 0;
      }
      

		  geometry_msgs::Pose msg23;	
		  msg23.position.x=X;
		  msg23.position.y=Y;
		  msg23.position.z=Z;
		  points_v.publish(msg23);
      

      return 0;
   }

   void YoloObjectDetector::Coordinates(int ObjID, int xmin, int ymin, int xmax, int ymax)
   {
      int xcenter = (((xmax-xmin)/2)+xmin);
      int ycenter = (((ymax-ymin)/2)+ymin);
      int Ind=0;
      int person_id=0; 
      float GrayValue=0;
      printf("\nwow1! %d",ObjID);
      if(ObjID==0)
      {
	person_id = person_id +1;
	printf("\nperson id :%d",person_id);
      }
     // float Value=0;

        /*  for(int i=xmin; i<=xmax; i++)
         for(int j=ymin; j<=ymax; j++)
        {
             Value=(float)DepthImageCopy_.at<float>(j,i);
             GrayValue+=Value;
             Ind++;

         }

          GrayValue=GrayValue/Ind;
     */
      GrayValue=(float)DepthImageCopy_.at<float>(ycenter,xcenter);
	  Invalid= true;
	  if (GrayValue!=0)
      {
         Invalid= false;
         Z=GrayValue*0.001;                         //meters
         X=((xcenter-339.5)*Z)/594.21 ;           //X=((U-Cx)*Z)/fx
         Y=((ycenter-242.7)*Z)/591.04;           //Y=((V-Cy)*Z)/fy
	//parameters found on https://github.com/OpenKinect/libfreenect/blob/master/examples/glpclview.c 
    //https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats#intrinsic_camera_calibration_of_the_kinect


         //ROS_INFO("X %f, Y %f, Z %f ,Invalid %d", X, Y, Z, Invalid);
     
   }
 }
  void YoloObjectDetector::FillRectWithColor(cv::Mat image,int ObjID, int xmin, int ymin, int xmax, int ymax)
  {
      cv::Point linept1(xmin,ymin+15);
      cv::Point linept2(xmin+60,ymin+15);

      cv::Point linept3(xmin,ymin+13);
      cv::Point linept4(xmin+60,ymin+13);

      cv::Point linept5(xmin,ymin+11);
      cv::Point linept6(xmin+60,ymin+11);

      cv::Point linept7(xmin,ymin+9);
      cv::Point linept8(xmin+60,ymin+9);

      cv::Point linept9(xmin,ymin+7);
      cv::Point linept10(xmin+60,ymin+7);

      cv::Point linept11(xmin,ymin+5);
      cv::Point linept12(xmin+60,ymin+5);

      cv::Point linept13(xmin,ymin+3);
      cv::Point linept14(xmin+60,ymin+3);

      cv::Point linept15(xmin,ymin+1);
      cv::Point linept16(xmin+60,ymin+1);

      cv::Point linept17(xmin,ymin+17);
      cv::Point linept18(xmin+60,ymin+17);

      cv::Point linept19(xmin,ymin+19);
      cv::Point linept20(xmin+60,ymin+19);

      cv::Point linept21(xmin,ymin+21);
      cv::Point linept22(xmin+60,ymin+21);

      cv::Point linept23(xmin,ymin+23);
      cv::Point linept24(xmin+60,ymin+23);

      cv::Point linept25(xmin,ymin+25);
      cv::Point linept26(xmin+60,ymin+25);

      cv::Point linept27(xmin,ymin+27);
      cv::Point linept28(xmin+60,ymin+27);

      cv::Point linept29(xmin,ymin+29);
      cv::Point linept30(xmin+60,ymin+29);


      // cv::line(image,linept1,linept2,cv::Scalar(255,0,255),2,8,0);
       cv::line(image,linept3,linept4,cv::Scalar(255,0,255),2,8,0);
       cv::line(image,linept5,linept6,cv::Scalar(255,0,255),2,8,0);
       cv::line(image,linept7,linept8,cv::Scalar(255,0,255),2,8,0);
       cv::line(image,linept9,linept10,cv::Scalar(255,0,255),2,8,0);
       cv::line(image,linept11,linept12,cv::Scalar(255,0,255),2,8,0);
       cv::line(image,linept13,linept14,cv::Scalar(255,0,255),2,8,0);
       cv::line(image,linept15,linept16,cv::Scalar(255,0,255),2,8,0);
   //    cv::line(image,linept17,linept18,cv::Scalar(255,0,255),2,8,0);
      // cv::line(image,linept19,linept20,cv::Scalar(255,0,255),2,8,0);
    //   cv::line(image,linept21,linept22,cv::Scalar(255,0,255),2,8,0);
    //   cv::line(image,linept23,linept24,cv::Scalar(255,0,255),2,8,0);
    //   cv::line(image,linept25,linept26,cv::Scalar(255,0,255),2,8,0);
    //   cv::line(image,linept27,linept28,cv::Scalar(255,0,255),2,8,0);
   //    cv::line(image,linept29,linept30,cv::Scalar(255,0,255),2,8,0);

  }

}


