/*
 * References:
 * Getting angles - http://www.pdnotebook.com/2012/07/measuring-angles-in-opencv/
 * Tracking color objects - https://dl.dropboxusercontent.com/u/28096936/tuts/objectTrackingTut.cpp
 */

#include <ros/ros.h>
// allows to subscribe to compressed image streams
#include <image_transport/image_transport.h>
// header for CvBridge as well as some useful constants and functions to image encodings
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// headers for OpenCV's image processing and GUI modules
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp> // CV_AA
//#include <opencv2/objdetect/objdetect.hpp>
#include <math.h> // atan2
#include <sstream> // patch template to_string

static const std::string OPENCV_WINDOW = "Normal";
static const std::string OPENCV_WINDOW_HSV = "Origin";
static const std::string OPENCV_WINDOW_HSV2 = "Second Point";

static const bool TRACK_OBJECTS = true;
static const bool USE_MORPH_OPS = true;

// default capture width and height
static const int FRAME_WIDTH = 640;
static const int FRAME_HEIGHT = 480;
// max number of objects to be detected in frame
static const int MAX_NUM_OBJECTS = 10; //50
// minimum and maximum object area
static const int MIN_OBJECT_AREA = 2*2; // 5*5
static const int MAX_OBJECT_AREA = FRAME_HEIGHT*FRAME_WIDTH/1.5;

//our sensitivity value to be used in the absdiff() function
static const int SENSITIVITY_VALUE = 20;
//size of blur used to smooth the intensity image output from absdiff() function
static const int BLUR_SIZE = 10;

namespace patch
{
  template < typename T > std::string to_string( const T& n )
  {
    std::ostringstream stm ;
    stm << n ;
    return stm.str() ;
  }
}

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
  int h_min_;
  int h_max_;
  int s_min_;
  int s_max_;
  int v_min_;
  int v_max_;

  int h_min2_;
  int h_max2_;
  int s_min2_;
  int s_max2_;
  int v_min2_;
  int v_max2_;
  
  cv_bridge::CvImagePtr cv_ptr_;
  bool has_prev_;
  cv::Mat gray_img_;
  cv::Mat gray_img_prev_;

  //we'll have just one object to search for
  //and keep track of its position.
   int the_object_[2];
   //bounding rectangle of the object, we will use the center of this as its position.
   cv::Rect object_bounding_rectangle_;
  
public:
  ImageConverter():it_(nh_)
  {
    has_prev_ = false;
    
    the_object_[0] = 0;
    the_object_[1] = 0;
    object_bounding_rectangle_ = cv::Rect(0,0,0,0);
    
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
    
    cv::namedWindow(OPENCV_WINDOW);
    cv::namedWindow(OPENCV_WINDOW_HSV);
    cv::namedWindow(OPENCV_WINDOW_HSV2);
      
    h_min_ = 61;
    h_max_ = 85;
    s_min_ = 92;
    s_max_ = 126;
    v_min_ = 140;
    v_max_ = 178;

    h_min2_ = 2;
    h_max2_ = 36;
    s_min2_ = 84;
    s_max2_ = 255;
    v_min2_ = 148;
    v_max2_ = 255;
  
    // Create trackbars in "Control" window
    cv::createTrackbar("LowH", OPENCV_WINDOW_HSV, &h_min_, 179);
    cv::createTrackbar("HighH", OPENCV_WINDOW_HSV, &h_max_, 179);
    cv::createTrackbar("LowS", OPENCV_WINDOW_HSV, &s_min_, 255);
    cv::createTrackbar("HighS", OPENCV_WINDOW_HSV, &s_max_, 255);
    cv::createTrackbar("LowV", OPENCV_WINDOW_HSV, &v_min_, 255);
    cv::createTrackbar("HighV", OPENCV_WINDOW_HSV, &v_max_, 255);

    // Create trackbars in "Control" window
    cv::createTrackbar("LowH", OPENCV_WINDOW_HSV2, &h_min2_, 179);
    cv::createTrackbar("HighH", OPENCV_WINDOW_HSV2, &h_max2_, 179);
    cv::createTrackbar("LowS", OPENCV_WINDOW_HSV2, &s_min2_, 255);
    cv::createTrackbar("HighS", OPENCV_WINDOW_HSV2, &s_max2_, 255);
    cv::createTrackbar("LowV", OPENCV_WINDOW_HSV2, &v_min2_, 255);
    cv::createTrackbar("HighV", OPENCV_WINDOW_HSV2, &v_max2_, 255);
  }
  
  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
    cv::destroyWindow(OPENCV_WINDOW_HSV);
    cv::destroyWindow(OPENCV_WINDOW_HSV2);
  }
  
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    try
    {
      // OpenCV expects color images to use BGR channel order
      cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    /*cv::Mat diffImage;
    cv::Mat thresGray;
    if (hasPrev)
    {
      // perform frame difference
      cv::absdiff(grayImage_prev, grayImage, diffImage);
      
      // threshold intensity image at a given sensitivity value
      cv::threshold(diffImage, thresGray, SENSITIVITY_VALUE, 255, cv::THRESH_BINARY);
      cv::blur(thresGray,thresGray,cv::Size(BLUR_SIZE,BLUR_SIZE));
      cv::blur(thresGray,thresGray,cv::Size(BLUR_SIZE,BLUR_SIZE));
      cv::threshold(thresGray, thresGray, SENSITIVITY_VALUE, 255, cv::THRESH_BINARY);
      cv::dilate(thresGray, thresGray, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(50,50)));
      cv::dilate(thresGray, thresGray, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(50,50)));
      cv::imshow("Gray Threshold Image", thresGray);
      
      cv::Mat maskImage;
      cv_ptr->image.copyTo(maskImage, thresGray);
      cv::imshow("Mask Image", maskImage);
      
      if (TRACK_OBJECTS)
      {
	searchForMovement(thresGray, cv_ptr->image);
      }
    }
    hasPrev = true;
    grayImage.copyTo(grayImage_prev);*/
    
    // Convert to HSV
    cv::Mat hsv_image;
    
    cv::Mat hsv_image_small(cv::Size(160,120), hsv_image.type());
    cv::cvtColor(cv_ptr_->image, hsv_image, CV_BGR2HSV);
    
    // Use smaller image for faster processing
    cv::resize(hsv_image, hsv_image_small, hsv_image_small.size());
    
    // filter HSV image between values and store filtered image to threshold matrix
    cv::Mat thres_img;  // green
    cv::Mat thres_img2; // pink
    cv::inRange(hsv_image_small, cv::Scalar(h_min_, s_min_, v_min_), cv::Scalar(h_max_, s_max_, v_max_), thres_img);
    cv::inRange(hsv_image_small, cv::Scalar(h_min2_, s_min2_, v_min2_), cv::Scalar(h_max2_, s_max2_, v_max2_), thres_img2);
    
    // perform morphological operations on threshold image to eliminate noise 
    // and emphasize the filtered object(s)
    if (USE_MORPH_OPS)
    {
      morphOps(thres_img);
      morphOps(thres_img2);
    }
    
    
    double x1=0, y1=0, x2=0, y2=0;
    if (TRACK_OBJECTS)
    {
      trackFilteredObject(thres_img, cv_ptr_->image, x1, y1);
      double px1 = x1/thres_img.size().width;
      double py1 = y1/thres_img.size().height;
      x1 = ceil(cv_ptr_->image.size().width * px1)-12;
      y1 = ceil(cv_ptr_->image.size().height * py1)-12;
      // draw object location on screen
      drawObject(x1, y1, cv_ptr_->image);
      trackFilteredObject(thres_img2, cv_ptr_->image, x2, y2);
      double px2 = x2/thres_img.size().width;
      double py2 = y2/thres_img.size().height;
      x2 = ceil(cv_ptr_->image.size().width * px2)-12;
      y2 = ceil(cv_ptr_->image.size().height * py2)-12;
      // draw object location on screen
      drawObject(x2, y2, cv_ptr_->image);
      getAngle(x1, y1, x2, y2, cv_ptr_->image);
    }
    
    // Draw an example circle on the video stream
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    
    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr_->image);
    cv::imshow(OPENCV_WINDOW_HSV, thres_img);
    cv::imshow(OPENCV_WINDOW_HSV2, thres_img2);
    
    cv::waitKey(50);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr_->toImageMsg());
  
  }
  
  
  void searchForMovement(cv::Mat threshold_image, cv::Mat &frame)
  {
    //notice how we use the '&' operator for objectDetected and cameraFeed. This is because we wish
    //to take the values passed into the function and manipulate them, rather than just working with a copy.
    //eg. we draw to the cameraFeed to be displayed in the main() function.
    bool object_detected = false;
    cv::Mat temp;
    threshold_image.copyTo(temp);
    //these two vectors needed for output of findContours
    cv::vector< cv::vector<cv::Point> > contours;
    cv::vector<cv::Vec4i> hierarchy;
    //find contours of filtered image using openCV findContours function
    //findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );// retrieves all contours
    cv::findContours(temp, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);// retrieves external contours

    //if contours vector is not empty, we have found some objects
    if (contours.size() > 0) object_detected = true;
    else object_detected = false;

    if (object_detected)
    {
      //the largest contour is found at the end of the contours vector
      //we will simply assume that the biggest contour is the object we are looking for.
      cv::vector< cv::vector<cv::Point> > largest_contour_vec;
      largest_contour_vec.push_back(contours.at(contours.size()-1));
      //make a bounding rectangle around the largest contour then find its centroid
      //this will be the object's final estimated position.
      object_bounding_rectangle_ = cv::boundingRect(largest_contour_vec.at(0));
      int xpos = object_bounding_rectangle_.x+object_bounding_rectangle_.width/2;
      int ypos = object_bounding_rectangle_.y+object_bounding_rectangle_.height/2;

      //update the objects positions by changing the 'theObject' array values
      the_object_[0] = xpos , the_object_[1] = ypos;
    }
    //make some temp x and y variables so we dont have to type out so much
    int x = the_object_[0];
    int y = the_object_[1];
    
    //draw some crosshairs around the object
    cv::circle(frame, cv::Point(x,y), 20, cv::Scalar(0,255,0), 2);
    cv::line(frame, cv::Point(x,y), cv::Point(x,y-25), cv::Scalar(0,255,0), 2);
    cv::line(frame, cv::Point(x,y), cv::Point(x,y+25), cv::Scalar(0,255,0), 2);
    cv::line(frame, cv::Point(x,y), cv::Point(x-25,y), cv::Scalar(0,255,0), 2);
    cv::line(frame, cv::Point(x,y), cv::Point(x+25,y), cv::Scalar(0,255,0), 2);

    //write the position of the object to the screen
    cv::putText(frame,"Tracking object at (" + intToString(x)+","+intToString(y)+")", cv::Point(x,y),1,1, cv::Scalar(255,0,0),2);
  }
  
  
  void getAngle(double x1, double y1, double x2, double y2, cv::Mat &frame)
  {
    cv::line(frame, cv::Point(x1,y1), cv::Point(x2,y2), CV_RGB(0,255,0), 4, CV_AA);
    //cv::line(camera, cv::Point(x1,y1), cv::Point(camera.size().width,y1), cv::Scalar(100,100,100,100), 4, CV_AA);
    cv::line(frame, cv::Point(x1,y1), cv::Point(x1,frame.size().height), cv::Scalar(100,100,100,100), 4, CV_AA);
    //float xf1 = x1, yf1 = y1, xf2 = x2, yf2 = y2;
    int angle = int(atan2((y1-y2), (x2-x1)) * 180/M_PI) + 90;
    if (angle < 0)
    {
      angle += 360;
    }
    cv::putText(frame, patch::to_string(angle), cv::Point(int(x1) + 50, (int(y2)+int(y1))/2), 1, 2, cv::Scalar(0,255,0), 2);
  }
  
  void trackFilteredObject(cv::Mat &threshold, cv::Mat &frame, double &x, double &y)
  {
    cv::Mat temp = threshold.clone();
    
    // two vectors needed for output of findContours
    cv::vector< cv::vector<cv::Point> > contours;
    cv::vector<cv::Vec4i> hierarchy;
    // find contours of filtered image using openCV findContours functions
    cv::findContours(temp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    // use moments method to find our filtered object
    double ref_area = 0;
    bool object_found = false;
    if (hierarchy.size() > 0) 
    {
      int numObjects = hierarchy.size();
      // if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
      if (numObjects <= MAX_NUM_OBJECTS)
      {
	//int x = 0, y = 0;
	for (int index = 0; index >= 0; index = hierarchy[index][0]) 
	{
	  cv::Moments moment = cv::moments((cv::Mat)contours[index]);
	  double area = moment.m00;
	  
	  // if the area is less than 20px by 20px then it is probably just noise
	  // if the area is the same as the 3/2 of the image size, probably just a bad filter
	  // we only want the object with the largest area so we safe a reference area each
	  // iteration and compare it to the area in the next iteration.
	  if (area > MIN_OBJECT_AREA && area < MAX_OBJECT_AREA && area > ref_area)
	  {
	    x = moment.m10/area;
	    y = moment.m01/area;
	    object_found = true;
	    ref_area = area;
	  }
	  /*else
	  {
	    objectFound = false;
	  }*/
	}
	// let user know you found an object
	if (object_found)
	{
	  cv::putText(frame, "Tracking Object", cv::Point(0,50), 2, 1, cv::Scalar(0,255,0), 2);
	}
	else
	{
	  cv::putText(frame, "TOO MUCH NOISE! ADJUST FILTER", cv::Point(0,50), 1, 2, cv::Scalar(0,0,255), 2);
	}
      }
    }
  }
  
  void drawObject(int x, int y, cv::Mat &frame)
  {
    // use some of the openCV drawing functions to draw crosshairs
    // on your tracked image!
    
    // if and else statements to prevent memory errors from
    // writing off the screen (ie. (-25,-25) is not within the window!)
    
    cv::circle(frame, cv::Point(x,y), 20, cv::Scalar(0,255,0), 2);
    
    if (y-25 > 0) 
      cv::line(frame, cv::Point(x,y), cv::Point(x,y-25), cv::Scalar(0,255,0), 2);
    else
      cv::line(frame, cv::Point(x,y), cv::Point(x,0), cv::Scalar(0,255,0), 2);
    if (y+25 < FRAME_HEIGHT)
      cv::line(frame, cv::Point(x,y), cv::Point(x,y+25), cv::Scalar(0,255,0), 2);
    else 
      cv::line(frame, cv::Point(x,y), cv::Point(x,FRAME_HEIGHT), cv::Scalar(0,255,0),2);
    if (x-25 > 0)
      cv::line(frame, cv::Point(x,y), cv::Point(x-25,y), cv::Scalar(0,255,0), 2);
    else 
      cv::line(frame, cv::Point(x,y), cv::Point(0,y), cv::Scalar(0,255,0),2);
    if( x+25 < FRAME_WIDTH)
      cv::line(frame, cv::Point(x,y), cv::Point(x+25,y), cv::Scalar(0,255,0), 2);
    else 
      cv::line(frame, cv::Point(x,y), cv::Point(FRAME_WIDTH,y), cv::Scalar(0,255,0), 2);

    cv::putText(frame, intToString(x) +"," + intToString(y), cv::Point(x,y+30), 1, 1, cv::Scalar(0,255,0), 2);     
  }
  
  std::string intToString(int number)
  {
    std::stringstream ss;
    ss << number;
    return ss.str();
  }
  
  void morphOps(cv::Mat &threshold)
  {
    // create structuring element that will be used to dilate and erode image
    // the element chosen is a 3px by 3px rectangle
    cv::Mat erode_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2,2));
    // dilate with larger element so make sure object is nicely visible
    cv::Mat dilate_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2,2)); //8,8
    
    cv::erode(threshold, threshold, erode_kernel);
    cv::dilate(threshold, threshold, dilate_kernel);
    cv::dilate(threshold, threshold, dilate_kernel);
    cv::erode(threshold, threshold, erode_kernel);
    cv::dilate(threshold, threshold, dilate_kernel);
    cv::dilate(threshold, threshold, dilate_kernel);
    cv::dilate(threshold, threshold, dilate_kernel);
  }
 
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  std::cout<<"Test";

  ImageConverter ic;
  ros::spin();
  
  return 0;
}