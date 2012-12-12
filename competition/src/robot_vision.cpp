#include <vector>
#include <math.h>
#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <competition/Ball.h>
#include <competition/BallsMessage.h>
#include <competition/centerSrv.h>

#define PI 3.14159265

namespace enc = sensor_msgs::image_encodings;

class ImageHandler 
{
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::ServiceServer centerSrv_;
  IplImage latestImg;
  int counter_;
	

public:
	ImageHandler()
  	  : it_(nh_)
  	{
    		image_sub_ = it_.subscribe("camera/rgb/image_color", 1, &ImageHandler::imageCb, this);
		pub_ = nh_.advertise<competition::BallsMessage> ("visible_balls", 10, true);
		centerSrv_ = nh_.advertiseService("alignment", &ImageHandler::centerSrv, this);
		/*cv::namedWindow("Camera");
		cv::namedWindow("Red");
		cv::namedWindow("Green");*/
		
  	} 

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
	  if (counter_ < 9) {
	    counter_++;
	    return;
	  }
		counter_ = 0;
		//Get image from camera    		
		cv_bridge::CvImagePtr cv_ptr;
    		try
    		{
      			cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    		}
    		catch (cv_bridge::Exception& e)
    		{
      			ROS_ERROR("cv_bridge exception: %s", e.what());
      			return;
    		}
    		latestImg = cv_ptr->image;
    		//cv::imshow("Camera", cv_ptr->image);
    		
		//Get transform
		tf::StampedTransform transform = getTransform();
		
    		IplImage img = cv_ptr->image;
		//cv::Mat img = cv_ptr->image;
		std::vector<cv::KeyPoint> Rballs = findObjects(&img, 0);
		std::vector<cv::KeyPoint> Gballs = findObjects(&img, 1);
		pubLocations(transform, Rballs, Gballs);
  	}

	std::vector<cv::KeyPoint> findObjects(IplImage *imageBGR, int color) //TODO: Add blue area
	{
		//Create HSV image
		IplImage *imageHSV = cvCreateImage(cvGetSize(imageBGR), 8, 3);
		
		//Convert color from BGR to HSV
		cvCvtColor(imageBGR, imageHSV, CV_BGR2HSV);
		
		//Create planes for H, S and V
		IplImage *planeH = cvCreateImage(cvGetSize(imageBGR), 8, 1);
		IplImage *planeS = cvCreateImage(cvGetSize(imageBGR), 8, 1);	
		IplImage *planeV = cvCreateImage(cvGetSize(imageBGR), 8, 1);
	
		//Cut imageHSV in to the planes
		cvCvtPixToPlane(imageHSV, planeH, planeS, planeV, 0);
	
		//Apply filtering for correct color pixels and save in color specific planes
		if(color==0) {	
			//Red
			cvInRangeS(planeH, cv::Scalar::all(10), cv::Scalar::all(170), planeH);
			cvNot(planeH, planeH);
			cvThreshold(planeS, planeS, 60, UCHAR_MAX, CV_THRESH_BINARY);
			cvThreshold(planeV, planeV, 60, UCHAR_MAX, CV_THRESH_BINARY);
		} else if(color==1) {
			//Green
			cvInRangeS(planeH, cv::Scalar::all(40), cv::Scalar::all(65), planeH);
			cvThreshold(planeS, planeS, 60, UCHAR_MAX, CV_THRESH_BINARY);
			cvThreshold(planeV, planeV, 60, UCHAR_MAX, CV_THRESH_BINARY);
		}	

		//Combine H, S and V layers
		IplImage* image = cvCreateImage(cvGetSize(imageBGR), 8, 1);
		cvAnd(planeH, planeS, image);
		cvAnd(image, planeV, image);
	
		//Create Mat type image for blob detection
		cv::Mat imageBlob(image);
	
		//Set up the Red blob detector parameters
		cv::SimpleBlobDetector::Params params;
		params.minDistBetweenBlobs = 25.0f;
		params.filterByInertia = false;
		params.filterByConvexity = false;
		params.filterByColor = true;
	        params.blobColor = 255;
		params.filterByCircularity = false;
		params.filterByArea = true;
		if(color==0) {
			params.minArea = 25.0f;			
		} else if(color==1) {
			params.minArea = 25.0f;
		}
		params.maxArea = 10000.0f;
		
		//Set up and create detector
		cv::Ptr<cv::FeatureDetector> blob_detector = new cv::SimpleBlobDetector(params);
		blob_detector->create("SimpleBlob");
	
		//Detect objects
		std::vector<cv::KeyPoint> keypoints;
		blob_detector->detect(imageBlob, keypoints);

		//Draw keypoints for testing purposes
		cv::drawKeypoints(imageBlob, keypoints, imageBlob, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	
		if(color==0) {
			cv::imshow("Red", imageBlob);
			for(int i = 0; i < keypoints.size(); i++) {
				std::cout << "Red ball " << i << ": ( " << keypoints[i].pt.x << " , " << keypoints[i].pt.y << " )" << std::endl;
			}
		}
		else if(color==1) {
			cv::imshow("Green", imageBlob);
			for(int i = 0; i < keypoints.size(); i++) {
				std::cout << "Green ball " << i << ": ( " << keypoints[i].pt.x << " , " << keypoints[i].pt.y << " )" << std::endl;
			}
		}
		
		cvReleaseImage(&imageHSV);
		cvReleaseImage(&planeH);
		cvReleaseImage(&planeS);
		cvReleaseImage(&planeV);
		cvReleaseImage(&image);
		return keypoints;
	}

	tf::StampedTransform getTransform()
	{	
		//Get the position of the robot from the /map -> /base_link transform
		tf::TransformListener listener;
	
		tf::StampedTransform transform;
		try {
			listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
		}
		catch (tf::TransformException ex) {
			ROS_ERROR("%s", ex.what());
		}
		
		return transform;
	}
	
	
  void pubLocations(tf::StampedTransform transform, std::vector<cv::KeyPoint> keypointsR, std::vector<cv::KeyPoint> keypointsG)
  {
    
    //Calculate object locations based on positions in image and measured variables from robot
    //Define fixed variables
    float camYmax = 45.0;
    float camXmax = 56.0;
    float imgYmax = 480.0;
    float imgXmax = 640.0;
    float camAng = 25.0;
    float camDgrd = 0.3;
    float camDaxl = 0.1;
    
    //Create new BallsMessage object
    competition::BallsMessage Balls;
    Balls.header.frame_id = "map";
    Balls.header.stamp = ros::Time::now();
    
    for(int i = 0; i < keypointsR.size(); i++)
    {
      //Calculate object vertical angle from centre of camera
      float ya = imgYmax / 2 - (imgYmax - keypointsR[i].pt.y);
      float yb = imgYmax / 2;
      float alpha = ya / yb * camYmax / 2;
      std::cout << "yAlpha: " << alpha << std::endl;
      
      //Calculate angle between object and robot vertical axis, and calculate distance on the ground
      float beta = 90.0 - alpha - camAng;
      float yDist = tan(beta*PI/180) * camDgrd;
      std::cout << "yDist: " << yDist << std::endl;

      //Calculate object horizontal angle from centre of camera
      float xa = imgXmax / 2 - keypointsR[i].pt.x;
      float xb = imgXmax / 2;
      float gamma = xa / xb * camXmax / 2;
      std::cout << "xGamma: " << gamma << std::endl;

      //Calculate x-axis distance from centre of robot
      float xDist = tan(gamma*PI/180) * yDist;
      std::cout << "xDist: " << xDist << std::endl;
      //Calculate distance and angle in comparison to robot centre of axis
      yDist = yDist + camDaxl;
      float bDist = sqrt(xDist*xDist + yDist*yDist);
      float bAng = atan(xDist/(yDist+camDaxl)); // * 180 / PI;
      
       //Create new Ball
      competition::Ball ball;
      ball.color = 0;
      ball.location.x = bDist * cos(bAng + tf::getYaw(transform.getRotation())) + transform.getOrigin().x();
      ball.location.y = bDist * sin(bAng + tf::getYaw(transform.getRotation())) + transform.getOrigin().y();
      ball.location.z = 0;
      Balls.balls.push_back(ball);
      
      std::cout << "Red ball at " << bDist << " at angle " << bAng*180/PI << ", robot angle is " << tf::getYaw(transform.getRotation()) << std::endl;
      
    }

    for(int i = 0; i < keypointsG.size(); i++)
    {
      //Calculate object vertical angle from centre of camera
      float ya = imgYmax / 2 - (imgYmax - keypointsG[i].pt.y);
      float yb = imgYmax / 2;
      float alpha = ya / yb * camYmax / 2;
	
      //Calculate angle between object and robot vertical axis, and calculate distance on the ground
      float beta = 90.0 - alpha - camAng;
      float yDist = tan(beta*PI/180) * camDgrd;

      //Calculate object horizontal angle from centre of camera
      float xa = imgXmax / 2 - keypointsG[i].pt.x;
      float xb = imgXmax / 1;
      float gamma = xa / xb * camXmax / 2;

      //Calculate x-axis distance from centre of robot
      float xDist = tan(gamma*PI/180) * yDist;
      //Calculate distance and angle in comparison to robot centre of axis
      yDist = yDist + camDaxl;
      float bDist = sqrt(xDist*xDist + yDist*yDist);
      float bAng = atan(xDist/(yDist+camDaxl)); // * 180 / PI;
      
      //Create new Ball
      competition::Ball ball;
      ball.color = 1;
      ball.location.x = bDist * cos(bAng + tf::getYaw(transform.getRotation())) + transform.getOrigin().x();
      ball.location.y = bDist * sin(bAng + tf::getYaw(transform.getRotation())) + transform.getOrigin().y();
      ball.location.z = 0;
      Balls.balls.push_back(ball);
      
      std::cout << "Green ball at " << bDist << " at angle " << bAng*180/PI << std::endl;
    }
    
    //Testing
    for(int i = 0; i < Balls.balls.size(); i++)
    {
      std::cout << Balls.balls[i].color << " " << Balls.balls[i].location.x << " " << Balls.balls[i].location.y << std::endl;
    }
    
    pub_.publish(Balls);
    //cv::waitKey(0);
  }
  
  //Check location of closest red ball
  bool centerSrv(competition::centerSrv::Request &req, competition::centerSrv::Response &res)
  {
    std::vector<cv::KeyPoint> Rballs = findObjects(&latestImg, 0);
    float cDist;
    float cAng;
    competition::Ball cBall;
    
    //Calculate object locations based on positions in image and measured variables from robot
    //Define fixed variables
    float camYmax = 45.0;
    float camXmax = 56.0;
    float imgYmax = 480.0;
    float imgXmax = 640.0;
    float camAng = 25.0;
    float camDgrd = 0.27;
    float camDaxl = 0.13;
    
    tf::StampedTransform transform = getTransform();
    
    
    
    for(int i = 0; i < Rballs.size(); i++)
    {
      //Calculate object vertical angle from centre of camera
      float ya = imgYmax / 2 - Rballs[i].pt.y;
      float yb = imgYmax / 2;
      float alpha = ya / yb * camYmax / 2;
      std::cout << "yAlpha: " << alpha << std::endl;
      
      //Calculate angle between object and robot vertical axis, and calculate distance on the ground
      float beta = 90.0 - alpha - camAng;
      float yDist = tan(beta*PI/180) * camDgrd;
      std::cout << "yDist: " << yDist << std::endl;

      //Calculate object horizontal angle from centre of camera
      float xa = imgXmax / 2 - Rballs[i].pt.x;
      float xb = imgXmax / 2;
      float gamma = xa / xb * camXmax / 2;
      std::cout << "xGamma: " << gamma << std::endl;

      //Calculate x-axis distance from centre of robot
      float xDist = tan(gamma*PI/180) * yDist;
      std::cout << "xDist: " << xDist << std::endl;
      //Calculate distance and angle in comparison to robot centre of axis
      yDist = yDist + camDaxl;
      float bDist = sqrt(xDist*xDist + yDist*yDist);
      float bAng = atan(xDist/(yDist+camDaxl)); // * 180 / PI;
      
       //Create new Ball
      competition::Ball ball;
      ball.color = 0;
      ball.location.x = bDist * cos(bAng + tf::getYaw(transform.getRotation())) + transform.getOrigin().x();
      ball.location.y = bDist * sin(bAng + tf::getYaw(transform.getRotation())) + transform.getOrigin().y();
      ball.location.z = 0;
      
      if (i == 0) {
	cDist = bDist;
	cAng = bAng;
	cBall = ball;
      }
      else if(cDist > bDist) {
	cDist = bDist;
	cAng = bAng;
	cBall = ball;
      }
      
      if (cDist > 1) {
	cDist = -1;
      }
      else {
	res.distance = cDist;
	res.angle = cAng;
	res.ball = cBall;
      }
      
      return true;
    }    
  }
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_handler");
  	ImageHandler ih;
  	ros::spin();	
	return 0;
}

//Save position variables for later use
//float xWorld = transform.getOrigin().x;
//float yWorld = transform.getOrigin().y;
//float aWorld = transform.getOrigin().yaw;         

//Draw locations of keypoints on to threshold images
//cv::drawKeypoints(imageBlobR, keypointsR, imageBlobR, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
//cv::drawKeypoints(imageBlobG, keypointsG, imageBlobG, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

//Display blob detection images
//cvNamedWindow("Red", CV_WINDOW_AUTOSIZE);
//cvNamedWindow("Green", CV_WINDOW_AUTOSIZE);
//cv::imshow("Red", imageBlobR);
//cv::imshow("Green", imageBlobG);

//std::cout << keypointsR[0].pt.x << " " << keypointsR[0].pt.y << std::endl;
//cvShowImage("Test", imageBlob);
//cv::waitKey(0);
	
//cv::Mat getImg()
//{
	//Load image and create empty copy of same size
	//TODO: Change to load image from camera topic in to Mat variable and make IplImage pointer for it
	//IplImage *imageBGR = cvLoadImage("/home/ismo/fuerte_workspace/pioneer-dreamteam/balls2.jpg", CV_LOAD_IMAGE_COLOR);
	//IplImage *imageHSV = cvCreateImage(cvGetSize(imageBGR), 8, 3);
	//return Mat(imageHSV);
//}

