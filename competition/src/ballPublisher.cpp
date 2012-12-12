#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <competition/BallsMessage.h>
#include <competition/removeBall.h>
#include <competition/Balls.h>

#include "common/functions.h"

const uint8_t RED {0};
const uint8_t GREEN {1};

const double MIN_DIST_BETWEEN_BALLS {0.15}; //TODO Check in robo room.
const double PUBLISH_DELAY {0.1};

class BallPublisher {
public:
  BallPublisher();
  void init();
  
private:
  void visibleBallsCallback(const competition::BallsMessage& seenBalls);
  void publishTimerCallback(const ros::TimerEvent& event);
  bool ballRemoveService(competition::removeBall::Request& req, competition::removeBall::Response& res);
  bool redBallsService(competition::Balls::Request& req, competition::Balls::Response& res);
  
  ros::NodeHandle nh_;
  ros::Publisher redBallsPub_;
  ros::Publisher pointCloudPub_;
  ros::Subscriber ballsSub_;
  ros::ServiceServer ballRemoveSrv_;
  ros::ServiceServer redBallsSrv_;
  ros::Timer publishTimer_;
  
  std::vector<competition::Ball> redBalls_;
  std::vector<competition::Ball> greenBalls_;
};


BallPublisher::BallPublisher()
{}


void BallPublisher::init()
{
  redBallsPub_ = nh_.advertise<competition::BallsMessage> ("red_balls", 1, true);
  pointCloudPub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("green_balls", 1, false);
  
  ballsSub_ = nh_.subscribe("visible_balls", 10, &BallPublisher::visibleBallsCallback, this);
  ballRemoveSrv_ = nh_.advertiseService("remove_red_ball", &BallPublisher::ballRemoveService, this);
  redBallsSrv_ = nh_.advertiseService("red_balls", &BallPublisher::redBallsService, this);
  
  publishTimer_ = nh_.createTimer(ros::Duration (PUBLISH_DELAY), &BallPublisher::publishTimerCallback, this, false, true);
}


bool BallPublisher::ballRemoveService (competition::removeBall::Request& req, competition::removeBall::Response& res)
{
  std::vector<competition::Ball>::iterator iter = redBalls_.begin();
  bool removed {false};
  while (iter != redBalls_.end())
  {
    if (competition::distanceBetweenPoints(req.ballToRemove.location, iter->location) < MIN_DIST_BETWEEN_BALLS)
    {
      redBalls_.erase(iter);
      removed = true;
      break;
    }
  }
  res.success = removed;
  return true;
}


bool BallPublisher::redBallsService (competition::Balls::Request& req, competition::Balls::Response& res)
{
  competition::BallsMessage ballsMsg;
  ballsMsg.header.frame_id = "map";
  ballsMsg.header.stamp = ros::Time::now();
  ballsMsg.balls = redBalls_;
  res.balls = ballsMsg;
  return true;
}


void BallPublisher::publishTimerCallback (const ros::TimerEvent& event)
{
  //ROS_INFO("Publishing red and green balls");
  competition::BallsMessage redMsg;
  redMsg.header.frame_id = "/map";
  redMsg.header.stamp = ros::Time::now();
  redMsg.balls = redBalls_;
  redBallsPub_.publish(redMsg);
  
  pcl::PointCloud<pcl::PointXYZ> greenMsg;
  greenMsg.header.frame_id = "/map";
  greenMsg.header.stamp = ros::Time::now();
  greenMsg.height = 1;
  greenMsg.width = greenBalls_.size();
  for (competition::Ball ball : greenBalls_)
  {
    pcl::PointXYZ point(ball.location.x,ball.location.y,0.0);
    /*point.x = ball.location.x;
    point.y = ball.location.y;*/
    greenMsg.points.push_back(point);
  }
  pointCloudPub_.publish(greenMsg);
}


void checkNewBalls(std::vector<competition::Ball>& mainList, std::vector<competition::Ball>& newList)
{
  for (competition::Ball ball : newList)
  {
    bool newBall {true};
    for (competition::Ball oldBall : mainList)
    {
      ROS_INFO("Distance between balls: %f", competition::distanceBetweenPoints(oldBall.location, ball.location));
      if (competition::distanceBetweenPoints(oldBall.location, ball.location) < MIN_DIST_BETWEEN_BALLS)
      {
        oldBall.location = ball.location;
        newBall = false;
        break;
      }
    }
    if (newBall)
      mainList.push_back(ball);
  }
}

void BallPublisher::visibleBallsCallback (const competition::BallsMessage& seenBalls)
{
  ROS_INFO("Received list of current balls: %i", seenBalls.balls.size());
  // Separate
  std::vector<competition::Ball> newGreens;
  std::vector<competition::Ball> newReds;
  
  for (competition::Ball ball : seenBalls.balls)
  {
    if (ball.color == RED)
      newReds.push_back(ball);
    else
      newGreens.push_back(ball);
  }
  
  checkNewBalls(greenBalls_, newGreens);
  checkNewBalls(redBalls_, newReds);
  
  ROS_INFO("Balls on green list: %d", greenBalls_.size());
  ROS_INFO("Balls on red list: %d", redBalls_.size());
}



int main(int argc, char** argv){
  ros::init(argc, argv, "ballPublisher");
  BallPublisher ballPublisher;
  ballPublisher.init();
  
  ROS_INFO("Ball publisher node started");
  
  ros::spin();
  
  return 0;
}
