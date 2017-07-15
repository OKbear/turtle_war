#include "ros/ros.h"
#include <termios.h>
#include <stdio.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <kobuki_msgs/BumperEvent.h>
#include <nav_msgs/Odometry.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

static const std::string OPENCV_WINDOW = "Image window";

#define TRIAL_TO_RETREAT 25
#define PERIOD_COLLISION_RELEASE 100
#define TRIAL_TO_DETERMINE_LOCKED 20
#define LOCKED_THRESHOLD 0.1
#define DAKOU_RIGHT 0
#define DAKOU_LEFT 1
#define PRIOD_DAKOU 15
#define GAIN_CHASE -0.01

class RoboCtrl
{
private:
	enum EState
	{
		STATE_IDLE    = 0,
		STATE_RUN     = 1,
		STATE_RETREAT = 2,
		STATE_LOCKED  = 3,
		STATE_CHASE   = 4,
		STATE_LOST    = 5,
	};

public:
	RoboCtrl() : it_(node)
	{
		ros::NodeHandle node;
		//購読するtopic名
		odom_sub_ = node.subscribe("odom", 1, &RoboCtrl::odomCallback, this);
		bumper_sub_ = node.subscribe("/mobile_base/events/bumper", 1, &RoboCtrl::bumperCallback, this);
		image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &RoboCtrl::imageCb, this);
		//Laser_sub_ = node.subscribe<sensor_msgs::LaserScan>("/scan", 100, &RoboCtrl::LaserScan, this);

		//配布するtopic名
		//twist_pub_ = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
		twist_pub_ = node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
		//内部関数初期化
		m_frontspeed = 0.5;
		m_turnspeed = 0.0;

		m_state = STATE_RUN;
		m_measVel = 0.0;

		// 蛇行用
		m_dakou = DAKOU_RIGHT;
		m_nPriodDakou = 0;

		// 衝突用
		m_nRetreat = 0;

		// ロック解除用
		m_collisionRelease = 0;

		// ロック解除用
		//m_nLockedDtmCnt = 0;

		cv::namedWindow(OPENCV_WINDOW);
	}

	~RoboCtrl()
	{
		cv::destroyWindow(OPENCV_WINDOW);
	}

	void moveRobo()
	{
		//速度データ型宣言
		geometry_msgs::Twist twist;

		switch( m_state ){
			case STATE_RUN:
				m_frontspeed = 0.5;
				ROS_INFO("DAKOU %d", m_dakou);
				if( ++m_nPriodDakou >= PRIOD_DAKOU ){
					if( m_dakou == DAKOU_RIGHT ){
						m_dakou = DAKOU_LEFT;
						m_turnspeed = 1.8;
					} else {
						m_dakou = DAKOU_RIGHT;
						m_turnspeed = -1.8;
					}
					m_nPriodDakou = 0;
				}
				if( ++m_collisionRelease >= PERIOD_COLLISION_RELEASE ){
					m_state = STATE_RETREAT;
					m_nRetreat = TRIAL_TO_RETREAT;
					m_frontspeed = -0.5;
					m_turnspeed = 1.8;
					m_collisionRelease = 0;
				}
				break;

			case STATE_RETREAT:
				if ( --m_nRetreat == 0 ){
					m_frontspeed = 0.5;
					m_turnspeed = 0.0;
					m_state = STATE_RUN;
					m_collisionRelease = 0;
				}
				break;

			case STATE_LOCKED:
				break;

			case STATE_CHASE:
				if( m_diffPos == 0.0 ) {
					m_state = STATE_LOST;
				} else {
					m_frontspeed = 0.5;
					m_turnspeed = m_diffPos * GAIN_CHASE;
				}
				break;
				case STATE_LOST:
						m_frontspeed = 0.0;
						m_turnspeed = 3.0;
				break;

		}

		ROS_INFO("NOW %d", m_state);
		ROS_INFO("diff %f", m_diffPos);

		//ROS速度データに内部関数値を代入
		twist.linear.x = m_frontspeed;
		twist.angular.z = m_turnspeed;

		twist_pub_.publish(twist);
	}

	void odomCallback(const nav_msgs::Odometry &odom)
	{

		/*
		double vx = odom.twist.twist.linear.x;
		double vy = odom.twist.twist.linear.y;
		double vz = odom.twist.twist.linear.z;

		if( vx*vx + vy*vy + vz*vz < LOCKED_THRESHOLD )
		{
			if( ++m_nLockedDtmCnt >= TRIAL_TO_DETERMINE_LOCKED ){
				m_state = STATE_LOCKED;
				m_nLockedDtmCnt = 0;

				m_frontspeed = -0.5;
				m_turnspeed  = -1.0;
			}
		} else {
			if( m_state == STATE_LOCKED){
				m_state = STATE_RUN;
			}
			m_nLockedDtmCnt = 0;
		}
		*/
		return;
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		const int centerpnt = 320;
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		cv::cvtColor(cv_ptr->image,hsv,CV_BGR2HSV);
		cv::inRange(hsv, cv::Scalar(0, 100, 100), cv::Scalar(30, 255, 255), mask); // 色検出でマスク画像の作成
		//cv::bitwise_and(cv_ptr->image,mask,image);

		cv::Moments mu = cv::moments( mask, false );
		cv::Point2f mc = cv::Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );

		int x = mu.m10/mu.m00;
		int y = mu.m01/mu.m00;

		if( (x >= 0) && (x <= 640) && (m_state != STATE_RETREAT)){
			m_diffPos = x - centerpnt;
			m_state = STATE_CHASE;
		} else {
			m_diffPos = 0;
		}

		//ROS_INFO("obj x=%d y=%d",x,y);

		// Update GUI Window
		cv::imshow(OPENCV_WINDOW, mask);
		cv::waitKey(3);
	}
	void LaserScan(const sensor_msgs::LaserScanConstPtr& Laser)
	{
		int image_width = 640;
		int image_height = 480;
		int line_thickness = 5;
		int frame_number = (Laser->angle_max - Laser->angle_min)/Laser->angle_increment + 1;
		double pixelsize;
		cv::Mat map_img(cv::Size(image_width, image_height), CV_8UC3, cv::Scalar::all(255));  // cv::Scalar(0,0,255)(B、G、R)
		pixelsize = Laser->range_max / image_height;

		double angle,length,pointX,pointY;

		//write right data
		for(int i = 0; i < frame_number; i++)
		{
			//angle = 3.14/2 + (Laser->angle_min - MOUNTING_ANGLE_RIGHT/180.0*3.14 + Laser->angle_increment * i);
			angle = 3.14/2 + (Laser->angle_min + Laser->angle_increment * i);
			length = Laser->ranges[i]/pixelsize;
			pointX = image_width/2 + length*cos(angle);
			//pointX = IMAGE_WIDTH/2 + (length*cos(angle) + MOUNTING_DISTANCE_RIGHT/pixelsize);
			pointY = image_height/2 - length*sin(angle);

			if (pointX > 0 && (Laser->ranges[i]>0.0))
			{
				cv::rectangle(map_img, cvPoint(int(pointX),int(pointY)), cvPoint(int(pointX)+line_thickness,int(pointY)+line_thickness), CV_RGB(255,0,0), CV_FILLED, 8, 0);
			}

		}
		cv::imshow("Laserscan", map_img);
		cv::waitKey(3);
	}


	void bumperCallback(const kobuki_msgs::BumperEvent &bumper)
	{
		if (bumper.state == 1) {
			ROS_INFO("HIT");
			m_frontspeed = -0.5;
			switch( bumper.bumper ) {
				case 0:		// Left hit
					m_turnspeed  = 1.8;
					break;

				case 1:		// Front hit
					m_turnspeed  = -1.8;
					break;

				case 2:		// Right hit
					m_turnspeed  = -1.8;
					break;
			}
			m_state = STATE_RETREAT;
			m_nRetreat = TRIAL_TO_RETREAT;
		}
	}

	private:
		ros::NodeHandle node;
		ros::Subscriber odom_sub_;
		ros::Subscriber bumper_sub_;
		ros::Subscriber Laser_sub_;
		ros::Publisher twist_pub_;

		//ROS時間
		ros::Time push_time_;
		//ROS変化秒
		ros::Duration under_time_;

		cv::Mat hsv;
		cv::Mat mask;
		cv::Mat image;

		image_transport::ImageTransport it_;
		image_transport::Subscriber image_sub_;

		EState m_state;

		// 対象データ
		double m_diffPos;

		// 蛇行用
		int m_dakou;
		int m_nPriodDakou;

		// 壁衝突用
		int m_nRetreat;

		// ロック解除用
		//int m_nLockedDtmCnt;

		// たまにロック解除
		int m_collisionRelease;

		double m_frontspeed;
		double m_turnspeed;

		double m_measVel;
};

int main(int argc, char **argv)
{
	//ROSのノード初期化
	ros::init(argc, argv, "robo_ctrl");
	RoboCtrl robo_ctrl;
	ros::Rate r(20);

	while (ros::ok())
	{
		robo_ctrl.moveRobo();
		ros::spinOnce();
		r.sleep();
	}
}
