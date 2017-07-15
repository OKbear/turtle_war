#include "ros/ros.h"
#include <termios.h>
#include <stdio.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <nav_msgs/Odometry.h>

#define TRIAL_TO_RETREAT 25
#define PERIOD_COLLISION_RELEASE 100
#define TRIAL_TO_DETERMINE_LOCKED 20
#define LOCKED_THRESHOLD 0.1
#define DAKOU_RIGHT 0
#define DAKOU_LEFT 1
#define PRIOD_DAKOU 15

class RoboCtrl
{
private:
	enum EState
	{
		STATE_IDLE    = 0,
		STATE_RUN     = 1,
		STATE_RETREAT = 2,
		STATE_LOCKED  = 3,
	};

public:
	RoboCtrl()
	{
		ros::NodeHandle node;
		//購読するtopic名
		odom_sub_ = node.subscribe("odom", 1, &RoboCtrl::odomCallback, this);
		bumper_sub_ = node.subscribe("/mobile_base/events/bumper", 1, &RoboCtrl::bumperCallback, this);

		//配布するtopic名
		twist_pub_ = node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
		//twist_pub_ = node.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop/", 1);

		//内部関数初期化
		m_frontspeed = 1.0;
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
	}

	void moveRobo()
	{
		//速度データ型宣言
		geometry_msgs::Twist twist;

		switch( m_state ){
			case STATE_RUN:
				m_frontspeed = 1.0;
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
					m_frontspeed = 1.0;
					m_turnspeed = 0.0;
					m_state = STATE_RUN;
					m_collisionRelease = 0;
				}
				break;

			case STATE_LOCKED:
				break;
		}

		ROS_INFO("NOW %d", m_state);

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

				m_frontspeed = -1.0;
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
		ros::Subscriber odom_sub_;
		ros::Subscriber bumper_sub_;
		ros::Publisher twist_pub_;

		EState m_state;

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
