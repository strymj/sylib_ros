/**
* @file MoveBaseGoal.h
* @brief sending ROS move_base library
* @author strymj
* @date 2017.05
*/

#ifndef MOVEBASEGOAL_H_
#define MOVEBASEGOAL_H_

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_listener.h>

/** @brief sylib namespace */
namespace sy
{
	/** @brief move_base goal class */
	class MoveBaseGoal
	{
		public:

			MoveBaseGoal();
			MoveBaseGoal(std::string csv_path);

			/** @brief structure for move_base goal. */
			struct Goal
			{/*{{{*/
				std::string goal_id;           /**< goal id */
				std::string frame_id;          /**< frame id */
				geometry_msgs::Pose2D pose2d;  /**< goal pose */

				Goal();
				Goal(std::string tag, std::string frame, geometry_msgs::Pose2D& pos);
				Goal(std::string tag, std::string frame, double x, double y, double theta);
			};/*}}}*/

			void loadCsvGoal(std::string csv_path);
			void setGoal(std::string tag, std::string frame, geometry_msgs::Pose2D& pos);
			void setGoal(std::string tag, std::string frame, double x, double y, double theta);
			bool makeNextGoal(move_base_msgs::MoveBaseGoal& move_base_goal);
			std::string getGoalID();
			double getGoalDistance(tf::StampedTransform& st);


		private:

			/** @brief goal list. goals are called from this list. */
			std::vector<Goal> goal_list;
			/** @brief now goal number */
			int now_goal_num;


	};  // class MoveBaseGoal

}  // namespace sy

#endif
