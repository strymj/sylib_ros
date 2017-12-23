/**
* @file move_base_goal.cpp
* @brief sending ROS move_base library
* @author strymj
* @date 2017.05
*/

#include <sylib_ros/MoveBaseGoal.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <sstream>

using namespace std;
using namespace sy;


/**
 * @brief constructer 
 */
MoveBaseGoal::MoveBaseGoal()
{/*{{{*/
	now_goal_num = -1;
}/*}}}*/


/**
 * @brief constructer. This function can load goal list from csv file. 
 * @param csv_path   goal csv file path. csv format must be "tag_name, goal_id, x, y, theta".
 */
MoveBaseGoal::MoveBaseGoal(string csv_path)
{/*{{{*/
	now_goal_num = -1;
	loadCsvGoal(csv_path);
}/*}}}*/


/**
 * @brief constructer 
 */
MoveBaseGoal::Goal::Goal(){}


/**
 * @brief constructer 
 * @param tag   goal tag.
 * @param frame   goal frame_id.
 * @param pos   goal pose.
 */
MoveBaseGoal::Goal::Goal(string tag, string frame, geometry_msgs::Pose2D& pos)
{/*{{{*/
	goal_id = tag;
	frame_id = frame;
	pose2d = pos;
}/*}}}*/


/**
 * @brief constructer 
 * @param tag   goal tag.
 * @param frame   goal frame_id.
 * @param x   goal position x.
 * @param y   goal position y.
 * @param theta   goal rotation theta.
 */
MoveBaseGoal::Goal::Goal(string tag, string frame, double x, double y, double theta)
{/*{{{*/
	goal_id = tag;
	frame_id = frame;
	pose2d.x = x;
	pose2d.y = y;
	pose2d.theta = theta;
}/*}}}*/


/**
 * @brief make next goal from goal csv list.
 * @param move_base_goal   next goal will set in this argument.
 * @return If there are no goals to set next, this function returns false. (But the argument returns 1st goal.)
 */
bool MoveBaseGoal::makeNextGoal(move_base_msgs::MoveBaseGoal& move_base_goal)
{/*{{{*/
	bool ans = false;
	if (now_goal_num+1 < goal_list.size())
	{
		++now_goal_num;
		ans = true;
	}
	else
	{
		now_goal_num = 0;
		ans = false;
	}

	Goal goal = goal_list[now_goal_num];
	move_base_goal.target_pose.header.frame_id = goal.frame_id;
	move_base_goal.target_pose.header.stamp = ros::Time::now();
	move_base_goal.target_pose.pose.position.x = goal.pose2d.x;
	move_base_goal.target_pose.pose.position.y = goal.pose2d.y;
	move_base_goal.target_pose.pose.position.z = 0.0;
	geometry_msgs::Quaternion quaternion;
	quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, goal.pose2d.theta);
	move_base_goal.target_pose.pose.orientation = quaternion;

	return ans;
}/*}}}*/


/**
 * @brief add goal at the end of goal list.
 * @param tag   goal tag.
 * @param frame   goal frame_id.
 * @param pos   goal pose.
 */
void MoveBaseGoal::setGoal(string tag, string frame, geometry_msgs::Pose2D& pos)
{/*{{{*/
	Goal goal(tag, frame, pos);
	goal_list.push_back(goal);
}/*}}}*/


/**
 * @brief add goal at the end of goal list.
 * @param tag   goal tag.
 * @param frame   goal frame_id.
 * @param x   goal position x.
 * @param y   goal position y.
 * @param theta   goal rotation theta.
 */
void MoveBaseGoal::setGoal(string tag, string frame, double x, double y, double theta)
{/*{{{*/
	Goal goal(tag, frame, x, y, theta);
	goal_list.push_back(goal);
}/*}}}*/


/** @fn MoveBaseGoal::getGoalID
 * @brief get goal ID.
 * @param move_base_goal   next goal will set in this argument.
 * @return If there are no goals to set next, this function returns false. (But the argument returns 1st goal.)
 */
string MoveBaseGoal::getGoalID()
{/*{{{*/
	if (now_goal_num == -1)
		return "have not set goal yet.";
	else
		return goal_list[now_goal_num].goal_id;
}/*}}}*/


/** @fn MoveBaseGoal::getGoalDistance
 * @brief get distance between now goal and argument frame.
 * @param st StampedTransform.
 * @return distance in meters.
 */
double MoveBaseGoal::getGoalDistance(tf::StampedTransform& st)
{/*{{{*/
	return hypot(
			st.getOrigin().getX() - goal_list[now_goal_num].pose2d.x,
			st.getOrigin().getY() - goal_list[now_goal_num].pose2d.y);
}/*}}}*/


/** @fn MoveBaseGoal::loadCsvGoal
 * @brief load goal list from csv.
 * @param csv_path goal csv data path.
 */
void MoveBaseGoal::loadCsvGoal(string csv_path)
{/*{{{*/
	fstream fs(csv_path);
	if(!fs)
	{
		ROS_INFO("Cannot load %s.", csv_path.c_str());
	}

	int count = 0;
	string str;

	while (getline(fs,str))
	{
		vector<string> record;
		istringstream streambuffer(str);
		string token;
		while (getline(streambuffer, token, ','))
		{
			record.push_back(token);
		}

		if (record.size() != 5)
			ROS_ERROR("Invalid csv format. usage : [GoalID, Frame, GoalX, GoalY, GoalTheta]");
		else
		{
			try {
				Goal goal;
				goal.goal_id = record[0];
				goal.frame_id = record[1];
				goal.pose2d.x = stof(record[2]);
				goal.pose2d.y = stof(record[3]);
				goal.pose2d.theta = stof(record[4]);
				goal_list.push_back(goal);
				// cout<<"x"<<goal.pose2d.x<<endl;
				// cout<<"y"<<goal.pose2d.y<<endl;
				// cout<<"t"<<goal.pose2d.theta<<endl;
			}
			catch (std::invalid_argument e) {
				ROS_ERROR("Cannot convert with stof");
			}
			catch (std::out_of_range e) {
				ROS_ERROR("Cannot convert with stof");
			}
		}

		record.clear();
	}

}/*}}}*/

