# nav_ros
## move base
构造函数：
as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);
executeCb()
{
    planThread()
    {
        makePlan(){
            getRobotPose()
            planner_->makePlan()

        }
    }
}