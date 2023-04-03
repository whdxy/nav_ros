# nav_ros
## move base

```c++
//构造函数
as_ = new MoveBaseActionServer(ros::NodeHandle(), "move_base", boost::bind(&MoveBase::executeCb, this, _1), false);
executeCb() // 回调1
{  
    planThread() 
    {
        makePlan(){    
            getRobotPose()
            planner_->makePlan()
        }
    }
    bool done = executeCycle(goal, global_plan);
}

goalCB // 回调2

// 接受目标坐标，然后将其转换成action形式，并发送
```

## teb
trajectory_planner_ros.cpp