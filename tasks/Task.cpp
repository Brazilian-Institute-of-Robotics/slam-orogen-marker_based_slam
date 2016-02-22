/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace marker_based_slam;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine) 
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
    /*Reading from input port*/
    _marker_poses.read(input);

    MarkerPose rbs_input;
    /*Converting to marker_based_slam::MarkerPose*/
    for (int i=0; i < input.size(); ++i )
    {
        base::samples::RigidBodyState rbs;
        std::stringstream ss("");
        
        rbs = input[i];
        std::string str = input[i].targetFrame;
        
        for (int i=9; i<str.size(); ++i)
        {
        	ss << str[i];
        }

        int id = 0;
        ss >> id;

        rbs_input.insert(MarkerPose::value_type(id, rbs));
    } 
    /*Set the input values and calculate the relative pose between markers*/
    map.setPose(rbs_input);
    map.calculatePoses();

    map.printRelativePoses();

    /*Transform std::map<std::pair<int, int>, base::samples::RigidBodyState> to std::vector<marker_based_marker_based_slam::RelativePoses>*/
    std::map<std::pair<int,int>, base::samples::RigidBodyState> all_relative_poses;
    std::vector<RelativePoses> relative_poses_output;
    
    map.getMap(all_relative_poses);
    
    for (RelativePosesIterator it = all_relative_poses.begin(); it != all_relative_poses.end(); ++it)
    {
		RelativePoses temp;

		temp.pair1 = it->first.first;
		temp.pair2 = it->first.second;
		temp.rbs =   it->second;

		relative_poses_output.push_back(temp);
    }
    
    /*Transform marker_based_slam::MarkerPose to std::vector<marker_based_slam::MapPose>*/
    MarkerPose camera_pose;
    std::vector<MapPose> camera_pose_output;

    map.getCameraPose(camera_pose); 
    map.printCameraPose(camera_pose);

    for (MarkerPoseIterator it = camera_pose.begin(); it != camera_pose.end(); ++it)
    {
    	MapPose temp;
    	temp.id = it->first;
    	temp.rbs = it->second;

        camera_pose_output.push_back(temp);

    }
    

    /*Write the relative poses and the camera pose in the output ports*/ 
    _relative_poses.write(relative_poses_output);
    _camera_pose.write(camera_pose_output); 


    input.clear();
    relative_poses_output.clear();
    camera_pose_output.clear();

}

void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
