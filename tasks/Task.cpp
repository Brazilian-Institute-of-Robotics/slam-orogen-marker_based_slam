/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <marker_based_slam/MapParser.hpp>

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

    //initialize bool variable to avoid to compute the same map multiple
    //times
    map_is_build = false;

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
    std::vector<base::samples::RigidBodyState> input;
    _marker_poses.read(input);
    MarkerPose rbs_input = rbsToMarkerPose(input);

    /*Set the input values and calculate the relative pose between markers*/
    mbslam.setPose(rbs_input);

    if (_known_map.get() & !map_is_build)
    {
        mbslam.getMapFromFile(_map_address.get(), PoseTransformationToRbs(_world2marker.get()));
        map_is_build = true;
    }
    else if (!_known_map.get())
    {
        mbslam.calculatePoses();
    }

    mbslam.printRelativePoses();

    /*Convert RelativePoses to std::vector<RelativeMarkerPoses>*/
    RelativePoses all_relative_poses = mbslam.getMap();
    std::vector<RelativeMarkerPoses> relative_poses_output;
    relative_poses_output = RelativePosesToRelativeMarkerPoses(all_relative_poses);
    
    /*Get the Camera(s) Pose(s)
    /*Transform MarkerPose to std::vector<MapPose>*/
    base::samples::RigidBodyState camera2robot = PoseTransformationToRbs(_camera2robot.get());
    MarkerPose camera_pose = mbslam.getCameraPose(camera2robot); 
    std::vector<MapPose> camera_pose_output = MarkerPoseToMapPose(camera_pose);
    mbslam.printCameraPose(camera_pose);

    /*Write the relative poses and the camera pose in the output ports*/ 
    _relative_poses.write(relative_poses_output);
    _camera_pose.write(camera_pose_output); 

    input.clear();
    if (_known_map.get()) relative_poses_output.clear();
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

MarkerPose Task::rbsToMarkerPose(std::vector<base::samples::RigidBodyState> input)
{
    MarkerPose rbs_input;

    /*Converting to marker_based_slam::MarkerPose*/
    for (int i=0; i < input.size(); ++i )
    {
        base::samples::RigidBodyState rbs;
        std::stringstream ss("");
        
        rbs = input[i];
        std::string str = input[i].targetFrame;
        
        for (int i=0; i<str.size(); ++i)
        {
            if(std::isdigit(str[i]))
            {
        	    ss << str[i];
            }
        }

        int id = 0;
        ss >> id;
        rbs_input.insert(MarkerPose::value_type(id, rbs));
    } 

    return rbs_input;
}

base::samples::RigidBodyState Task::PoseTransformationToRbs(PoseTransformation trans)
{
    base::samples::RigidBodyState rbs;

    rbs.position[0] = trans.x;
    rbs.position[1] = trans.y;
    rbs.position[2] = trans.z;

    rbs.orientation = MapParser::getQuaternionFromAxisAngle(trans.rot_x*M_PI/180, 
        trans.rot_y*M_PI/180, 
        trans.rot_z*M_PI/180);

    return rbs;


}
std::vector<RelativeMarkerPoses> Task::RelativePosesToRelativeMarkerPoses(RelativePoses all_relative_poses)
{
    /*Transform RelativePoses to std::vector<RelativeMarkerPoses>*/
    std::vector<RelativeMarkerPoses> relative_poses_output;

    for (RelativePosesIterator it = all_relative_poses.begin(); it != all_relative_poses.end(); ++it)
    {
        RelativeMarkerPoses temp;

        temp.pair1 = it->first.first;
        temp.pair2 = it->first.second;
        temp.rbs =   it->second;

        relative_poses_output.push_back(temp);
    }

    return relative_poses_output;
}

std::vector<MapPose> Task::MarkerPoseToMapPose(MarkerPose camera_pose)
{
    /*Transform marker_based_slam::MarkerPose to std::vector<marker_based_slam::MapPose>*/
    std::vector<MapPose> camera_pose_output;
    for (MarkerPoseIterator it = camera_pose.begin(); it != camera_pose.end(); ++it)
    {
    	MapPose temp;
    	temp.id = it->first;
    	temp.rbs = it->second;

        camera_pose_output.push_back(temp);

    }

    return camera_pose_output;

}

