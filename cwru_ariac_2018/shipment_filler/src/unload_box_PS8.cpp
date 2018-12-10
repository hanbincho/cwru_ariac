//unload_box.cpp:
// moves a box under camera, then removes sensed parts from box

//use a RobotBehaviorInterface object to communicate with the robot behavior action server
#include <robot_behavior_interface/RobotBehaviorInterface.h>

//we define a message type for a "part" that includes name, pose and associated location code (e.g. bin # or box location)
#include<inventory_msgs/Part.h>

//a "box inspector" object can compare a packing list to a logical camera image to see how we are doing
#include<box_inspector/box_inspector.h>

//conveyor interface communicates with the conveyor action server
#include<conveyor_as/ConveyorInterface.h>


const double COMPETITION_TIMEOUT=500.0; // need to  know what this is for the finals;
// want to ship out partial credit before time runs out!

void model_to_part(osrf_gear::Model model, inventory_msgs::Part &part, unsigned short int location) {
    part.name = model.type;
    part.pose.pose = model.pose;
    part.location = location; //by default
}

int main(int argc, char** argv) {
    // ROS set-ups:
    ros::init(argc, argv, "box_unloader"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    int ans;
    
 
    ROS_INFO("instantiating a RobotBehaviorInterface");
    RobotBehaviorInterface robotBehaviorInterface(&nh); //shipmentFiller owns one as well

    ROS_INFO("instantiating a ConveyorInterface");
    ConveyorInterface conveyorInterface(&nh);
   
    ROS_INFO("instantiating a BoxInspector");
    BoxInspector boxInspector(&nh);

    //instantiate an object of appropriate data type for our move-part commands
    inventory_msgs::Part current_part;

    geometry_msgs::PoseStamped box_pose_wrt_world;  //camera sees box, coordinates are converted to world coords
    
    bool status;    
    int nparts;

    //for box inspector, need to define multiple vectors for args, 
    //box inspector will identify parts and convert their coords to world frame
    //in the present example, desired_models_wrt_world is left empty, so ALL observed parts will be considered "orphaned"
        vector<osrf_gear::Model> desired_models_wrt_world;
        vector<osrf_gear::Model> satisfied_models_wrt_world;
        vector<osrf_gear::Model> misplaced_models_actual_coords_wrt_world;
        vector<osrf_gear::Model> misplaced_models_desired_coords_wrt_world;
        vector<osrf_gear::Model> missing_models_wrt_world;
        vector<osrf_gear::Model> orphan_models_wrt_world;
        vector<int> part_indices_missing;
        vector<int> part_indices_misplaced;
        vector<int> part_indices_precisely_placed;

    
    //use conveyor action  server for multi-tasking
    ROS_INFO("getting a box into position: ");
    int nprint = 0;
    conveyorInterface.move_new_box_to_Q1();  //member function of conveyor interface to move a box to inspection station 1
    while (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_SEEN_AT_Q1) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        nprint++;
        if (nprint % 10 == 0) {
            ROS_INFO("waiting for conveyor to advance a box to Q1...");
        }
    }

    //update box pose,  if possible              
    if (boxInspector.get_box_pose_wrt_world(box_pose_wrt_world)) {
        ROS_INFO_STREAM("box seen at: " << box_pose_wrt_world << endl);
    }
    else {
        ROS_WARN("no box seen.  something is wrong! I quit!!");
        exit(1);
    }
    
    // if survive to here, then box is at Q1 inspection station; 
    
    //inspect the box and classify all observed parts
    boxInspector.update_inspection(desired_models_wrt_world,
        satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
        misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
        orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
        part_indices_precisely_placed);
    ROS_INFO("orphaned parts in box: ");
    nparts = orphan_models_wrt_world.size();
    ROS_INFO("num parts seen in box = %d",nparts);
    for (int i=0;i<nparts;i++) {
       ROS_INFO_STREAM("orphaned  parts: "<<orphan_models_wrt_world[i]<<endl);
    }
    
    
    // observe box_camera_1 to get the three parts within the shipping box
    // need to compare coordinates from first camera to the obtained coordinates from quality inspector

    if (boxInspector.get_bad_part_Q1(current_part)) {
        ROS_INFO("found bad part: ");
        ROS_INFO_STREAM(current_part<<endl);
        
        cout<<"enter 1 to attempt to remove bad part: "; //poor-man's breakpoint
        cin>>ans;        

    //use the robot action server to acquire and dispose of the specified part in the box:
    }    

    osrf_gear::Model part_inspect;
    inventory_msgs::Part bad_part;
    bool flag_1 = false;
    bool flag_2 = false;
    for (int j = 0; j < nparts; j++) {
	part_inspect = orphan_models_wrt_world[j]; // go through all the parts in the box to see if coordinates match

	// check x-coordinates
	if (abs(part_inspect.pose.position.x - current_part.pose.pose.position.x) < 0.03) {
	    flag_1 = true;
	}

	// check y-coordinates
	if (abs(part_inspect.pose.position.y - current_part.pose.pose.position.y) < 0.03) {
	    flag_2 = true;
	}
	if (flag_1 && flag_2) {
	    // at this point, we have found the bad part that needs to be removed
	    model_to_part(part_inspect, bad_part, inventory_msgs::Part::QUALITY_SENSOR_1); // convert the bad part to the Part type
	    status = robotBehaviorInterface.pick_part_from_box(bad_part); // remove the identified bad part from the box
	    model_to_part(part_inspect, current_part, inventory_msgs::Part::QUALITY_SENSOR_1); // update the current part details with those of the identified model
	    // reset flags
	    flag_1 = false;
	    flag_2 = false;
	    break;
	}
    }

    // check to see if the name of the current_part has been updated from "model"
    //ROS_INFO("Here's info about the current_part");
    //ROS_INFO_STREAM(current_part<<endl);

    //use the robot action server to acquire and dispose of the specified part in the box:- ORIGINAL METHOD
    //status = robotBehaviorInterface.pick_part_from_box(current_part);

    //after removing the bad part, re-inspect the box:
    boxInspector.update_inspection(desired_models_wrt_world,
        satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
        misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
        orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
        part_indices_precisely_placed);
    ROS_INFO("orphaned parts in box: ");
    nparts = orphan_models_wrt_world.size();
    ROS_INFO("num parts seen in box = %d",nparts);
    for (int i=0;i<nparts;i++) {
       ROS_INFO_STREAM("orphaned  parts: "<<orphan_models_wrt_world[i]<<endl);
    }
    // check to see the updated contents (if bad part was correctly removed)
    ROS_INFO("Above is the updated contents in the box after bad part removal");
    
    // move robot to grasp and discard each remaining part in the box
    inventory_msgs::Part part_to_be_removed;
    for (int j = 0; j < nparts; j++) {
	model_to_part(orphan_models_wrt_world[j], part_to_be_removed, inventory_msgs::Part::QUALITY_SENSOR_1);
	status = robotBehaviorInterface.pick_part_from_box(part_to_be_removed);
    }

    //after all parts, re-inspect the box:
    boxInspector.update_inspection(desired_models_wrt_world,
        satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
        misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
        orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
        part_indices_precisely_placed);
    ROS_INFO("orphaned parts in box: ");
    nparts = orphan_models_wrt_world.size();
    ROS_INFO("num parts seen in box = %d",nparts);
    for (int i=0;i<nparts;i++) {
       ROS_INFO_STREAM("orphaned  parts: "<<orphan_models_wrt_world[i]<<endl);
    }
    ROS_INFO("Above is the updated contents in the box after complete removal");

    //box inspector sees "model", defined in osrf_gear; convert this to our datatype "Part"
    //void model_to_part(osrf_gear::Model model, inventory_msgs::Part &part, unsigned short int location) 
    //model_to_part(orphan_models_wrt_world[0], current_part, inventory_msgs::Part::QUALITY_SENSOR_1);

            return 0;
    //here's an oddity: this node runs to completion.  But sometimes, Linux complains bitterly about
    // *** Error in `/home/wyatt/ros_ws/devel/lib/shipment_filler/unload_box': corrupted size vs. prev_size: 0x000000000227c7c0 ***
    // don't know why.  But does not seem to matter.  If anyone figures this  out, please let me know.
}
