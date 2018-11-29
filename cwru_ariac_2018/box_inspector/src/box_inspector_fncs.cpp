


//given an image, compute all models w/rt box and return in a "Shipment" object
bool BoxInspector::model_poses_wrt_box(osrf_gear::Shipment &shipment_status) {
    ROS_INFO("model_poses_wrt_box()");
    geometry_msgs::Pose cam_pose, box_pose_wrt_cam, model_pose_wrt_cam, part_pose_wrt_box;
    geometry_msgs::PoseStamped box_pose_wrt_world, part_pose_wrt_world;
    Eigen::Affine3d affine_cam_wrt_world, affine_part_wrt_cam, affine_part_wrt_box,
            affine_box_pose_wrt_world, affine_part_wrt_world;
    shipment_status.products.clear(); //maybe not?
    get_new_snapshot_from_box_cam();
    bool found_box = false;
    int i_box = 0;
    int num_models;

    num_models = box_inspector_image_.models.size();
    //parse the  image and compute model poses w/rt box; ignore  the box itself
    if (num_models == 0) {
        ROS_WARN("model_poses_wrt_box(): image has zero models");
        return false;
    }
    ROS_INFO("box cam sees %d models", num_models);

    //look for the box:
    string box_name("shipping_box"); //does a model match this name?
    osrf_gear::Model model;
    cam_pose = box_inspector_image_.pose;

    for (int imodel = 0; imodel < num_models; imodel++) {
        model = box_inspector_image_.models[imodel];
        string model_name(model.type);
        if (model_name == box_name) {
            box_pose_wrt_cam = model.pose;
            ROS_INFO_STREAM("box pose w/rt camera: " << box_pose_wrt_cam << endl);
            found_box = true;
            i_box = imodel; //remember where box is in the list of models
            box_pose_wrt_world = compute_stPose(cam_pose, box_pose_wrt_cam);
            affine_box_pose_wrt_world = xformUtils_.transformPoseToEigenAffine3d(box_pose_wrt_world);
            break;
        }
    }
    if (!found_box) {
        ROS_WARN("model_poses_wrt_box(): did not find box in view");
        return false;
    }

    osrf_gear::Product product;
    
    //if here, image contains box, and box pose w/rt world is in affine_box_pose_wrt_world
    for (int imodel = 0; imodel < num_models; imodel++) {
        if (imodel != i_box) { //if here, have a model NOT the box
            model = box_inspector_image_.models[imodel];
            string model_name(model.type);
            ROS_INFO_STREAM("model: " << model << endl);
            model_pose_wrt_cam = model.pose;
            part_pose_wrt_world = compute_stPose(cam_pose, model_pose_wrt_cam);
            ROS_INFO_STREAM("part pose wrt world: " << part_pose_wrt_world << endl);
            
            //MISSING LINES HERE...FIXED IT
            Eigen::Affine3d model_affine_wrt_cam, box_affine_wrt_cam, model_affine_wrt_box;
            model_affine_wrt_cam=xformUtils_.transformPoseToEigenAffine3d(model_pose_wrt_cam);
            box_affine_wrt_cam=xformUtils_.transformPoseToEigenAffine3d(box_pose_wrt_cam);
            model_affine_wrt_box = box_affine_wrt_cam.inverse()*model_affine_wrt_cam;
            part_pose_wrt_box=xformUtils_.transformEigenAffine3dToPose(model_affine_wrt_box);
            //put this into "shipment"  object:
            //string shipment_type
            //box_inspector/Product[] products
            //  string type
            //  geometry_msgs/Pose pose    
            product.type = model.type;
            product.pose = part_pose_wrt_box;
            shipment_status.products.push_back(product);
        }
    }
    ROS_INFO_STREAM("resulting part poses w/rt box: " << shipment_status << endl);
    return true;
}



//given a shipment description that specifies desired parts and  poses with respect to box,
//convert this to poses of parts w/rt world;
//robot needs to know current and desired part poses  w/rt world

void BoxInspector::compute_shipment_poses_wrt_world(osrf_gear::Shipment shipment_wrt_box,
        geometry_msgs::PoseStamped box_pose_wrt_world,
        vector<osrf_gear::Model> &desired_models_wrt_world) {
	//desired_models_wrt_world.clear(); // why not?
	//Fixed, recheck logic later
	osrf_gear::Product product_wrt_box;
    desired_models_wrt_world.clear();
	for(int i=0;i<shipment_wrt_box.products.size();i++) {
	product_wrt_box.type = shipment_wrt_box.products[i].type;
	product_wrt_box.pose = shipment_wrt_box.products[i].pose;
	Eigen::Affine3d product_affine_wrt_box = xformUtils_.transformPoseToEigenAffine3d(product_wrt_box.pose);
	Eigen::Affine3d box_affine_wrt_world = xformUtils_.transformPoseToEigenAffine3d(box_pose_wrt_world.pose);
	Eigen::Affine3d product_affine_wrt_world = box_affine_wrt_world*product_affine_wrt_box;	
	geometry_msgs::Pose product_pose_wrt_world = xformUtils_.transformEigenAffine3dToPose(product_affine_wrt_world);
	osrf_gear::Model model;
	model.type=product_wrt_box.type;
	model.pose=product_pose_wrt_world;
	desired_models_wrt_world.push_back(model);
        }
    
    //compute and fill in terms in desired_models_wrt_world
}


