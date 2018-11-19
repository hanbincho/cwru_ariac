//get images from topic "simple_camera/image_raw"; remap, as desired;
//search for red pixels;
// convert (sufficiently) red pixels to white, all other pixels black
// compute centroid of red pixels and display as a blue square
// publish result of processed image on topic "/image_converter/output_video"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <xform_utils/xform_utils.h>
#include <Eigen/Dense>

static const std::string OPENCV_WINDOW = "Open-CV display window";
using namespace std;
using Eigen::MatrixXd;

int g_redratio; //threshold to decide if a pixel qualifies as dominantly "red"

const double BLOCK_HEIGHT=0.035; // hard-coded top surface of block relative to world frame

class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher block_pose_publisher_; // = n.advertise<std_msgs::Float64>("topic1", 1);
    geometry_msgs::PoseStamped block_pose_;
    XformUtils xformUtils;

public:

    ImageConverter(ros::NodeHandle &nodehandle)
    : it_(nh_) {
        // Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe("simple_camera/image_raw", 1,
                &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
        block_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("block_pose", 1, true); 
        block_pose_.header.frame_id = "world"; //specify the  block pose in world coords
        block_pose_.pose.position.z = BLOCK_HEIGHT;
        block_pose_.pose.position.x = 0.5; //not true, but legal
        block_pose_.pose.position.y = 0.0; //not true, but legal
        
        // need camera info to fill in x,y,and orientation x,y,z,w
        //geometry_msgs::Quaternion quat_est
        //quat_est = xformUtils.convertPlanarPsi2Quaternion(yaw_est);
        block_pose_.pose.orientation = xformUtils.convertPlanarPsi2Quaternion(0); //not true, but legal
        
        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter() {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    //image comes in as a ROS message, but gets converted to an OpenCV type
    void imageCb(const sensor_msgs::ImageConstPtr& msg); 
    
}; //end of class definition

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImagePtr cv_ptr; //OpenCV data type
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        // look for red pixels; turn all other pixels black, and turn red pixels white
        int npix = 0; //count the red pixels
        int isum = 0; //accumulate the column values of red pixels
        int jsum = 0; //accumulate the row values of red pixels
        int redval, blueval, greenval, testval;
	double scaling_factor = 0.002953175742;	// calculated by hand from Gazebo and openCV

        cv::Vec3b rgbpix; // OpenCV representation of an RGB pixel
	MatrixXd Tm(4, 4); // 4x4 transformation matrix
	MatrixXd center(2, 1); // 2x1 centroid matrix but in distances
	MatrixXd x_pr(2, 1); // x-axis rotational matrix
	MatrixXd y_pr(2, 1); // y-axis rotational matrix
	MatrixXd block_camera(4, 1); // block position coordinates relative to the camera
	MatrixXd block_robot(4, 1); // block position coordinates relative to the robot base

        //comb through all pixels (j,i)= (row,col)
        for (int i = 0; i < cv_ptr->image.cols; i++) {
            for (int j = 0; j < cv_ptr->image.rows; j++) {
                rgbpix = cv_ptr->image.at<cv::Vec3b>(j, i); //extract an RGB pixel
                //examine intensity of R, G and B components (0 to 255)
                redval = rgbpix[2] + 1; //add 1, to avoid divide by zero
                blueval = rgbpix[0] + 1;
                greenval = rgbpix[1] + 1;
                //look for red values that are large compared to blue+green
                testval = redval / (blueval + greenval);
                //if red (enough), paint this white:
                if (testval > g_redratio) {
                    cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 255;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[1] = 255;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[2] = 255;
                    npix++; //note that found another red pixel
                    isum += i; //accumulate row and col index vals
                    jsum += j;
                } else { //else paint it black
                    cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 0;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[1] = 0;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[2] = 0;
                }
            }
        }
	
	// Define center projection matrix- may want to redefine by finding the numbers myself
	center(0, 0) = 0.545;
	center(1, 0) = 0.322;

	// Define x-axis rotational matrix- calculated by hand from looking at Gazebo and openCV
	x_pr(0, 0) = 0.97708897; 
	x_pr(1, 0) = -0.212831261;

	// Define y-axis rotational matrix- based off the matrix: [cos thea, -sin theta; sin theta, cos theta] where the first column is the x-axis rotational values and the second is the y-axis rotational values
	y_pr(0, 0) = -x_pr(1, 0);
	y_pr(1, 0) = x_pr(0, 0);
	
	// Define transformation matrix based on Gazebo and openCV values
	// x-axis rotational matrix
	Tm(0, 0) = x_pr(0, 0);
	Tm(1, 0) = x_pr(1, 0);
	Tm(2, 0) = 0;
	Tm(3, 0) = 0;
	// y-axis rotational matrix
	Tm(0, 1) = y_pr(0, 0);
	Tm(1, 1) = y_pr(1, 0);
	Tm(2, 1) = 0;
	Tm(3, 1) = 0;
	// z-axis rotational matrix
	Tm(0, 2) = 0;
	Tm(1, 2) = 0;
	Tm(2, 2) = 1;
	Tm(3, 2) = 0;
	// translational matrix
	Tm(0, 3) = center(0, 0);
	Tm(1, 3) = center(1, 0);
	Tm(2, 3) = 0.5*BLOCK_HEIGHT;
	Tm(3, 3) = 1;
	
        //cout << "npix: " << npix << endl;
        //paint in a blue square at the centroid:
        int half_box = 5; // choose size of box to paint
        int i_centroid, j_centroid;
        double x_centroid, y_centroid;
        if (npix > 0) {
            i_centroid = isum / npix; // average value of u component of red pixels
            j_centroid = jsum / npix; // avg v component
            x_centroid = ((double) isum)/((double) npix); //floating-pt version
            y_centroid = ((double) jsum)/((double) npix);
            ROS_INFO("u_avg: %f; v_avg: %f",x_centroid,y_centroid);
            //cout << "i_avg: " << i_centroid << endl; //i,j centroid of red pixels
            //cout << "j_avg: " << j_centroid << endl;
            for (int i_box = i_centroid - half_box; i_box <= i_centroid + half_box; i_box++) {
                for (int j_box = j_centroid - half_box; j_box <= j_centroid + half_box; j_box++) {
                    //make sure indices fit within the image 
                    if ((i_box >= 0)&&(j_box >= 0)&&(i_box < cv_ptr->image.cols)&&(j_box < cv_ptr->image.rows)) {
                        cv_ptr->image.at<cv::Vec3b>(j_box, i_box)[0] = 255; //(255,0,0) is pure blue
                        cv_ptr->image.at<cv::Vec3b>(j_box, i_box)[1] = 0;
                        cv_ptr->image.at<cv::Vec3b>(j_box, i_box)[2] = 0;
                    }
                }
            }

        }

	// 4x1 block centroid coordinates to be multiplied to transformation matrix; computation: (i-ic)*s
	block_camera(0, 0) = (x_centroid-320)*scaling_factor;
	block_camera(1, 0) = (y_centroid-240)*scaling_factor;
	block_camera(2, 0) = 0.5*BLOCK_HEIGHT;
	block_camera(3, 0) = 1;

	// Get block coordinates relative to the robot base
	block_robot = Tm*block_camera;
	
        // Update GUI Window; this will display processed images on the open-cv viewer.
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3); //need waitKey call to update OpenCV image window

        // Also, publish the processed image as a ROS message on a ROS topic
        // can view this stream in ROS with: 
        //rosrun image_view image_view image:=/image_converter/output_video
        image_pub_.publish(cv_ptr->toImageMsg());
        
	// Set calculated distance values as the block position
        block_pose_.pose.position.x = block_robot(0, 0); 
        block_pose_.pose.position.y = block_robot(1, 0); 
        double theta=atan(x_pr(1, 0)/x_pr(0, 0)); 
	//ROS_INFO("pose_x: %f; pose_y: %f", block_pose_.pose.position.x, block_pose_.pose.position.y); // check to see if values are being published to block_pose node
        
        // need camera info to fill in x,y,and orientation x,y,z,w
        //geometry_msgs::Quaternion quat_est
        //quat_est = xformUtils.convertPlanarPsi2Quaternion(yaw_est);
        block_pose_.pose.orientation = xformUtils.convertPlanarPsi2Quaternion(theta); //not true, but legal
        block_pose_publisher_.publish(block_pose_);

	// Find the yaw, pitch, and roll of the block
	double yaw, pitch, roll;
	double a = block_pose_.pose.orientation.x;
	double b = block_pose_.pose.orientation.y;
	double c = block_pose_.pose.orientation.z;
	double d = block_pose_.pose.orientation.w;
	yaw = atan(2*(a*b +c*d)/(a^2 - b^2 - c^2 + d^2));
	pitch = -asin(2*(b*d - a*c));
	roll = atan((2*(a*d + b*c)/(a^2 + b^2 + c^2 - d^2));
	ROS_INFO("yaw: %f, pitch: %f, roll: %f", yaw, pitch, roll);
    }

int main(int argc, char** argv) {
    ros::init(argc, argv, "red_pixel_finder");
    ros::NodeHandle n; //        
    ImageConverter ic(n); // instantiate object of class ImageConverter
    //cout << "enter red ratio threshold: (e.g. 10) ";
    //cin >> g_redratio;
    g_redratio= 10; //choose a threshold to define what is "red" enough
    ros::Duration timer(0.1);
    double x, y, z;
    while (ros::ok()) {
        ros::spinOnce();
        timer.sleep();
    }
    return 0;
}
