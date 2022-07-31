#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>


#define METRE_TO_PIXEL_SCALE 50
#define FORWARD_SWIM_SPEED_SCALING 0.2
#define POSITION_GRAPHIC_RADIUS 20.0
#define HEADING_GRAPHIC_LENGTH 50.0
#define DEPTH 2
#define IMG_SIZE 400
#define Particle_select 3

// Class Localizer is a sample stub that you can build upon for your implementation
// (advised but optional: starting from scratch is also fine)
//
//Defining a structure for particles
struct particle {
 double particle_x = 0;
 double particle_y = 0;
 double particle_yaw = 0;
 double particle_weight = 0;
 int particle_ID = 0;
 
 bool operator<(const particle& a) const
    {
        return particle_weight < a.particle_weight;
    }
};

class Localizer {

public:
  ros::NodeHandle nh;
  image_transport::Publisher pub;
  image_transport::Subscriber gt_img_sub;
  image_transport::Subscriber robot_img_sub;
  ros::Publisher estimate_pub;

  ros::Subscriber motion_command_sub;

  geometry_msgs::PoseStamped estimated_location;

  cv::Mat map_image;
  cv::Mat localization_result_image;

  double posx = estimated_location.pose.position.x;
  double posy = estimated_location.pose.position.y;
  double yaw = 0 ;

  // Number of particles to draw
	int num_particles = 10; 
	// Flag, if filter is initialized
	bool is_initialized = false;

	int count = 0;

	particle final_particle;
	
	// Vector of weights of all particles
	std::vector<double> weights; //array of weights
	

	// Set of current particles
	std::vector<particle> particles; // array of particles

	const cv::Mat k = (cv::Mat_<double>(3,3) << 238.3515418007097/DEPTH, 0.0, 200.5,
												 0.0, 238.3515418007097/DEPTH, 200.5,
												  0.0, 0.0, 1.0);
	const cv::Mat T_cam = (cv::Mat_<double>(3,3) << 1, 0, -0.07,
													0, 1, -0.384,
													0, 0, 1);

  Localizer( int argc, char** argv ){

	image_transport::ImageTransport it(nh);
	pub = it.advertise("/assign1/localization_debug_image", 1);
	estimate_pub = nh.advertise<geometry_msgs::PoseStamped>( "/assign1/localization_estimate",1);
	std::string ag_path = ros::package::getPath("aqua_gazebo");
	map_image = cv::imread((ag_path+"/materials/fishermans_small.png").c_str(), cv::IMREAD_COLOR);

	estimated_location.pose.position.x = 0;
	estimated_location.pose.position.y = 0;

	particles = init(num_particles);
	localization_result_image = map_image.clone();

	robot_img_sub = it.subscribe("/aqua/back_down/image_raw", 1, &Localizer::robotImageCallback, this);
	motion_command_sub = nh.subscribe<geometry_msgs::PoseStamped>("/aqua/target_pose", 1, &Localizer::motionCommandCallback, this);

	ROS_INFO( "localizer node constructed and subscribed." );
  }


  std::vector<particle> init(int N_particles)
  {
  	if(is_initialized == false){
  		weights.resize(N_particles);//array of weights
	    particles.resize(N_particles);// array of particles

	    std::default_random_engine gen; //http://www.cplusplus.com/reference/random/default_random_engine/

	    std::normal_distribution<double> dist_x(posx, 0.3); // mean is centered around the new measurement
	    std::normal_distribution<double> dist_y(posy,0.3);
	    std::normal_distribution<double> dist_yaw(yaw, 0.1);
		

	    for(int i= 0; i< N_particles; i++)
		{	
			particle P;
			P.particle_x = dist_x(gen);;
			P.particle_y = dist_y(gen);;
			P.particle_yaw = dist_yaw(gen);
			P.particle_weight = 0;
			P.particle_ID= i;
			particles[i]= P;
			weights[i] = P.particle_weight;
			is_initialized = true;
		}
  	}
  	return particles;
  }

  std::vector<particle> resample(std::vector<particle> Par)
  	{
  		std::default_random_engine gen; //http://www.cplusplus.com/reference/random/default_random_engine/

	    std::normal_distribution<double> dist_x(final_particle.particle_x, 1); // mean is centered around the new measurement
	    std::normal_distribution<double> dist_y(final_particle.particle_y,1);
	    std::normal_distribution<double> dist_yaw(final_particle.particle_yaw, 0.1);

  		for(int i = Particle_select; i < num_particles; i++)
		{
			particle P;
			P.particle_x = dist_x(gen);;
			P.particle_y = dist_y(gen);;
			P.particle_yaw = dist_yaw(gen);
			P.particle_weight = 0;
			Par[i]= P;
		}
		return Par;

  	}

  particle final_select( std::vector<particle> a){

  	for (int i = 0; i < Particle_select; i++){

			final_particle.particle_x = (a[i].particle_x + FORWARD_SWIM_SPEED_SCALING * estimated_location.pose.position.x)/Particle_select;
			final_particle.particle_y = (a[i].particle_y + FORWARD_SWIM_SPEED_SCALING * estimated_location.pose.position.y )/Particle_select;
			final_particle.particle_yaw = (a[i].particle_yaw )/Particle_select;
		}
		return final_particle;
  }

  cv::Mat IMG_transform(int x, int y, double theta, cv::Mat mapImage)
  {
  	cv::Mat T_robot = (cv::Mat_<double>(3,3) <<  1, 0, -8.0-x,
  												 0, 1, -25.2-y,
  												  0, 0, 1);
	cv::Mat R = (cv::Mat_<double>(3,3) << cos(CV_PI + theta), sin(CV_PI + theta), 0,
										 -sin(CV_PI + theta), cos(CV_PI + theta), 0,
										  0, 0, 1);
	cv::Mat T = k * T_cam * R * T_robot;
	cv::Mat T_inv = METRE_TO_PIXEL_SCALE * T.inv();
	cv::Mat tranIMG;
	tranIMG.create(cv::Size2d(IMG_SIZE,IMG_SIZE),mapImage.type());
	for(int i = 0; i < IMG_SIZE; i++ )
    {
        for(int j = 0; j < IMG_SIZE; j++ )
        {
            cv::Mat realCoord = (cv::Mat_<double>(3, 1)<< i , j ,1);
            cv::Mat ImgCoord = T_inv * realCoord;

            int x = mapImage.size().height - ImgCoord.at<double>(1,0);
            int y = ImgCoord.at<double>(0,0);

            tranIMG.at<cv::Vec3b>(i,j) = mapImage.at<cv::Vec3b>(x,y);


        }

    }
	return tranIMG;
  }

	cv::Mat IMG_transform_patch(int x, int y, double theta, cv::Mat mapImage, int patch_size)
  {
  	cv::Mat T_robot = (cv::Mat_<double>(3,3) <<  1, 0, -8.0-x,
  												 0, 1, -25.2-y,
  												  0, 0, 1);
	cv::Mat R = (cv::Mat_<double>(3,3) << cos(CV_PI + theta), sin(CV_PI + theta), 0,
										 -sin(CV_PI + theta), cos(CV_PI + theta), 0,
										  0, 0, 1);
	cv::Mat T = k * T_cam * R * T_robot;
	cv::Mat T_inv = METRE_TO_PIXEL_SCALE * T.inv();
	cv::Mat tranIMG_patch;
	tranIMG_patch.create(cv::Size2d(patch_size,patch_size),mapImage.type());
	for(int i = IMG_SIZE/2 - patch_size/2; i < IMG_SIZE/2 + patch_size/2; i++ )
    {
        for(int j = IMG_SIZE/2 - patch_size/2; j < IMG_SIZE/2 + patch_size/2; j++ )
        {
            cv::Mat realCoord = (cv::Mat_<double>(3, 1)<< i , j ,1);
            cv::Mat ImgCoord = T_inv * realCoord;

            int x = mapImage.size().height - ImgCoord.at<double>(1,0);
            int y = ImgCoord.at<double>(0,0);

            tranIMG_patch.at<cv::Vec3b>(i - IMG_SIZE/2 + patch_size/2,j- IMG_SIZE/2 + patch_size/2) = mapImage.at<cv::Vec3b>(x,y);


        }

    }


    return tranIMG_patch;
  }
	
  double IMG_compare(cv::Mat Robot_IMG, cv::Mat Part_IMG)
  	{
  		int center = Robot_IMG.size().height/2;
  		int span = 1;
  		double diff = 0;
  		double total_diff = 0;
  		std::vector <double> differences;

  			for(int i = center - span; i <= center + span; i+=span)
  			{
  				for (int j = center - span; j <= center + span; j+=span)
  				{
  					cv::Vec3b predicted_color = Robot_IMG.at<cv::Vec3b>(i,j);
					cv::Vec3b predicted_color2 = Part_IMG.at<cv::Vec3b>(i,j);
					diff = cv::norm(predicted_color, predicted_color2);
					total_diff += diff;
				}
  			}
  		return total_diff;
  	}

	void robotImageCallback( const sensor_msgs::ImageConstPtr& robot_img ){


		int patch_size = 100;

		
  		// cv::imwrite("/home/nikhil/a1_overlay_ws/src/comp765_assign1/src/map_image.png", map_image);
  		// cv::Mat robot_image = cv_bridge::toCvCopy(robot_img, sensor_msgs::image_encodings::BGR8)->image;
  		// cv::imwrite("/home/nikhil/a1_overlay_ws/src/comp765_assign1/src/robot_image.png", robot_image);

  		// cv::Rect myROI(robot_image.size().height/2 - patch_size/2 , robot_image.size().width/2 - patch_size/2, patch_size, patch_size);
  		// cv::Mat Robot_patch = robot_image(myROI);
		// cv::imwrite("/home/nikhil/a1_overlay_ws/src/comp765_assign1/src/robot_patch.png", Robot_patch);  		
		
		// cv::Mat Particle_image = IMG_transform(posx,posy,yaw,map_image);
		// cv::imwrite("/home/nikhil/a1_overlay_ws/src/comp765_assign1/src/Particle_image.png", Particle_image);

		// cv::Mat Particle_patch = IMG_transform_patch(posx,posy,yaw,map_image,patch_size);
		// cv::imwrite("/home/nikhil/a1_overlay_ws/src/comp765_assign1/src/Particle_patch.png", Particle_patch);

		double tempdiff = IMG_compare(Robot_patch,Particle_patch);
							
		for(int i = 0; i < num_particles; i++)
		{
			cv::Mat P_patch;
			particle P = particles[i];
			P_patch = IMG_transform_patch(P.particle_x, P.particle_y, P.particle_yaw, map_image, patch_size);
			P.particle_weight = IMG_compare(Robot_patch,Particle_patch);
			weights[i] = P.particle_weight;
			particles[i] = P;
		}
		

		/*std::sort(particles.begin(), particles.end()); // sorting particles vector. 

		final_particle = final_select(particles);
		particles = resample (particles);
*/

		//std::cout << "reached the end of image callback";
		
	}


	// TODO: You must fill in the code here to implement an observation model for your localizer
  

  // Function motionCommandCallback is a example of how to work with Aqua's motion commands (your view on the odometry).
  // The initial version simply integrates the commands over time to make a very rough location estimate.
  // TODO: You must improve it to work with the localizer you implement.
  //
  // Note the somewhat unique meaning of fields in motion_command
  //    motion_command
  //      pose
  //        position
  //          x - requested forward swim effort in a unitless number ranging from 0.0 to 1.0. You must translate this into
  //              a velocity estimate in some way. Currently a simple constant is used.
  //          y - requested up/down swim effort. Not used in this assignment
  //          z - unused
  //        orientation - A quaternion that represents the desired body orientation w.r.t. global frame. Note that
  //                      Aqua's controller will not achieve this orientation perfectly, so you must consider that
  //                      this is only a noisy estimate of the robot's orientation (correct for it with your filter!)
  //
  // Note that we use negative angles because the geometry of the map image is formed with its Z axis pointing downwards
  // (the camera is looking at the ocean floor)
  //
  void motionCommandCallback(const geometry_msgs::PoseStamped::ConstPtr& motion_command ){

	geometry_msgs::PoseStamped command = *motion_command;
	double target_roll, target_pitch, target_yaw;
	tf::Quaternion target_orientation;
	tf::quaternionMsgToTF(command.pose.orientation, target_orientation);
	tf::Matrix3x3(target_orientation).getEulerYPR( target_yaw, target_pitch, target_roll );
	
	std::sort(particles.begin(), particles.end()); // sorting particles vector. 

	final_particle = final_select(particles);
	particles = resample (particles);

	//std:: cout<<" entered motion model  "<< particles.size();

	// The following three lines implement the basic motion model example
	estimated_location.pose.position.x = estimated_location.pose.position.x + FORWARD_SWIM_SPEED_SCALING * command.pose.position.x * cos( -target_yaw );
	estimated_location.pose.position.y = estimated_location.pose.position.y + FORWARD_SWIM_SPEED_SCALING * command.pose.position.x * sin( -target_yaw );
	estimated_location.pose.orientation = command.pose.orientation;
	yaw = target_yaw;
	/*estimated_location.pose.position.x = FORWARD_SWIM_SPEED_SCALING * final_particle.particle_x * cos( -target_yaw );
	estimated_location.pose.position.y = FORWARD_SWIM_SPEED_SCALING * final_particle.particle_y * sin( -target_yaw );
	estimated_location.pose.orientation = command.pose.orientation;
*/
	// The remainder of this function is sample drawing code to plot your answer on the map image.

	// This line resets the image to the original map so you can start drawing fresh each time.
	// Comment the one following line to plot your whole trajectory
	localization_result_image = map_image.clone();

	int estimated_robo_image_x = localization_result_image.size().width/2 + METRE_TO_PIXEL_SCALE * estimated_location.pose.position.x;
	int estimated_robo_image_y = localization_result_image.size().height/2 + METRE_TO_PIXEL_SCALE * estimated_location.pose.position.y;

	/*int estimated_heading_image_x = estimated_robo_image_x + HEADING_GRAPHIC_LENGTH * cos(-target_yaw);
	int estimated_heading_image_y = estimated_robo_image_y + HEADING_GRAPHIC_LENGTH * sin(-target_yaw);
*/
	int estimated_heading_image_x = final_particle.particle_x + HEADING_GRAPHIC_LENGTH * cos(-target_yaw);
	int estimated_heading_image_y = final_particle.particle_x + HEADING_GRAPHIC_LENGTH * sin(-target_yaw);
	
	//std::cout << "yaw"<< target_yaw<< "pitch"<<target_pitch << "roll"<<target_roll ;
	cv::circle( localization_result_image, cv::Point(estimated_robo_image_x, estimated_robo_image_y), POSITION_GRAPHIC_RADIUS, CV_RGB(250,0,0), -1);
	cv::line( localization_result_image, cv::Point(estimated_robo_image_x, estimated_robo_image_y), cv::Point(estimated_heading_image_x, estimated_heading_image_y), CV_RGB(250,0,0), 10);

	estimate_pub.publish( estimated_location );
  }

  // This function publishes your localization result image and spins ROS to execute its callbacks
  void spin(){

	ros::Rate loop_rate(30);
	while (nh.ok()) {
	  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", localization_result_image).toImageMsg();
	  pub.publish(msg);

	  ros::spinOnce();
	  loop_rate.sleep();
	}
  }
};

int main(int argc, char** argv){

  ros::init(argc, argv, "localizer");
  Localizer my_loc(argc, argv);
  my_loc.spin();
}
