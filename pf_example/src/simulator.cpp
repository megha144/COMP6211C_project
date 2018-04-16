
#include <math.h>
#include <string.h>

// Include ROS messages
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <pf_example/RangeMeasurement.h>
#include <tf/transform_broadcaster.h>

# define LOOP_RATE 10



/** \class VehicleSimulator
 * main class
 **/
class VehicleSimulator{

    public:

        /** Default constructor to initialize everything **/
        VehicleSimulator(ros::NodeHandle& _node, const std::string & vehicle_state_topic, 
                const std::string & vehicle_speed_topic,const std::string & range_topic) {
            pub_vehicle_state = _node.advertise<geometry_msgs::Vector3Stamped>(vehicle_state_topic,  1);
            pub_vehicle_speed = _node.advertise<geometry_msgs::Vector3Stamped>(vehicle_speed_topic,  1);
            pub_range = _node.advertise<pf_example::RangeMeasurement>(range_topic,  1);
        }

        static double SQR(double x) {return x*x;}

        void Loop() {

            printf("Entering loop ...");
            ros::Rate loop_rate(LOOP_RATE);

            ros::Time start = ros::Time::now();
            ros::Time last_range = ros::Time::now();
            loop_rate.sleep();
            srand48((int)(start.toSec()));

            while(ros::ok()){

                ros::Time now = ros::Time::now();
                double dt = (now - start).toSec();
                geometry_msgs::Vector3Stamped state, speed;
                state.header.stamp = now;
                state.header.frame_id = "/world";
                state.vector.x = cos(dt*M_PI/5)*1.0;
                state.vector.y = sin(dt*M_PI/3)*0.5;
                state.vector.z = sin(dt*M_PI/7)*0.5;

                speed.header.stamp = now;
                speed.header.frame_id = "/world";
                speed.vector.x = - (1*M_PI/5) * sin(dt*M_PI/5)  + (-1 + 2*drand48())*0.05;
                speed.vector.y = + (0.5*M_PI/3) * cos(dt*M_PI/3) + (-1 + 2*drand48())*0.05;
                speed.vector.z = + (0.5*M_PI/7) * cos(dt*M_PI/7) + (-1 + 2*drand48())*0.05;

                pf_example::RangeMeasurement range;
                range.header.stamp = now;
                range.header.frame_id = "/vehicle";
                range.value = sqrt(SQR(state.vector.x)+SQR(state.vector.y)+SQR(state.vector.z)) * (1 + (-1 + 2*drand48())*0.10);

                transform.setOrigin( tf::Vector3(state.vector.x, state.vector.y, state.vector.z) );
                transform.setRotation( tf::createQuaternionFromYaw(0) );
                br.sendTransform(tf::StampedTransform(transform, now, "/world", "/vehicle"));
                pub_vehicle_state.publish(state);
                pub_vehicle_speed.publish(speed);
                if ((now-last_range).toSec() > 0.5) {
                    pub_range.publish(range);
                    last_range = now;
                }

                loop_rate.sleep();
            }
        };



    protected:
        // ros node, publishers, subscribers and services
        ros::Publisher 	pub_vehicle_state;
        ros::Publisher 	pub_vehicle_speed;
        ros::Publisher 	pub_range;

        // tf
        tf::Transform transform;
        tf::TransformBroadcaster br;

};




/** Main function **/
int main( int argc, char** argv )
{
	// calling ros init before parsing command line
	ros::init(argc, argv, "simulator");
	ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
	
	VehicleSimulator vehicle_simulator(nh, "/vehicle/state", "/vehicle/speed", "/sensor/range");

	ROS_INFO("Starting Vehicle Simulator"); 

	vehicle_simulator.Loop();

	return 0;
}


