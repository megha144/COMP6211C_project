#include <math.h>
#include <string.h>
#include <vector>
#include <map>
#include <boost/thread/locks.hpp>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
     

/** Exercise: Implement a particle filter by modifying the function marked with
 * "TODO". This implementation depends on the GNU Scientific Library for random
 * number generation. Install it with 
 * sudo apt-get install libgsl0-dev
 *
 * **/


// Include ROS messages
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <pf_example/RangeMeasurement.h>
#include <tf/transform_broadcaster.h>

# define NUM_PARTICLES 200



/** \class VehicleLocalisation
 * main class
 **/
class VehicleLocalisation{

    protected:
        // Random number generator
        const gsl_rng_type * T;
        gsl_rng * r;
     

    public:

        /** Default constructor to initialize everything **/
        VehicleLocalisation(ros::NodeHandle& _node, 
                const std::string & vehicle_speed_topic,
                const std::string & range_topic,
                const std::string & pc_topic) {
            pub_pc = _node.advertise<sensor_msgs::PointCloud>(pc_topic,  1);
            sub_vehicle_speed = _node.subscribe(vehicle_speed_topic,  1, &VehicleLocalisation::speedCallback,this);
            sub_range = _node.subscribe(range_topic,  1, &VehicleLocalisation::rangeCallback,this);

            int num_particles = NUM_PARTICLES;
            _node.param<int>("num_particles", num_particles, NUM_PARTICLES);
            last_speed_msg = last_estimate = ros::Time::now();

            /* create a generator chosen by the 
               environment variable GSL_RNG_TYPE */

            gsl_rng_env_setup();

            T = gsl_rng_default;
            r = gsl_rng_alloc (T);


            // Initialise the particles in a 3D box
            particles.resize(num_particles);
            normalizeParticles();
            for (unsigned int i=0;i<particles.size();i++) {
                particles[i].x = gsl_ran_flat(r,-2,2);
                particles[i].y = gsl_ran_flat(r,-2,2);
                particles[i].z = gsl_ran_flat(r,-0.5,0.5);
            }
        }

        ~VehicleLocalisation() {
            gsl_rng_free (r);
        }

    protected:
        struct Particle {
            float x,y,z;
            float importance;
            Particle() {x=y=z=importance=0;}
        };

        boost::mutex mutex;
        std::vector<Particle> particles;
        ros::Time last_estimate, last_speed_msg;
        typedef std::multimap< float,unsigned int,std::less<float> > CDF;

    protected:
        static double SQR(double x) {return x*x;}

        double measurementLikelihood(const Particle & p, double range) {
            // TODO: Compute the importance of each particle here, based 
            // on its x,y,z value, assuming range is the measurement of the
            // distance to the origin. It does not have to be normalised.

            return 1.0;
        }

        Particle sampleFromMotionModel(float dt, const Particle & p, double vx, double vy, double vz) {
            Particle newp = p;
            // TODO: set the state of newp so that it accounts for the velocity
            // information (vx,vy,vz). Remember that it should be "sampling",
            // not just integration. Use gsl_ran_gaussian(r, sigma) to draw a
            // number according to a gaussian distribution centered on 0, with
            // covariance sigma: 
            // http://www.gnu.org/software/gsl/manual/html_node/Random-Number-Distributions.html

            newp.x = p.x;
            newp.y = p.y; 
            newp.z = p.z; 

            return newp;
        }

        void speedCallback(const geometry_msgs::Vector3Stamped& msg){
            boost::unique_lock<boost::mutex> lock(mutex);
            last_estimate = msg.header.stamp;
            float dt = (msg.header.stamp - last_speed_msg).toSec();
            last_speed_msg = msg.header.stamp;

            // Apply motion model to particles
            for (unsigned int i=0;i<particles.size();i++) {
                particles[i] = sampleFromMotionModel(dt, particles[i], 
                        msg.vector.x, msg.vector.y, msg.vector.z); 
            }

            // Make sure the particles don't go wandering too far. This is not
            // stricly required, but makes rviz more stable
            for (unsigned int i=0;i<particles.size();i++) {
                if (particles[i].x < -2) particles[i].x = -2;
                if (particles[i].x > +2) particles[i].x = +2;
                if (particles[i].y < -2) particles[i].y = -2;
                if (particles[i].y > +2) particles[i].y = +2;
                if (particles[i].z < -2) particles[i].z = -2;
                if (particles[i].z > +2) particles[i].z = +2;
            }
            
            // Publishing results
            publishEstimate();
        };

        void rangeCallback(const pf_example::RangeMeasurement& msg){
            boost::unique_lock<boost::mutex> lock(mutex);
            last_estimate = msg.header.stamp;

            // Update importance

            for (unsigned int i=0;i<particles.size();i++) {
                particles[i].importance = measurementLikelihood(particles[i], msg.value);
            }


            normalizeParticles();

            // Resample the particles
            std::vector<Particle> oldparticles = particles;

            for (unsigned int i=0;i<particles.size();i++) {
                // TODO: Implement an importance-based sampling that will
                // select particle j with a likelihood proportional to its
                // importance. Hint: use the STL multimap class (in particular
                // the lower_bound function, and get inspiration from inversion
                // sampling.
                unsigned int j = i;
                particles[i] =  oldparticles[j];
            }

            normalizeParticles();
            // Publishing results
            publishEstimate();
        };

        void normalizeParticles() {
            float sum = 0;
            for (unsigned int i=0;i<particles.size();i++) {
                sum += particles[i].importance;
            }
            if (sum > 0) {
                for (unsigned int i=0;i<particles.size();i++) {
                    particles[i].importance /= sum;
                }
            } else {
                // In this case all the weight are zero. It could be at the
                // initialisation, or it could be that all the measurement
                // where wrong. We set a uniform importance value.
                for (unsigned int i=0;i<particles.size();i++) {
                    particles[i].importance = 1./particles.size();
                }
            }
        }

        void publishEstimate() {
            // Compute particle mean
            float mx=0,my=0,mz=0;
            for (unsigned int i=0;i<particles.size();i++) {
                mx += particles[i].x * particles[i].importance;
                my += particles[i].y * particles[i].importance;
                mz += particles[i].z * particles[i].importance;
            }
            
            // publish estimation
            transform.setOrigin( tf::Vector3(mx,my,mz));
            transform.setRotation( tf::createQuaternionFromYaw(0) );
            br.sendTransform(tf::StampedTransform(transform, last_estimate, "/world", "/estimate"));

            // publish the particles as a point cloud
            sensor_msgs::PointCloud pc;
            pc.header.frame_id = "/world";
            pc.header.stamp = last_estimate;
            pc.channels.resize(1);
            pc.channels[0].name="intensity";
            pc.channels[0].values.resize(particles.size());
            pc.points.resize(particles.size());

            for (unsigned int i=0;i<particles.size();i++) {
                pc.channels[0].values[i] = particles[i].importance;
                pc.points[i].x = particles[i].x;
                pc.points[i].y = particles[i].y;
                pc.points[i].z = particles[i].z;
            }
            pub_pc.publish(pc);
        }


    protected:
        // ros node, publishers, subscribers and services
        ros::Subscriber 	sub_vehicle_speed;
        ros::Subscriber 	sub_range;
        ros::Publisher 	    pub_pc;

        // tf
        tf::Transform transform;
        tf::TransformBroadcaster br;

};




/** Main function **/
int main( int argc, char** argv )
{
	// calling ros init before parsing command line
	ros::init(argc, argv, "localisation");
	ros::NodeHandle nh("~");
	
	VehicleLocalisation vehicle_Localisation(nh, "/vehicle/speed", "/sensor/range", "/localisation/particles");

	ROS_INFO("Starting Vehicle Localisation"); 

    ros::spin();

	return 0;
}


