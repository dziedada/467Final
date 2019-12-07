// Planner Main
/*
 * LCM communication with EKF and Arm Controller
 */
#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <vector>
#include <utility>
#include <common/messages/arm_path_t.hpp>
#include <common/messages/ball_t.hpp>
#include <common/message_channels.hpp>
#include <common/point.hpp>
#include <planner/arm_planner.hpp>
#include <lcm/lcm-cpp.hpp>
#include <Eigen/Core>

using std::cout;
using std::endl;
using std::cerr;
using std::vector;
using std::pair;

class Handler
    {
    public:
        Handler( )
            {
            planner = ArmPlanner( );
            }

        ~Handler( ) { }

        void handleEKFMessage( const lcm::ReceiveBuffer *rbuf, const std::string &channel, const ball_detections_t *ball )
            {
            cout << " update received at " << ball->utime << endl;

            planner.updateBalls( *ball );

            std::vector<Vector2d> plan = planner.calculatePlan( );
            auto pos =  planner.balls[0].getPos();
            cout << pos << endl;
            cout << " calculated Plan at " << endl;
            //planner.publishPlan( plan );

            }
        
    private:
        ArmPlanner planner;
    };

int main( int argc, char ** argv )
	{
	cout << "Starting Planner..." << endl;

    lcm::LCM lcm;
    if ( !lcm.good( ) )
        return 1;

    cout << " - Completetd LCM initialization" << endl;
    Handler handler;
    cout << " - Created Handler object" << endl;
    lcm.subscribe( "BALL_DETECTIONS", &Handler::handleEKFMessage, &handler );
    cout << " - Subscribed to ARM_PATH" << endl;

    // sleep( 2 );

    // arm_path_t path;
    // path.waypoints_num = 2;
    // vector< vector< double > > waypoints( 2, vector< double >( 2, 0 ) );
    // waypoints[0][0] = 0.1;
    // waypoints[0][1] = 0.08;
    // waypoints[1][0] = 0.08;
    // waypoints[1][1] = 0.15;
    // path.waypoints = waypoints;
    // path.angles_num = 0;
    // std::vector<std::vector<double>> vec( 0, vector<double>( 0, 0 ) );
    // path.angles = vec;
    // path.speed = 1;
    
    lcm.publish( "ARM_PATH", &path );

    while ( 0 == lcm.handle( ) );

    return 0;
	}
