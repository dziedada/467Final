// Planner Main
/*
 * LCM communication with EKF and Arm Controller
 */
#include <iostream>
#include <stdio.h>
#include <vector>
#include <utility>
#include <common/messages/arm_path_t.hpp>
#include <common/messages/ball_t.hpp>
#include <common/message_channels.hpp>
#include <common/point.hpp>
#include <planner/arm_planner.hpp>
#include <lcm/lcm-cpp.hpp>

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

            //pair< Point < double >, Point < double > > plan = planner.calculatePlan( );
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
    lcm.subscribe( "ARM_PATH", &Handler::handleEKFMessage, &handler );
    cout << " - Subscribed to ARM_PATH" << endl;

    while ( 0 == lcm.handle( ) );

    return 0;
	}
