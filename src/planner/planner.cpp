// Planner Main
/*
 * LCM communication with EKF and Arm Controller
 */
#include <iostream>
#include <stdio.h>
#include <common/messages/arm_path_t.hpp>
#include <common/messages/ball_detections_t.hpp>
#include <common/messages/ball_detection_t.hpp>
#include <common/message_channels.hpp>
#include <planner/arm_planner.hpp>
#include <lcm/lcm-cpp.hpp>

using std::cout;
using std::endl;
using std::cerr;

class Handler
    {
    public:
        Handler( )
            {
            }

        ~Handler( ) { }

        void handleEKFMessage( const lcm::ReceiveBuffer *rbuf, const std::string &channel, const ball_detections_t *balls )
            {
            cout << "Received " << balls->num_detections << " balls from EKF" << endl;
            }
        
        void sendArmPathMessage( )
            {
            cout << "Sending arm a plan" << endl;
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

    Handler handler;
    lcm.subscribe( "CHANNEL", &Handler::handleEKFMessage, &handler );

    while ( 0 == lcm.handle( ) );

    return 0;
	}
