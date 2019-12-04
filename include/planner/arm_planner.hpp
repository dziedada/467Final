// Arm Planner Header
/*
 * Contains belief of world representation
 * Receives ball coordinates, velocities, and color from Kalman Filter
 * Receives goal coordinates and dimensions from vision
 * Outputs arm orientations to python arm controller
 */
#include <vector>
#include "ball.hpp"
#include <lcm/lcm-cpp.hpp>
#include <common/message_channels.hpp>
#include <common/messages/arm_path_t.hpp>

class ArmPlanner
	{
	private:
		std::vector< Point < double > > goals;
		std::vector< Ball > balls;
        
        lcm::LCM *lcm;

	public:
		ArmPlanner( )
			{
			}

        void addGoals( std::vector< Point< double > > newGoals )
            {
            goals = newGoals;
            }

        void updateBalls( std::vector< Ball > newBalls )
            {
            // add the incoming balls to our vector of balls and mark the time
            for ( auto &ball: newBalls )
                {
                for ( auto &existing: balls )
                    {
                    if ( ball.id == existing.id )
                        existing = ball;
                    }
                    balls.push_back( ball );
                }
            }
        
        void sendPlan( )
            {
            arm_path_t path;
            path.waypoints_num = 1;

            Point < double > first( 0.1, 0.08 );
            Point < double > second( 0.08, 0.15 );

            path.waypoints[0][0] = first.x;
            path.waypoints[0][1] = first.y;
            path.speed = 1;

            lcm->publish( channel::ARM_PATH, &path );

            path.waypoints[0][0] = second.x;
            path.waypoints[0][1] = second.y;
            }
	};
