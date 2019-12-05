// Arm Planner Header
/*
 * Contains belief of world representation
 * Receives ball coordinates, velocities, and color from Kalman Filter
 * Receives goal coordinates and dimensions from vision
 * Outputs arm orientations to python arm controller
 */
#include <math.h>
#include <vector>
#include <cfloat>
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

        double armInnerRadius = 0.100;
        double armOuterRadius = 0.175;

	public:
		ArmPlanner( )
			{
			}

        void addGoals( std::vector< Point< double > > newGoals )
            {
            goals = newGoals;
            }

        void updateBall( Ball ball )
            {
            // add the incoming balls to our vector of balls and mark the time
            bool exists = false;
            for ( auto &existing: balls )
                {
                if ( ball.id == existing.id )
                    existing = ball;
                }

            if ( !exists )
                balls.push_back( ball );
            }

        void updateBalls( std::vector< Ball > newBalls )
            {
            // add the incoming balls to our vector of balls and mark the time
            for ( auto &ball: newBalls )
                {
                updateBall( ball );
                }
            }
        
        float convertUTimeToSeconds( int64_t utime )
            {
            return static_cast< float > ( utime ) / 10e9;
            }

        int64_t convertSecondsToUTime( float seconds )
            {
            return static_cast< int64_t > ( seconds * 10e9 );
            }

        double distanceToBase( Point< double > pt )
            {
            return sqrt( pt.x * pt.x + pt.y * pt.y );
            }

        // project the ball out time seconds
        Point < double > projectBall( Ball &ball, int64_t time )
            {
            Point< double > projection = ball.coordinate;
            projection.x += ball.velocity.x * time;
            projection.y += ball.velocity.y * time;
            }

        // check to see if the ball will reach radius of us within
        // 2 seconds
        // give a range of the start end of reachable 
        Point< float > projectTimeToReach( Ball &ball )
            {
            float start = -1, end = 2;
            for ( float interval = 0.0;  interval < 2.0;  interval += 0.05 )
                {
                Point< double > projection = projectBall( ball, convertSecondsToUTime( interval ) );

                if ( start == -1 && distanceToBase( projection ) < armOuterRadius )
                    {
                    start = interval;
                    }
                else if ( start != -1 && distanceToBase( projection ) < armInnerRadius )
                    {
                    end = interval - 0.05; 
                    }
                }
            }

        void calculatePlan( )
            {
            // Find closest ball
            Ball * closest;
            double closestDistance = DBL_MAX;
            for ( auto &ball: balls )
                {
                double distance = sqrt ( ball.coordinate.x * ball.coordinate.x + ball.coordinate.y * ball.coordinate.y );
                if ( distance < closestDistance )
                    {
                    closest = &ball;
                    closestDistance = distance;
                    }
                }

            // Project the ball out between time intervals
            Point< float > times = projectTimeToReach( *closest );

            // pick best place to hit ball between
            for ( float interval = times[ 0 ];  interval
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
