// Arm Planner Header
/*
 * Contains belief of world representation
 * Receives ball coordinates, velocities, and color from Kalman Filter
 * Receives goal coordinates and dimensions from vision
 * Outputs arm orientations to python arm controller
 */
#include <math.h>
#include <utility>
#include <vector>
#include <cfloat>
#include <Eigen/Core>
#include "ball.hpp"
#include <lcm/lcm-cpp.hpp>
#include <common/message_channels.hpp>
#include <common/messages/arm_path_t.hpp>
#include <common/messages/ball_detections_t.hpp>
#include <common/messages/ball_detection_t.hpp>

using Eigen::Vector2d;
using Eigen::Vector4d;

class ArmPlanner
	{
	private:
		std::vector< Point < double > > goals;
		std::vector< Ball > balls;
        
        lcm::LCM *lcm;

        double armInnerRadius = 0.100;
        double armOuterRadius = 0.175;
        int emptyFrames = 0;

        Point < double > armBase( 0.0, 0.0 );
        double bicepJointLength = 0.1;
        double elbowJointLength = 0.1;

	public:
		ArmPlanner( )
			{
			}

        void addGoals( std::vector< Point< double > > newGoals )
            {
            goals = newGoals;
            }

        void updateBall( const ball_detection_t &ball )
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

        // TODO: Add multi-ball tracking
        void updateBalls(const ball_detections_t &newBalls )
        {
            // each detection is either a new ball, existing ball, or False
            double minDist = -1;
            ball_detection_t bestDetection;

            // handle if the ball is a new ball
            if(newBalls.detections.empty())
            {
                emptyFrames++;
                return;
            }
            else 
            {
                // TODO: Make configurable
                if(emptyFrames > 100) { balls.clear(); }
                emptyFrames = 0;
            }

            for (size_t i = 0; i < newBalls.detections.size(); ++i)
            {
                ball_detection_t detection = newBalls[i];
                if(balls.empty()) {
                    // chose detection closest to the arm 
                    if(minDist == -1 || sqrt(detection.position[0]*detection.position[0] + 
                        detection.position[1]*detection.position[1]) < minDist)  
                    {
                        bestDetection = detection;
                        minDist = sqrt(detection.position[0]*detection.position[0] + 
                                    detection.position[1]*detection.position[1]);
                    }
                }
                else {
                    // chose the detection closest to previous ball
                    // TODO: Use a
                    Vector2d prevBallPos = balls[0].predict_coordinate();

                    if(minDist == -1) bestDetection = detection;
                    sqrt(detection.position[0]*detection.position[0] + 
                                    detection.position[1]*detection.position[1]);

                }
            }

            if(balls.empty()) { 
                balls.push_back( Ball(bestDetection.id, bestDetection.utime,
                    Eigen::Vector2d(bestDetection.position[0], bestDetection[1])) );
            }
            else {
                balls[0].update(bestDetection);
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
            return projection;
            }

        // check to see if the ball will reach radius of us within
        // 2 seconds
        // give a range of the start end of reachable 

        double ballSpotHeuristic( Point < double > spot )
            {
            return 1.0;
            }

        double temporaryHeuristic( std::pair< Point < double >, Point < double > > spots )
            {
            return 1.0;
            }

        Point < double > chooseGoal ( Ball &ball )
            {
            return Point < double >( -0.05, 0.3 );
            }

        std::pair < Point < double >, Point < double > > calculateWaypoints( Ball &ball, Point < double > &spot, Point < double > &goal )
        {
            // calculate the angle between the spot and goal
            double thetaToGoal = calculateAngleRadians( Point < double >( 0.0, 0.0 ), Point< double >( 0.0, 0.5 );
            double minimumVerticalVelocity = velocity.y / 3; // dependent on the elasticity of our putter

            
            // TODO remove these test overwrites when calculation is complete
            Point < double > first( 0.1, 0.08 );
            Point < double > second( 0.08, 0.15 );
            return std::make_pair( first, second );
        }

        std::pair< Point < double >, Point < double > > calculatePlan( )
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

            // a spot is a place to hit the ball from
            double bestSpotScore = DBL_MAX;
            Point < double > bestSpot;
            // pick best place to hit ball between
            for ( float interval = times.x;  interval <= times.y;  ++interval )
                {
                Point< double > spot = projectBall( *closest, interval );
                double spotScore = ballSpotHeuristic( spot );
                if ( spotScore < bestSpotScore )
                    {
                    bestSpot = spot;
                    bestSpotScore = spotScore; 
                    }
                }

            bestSpot = projectBall( *closest, times.x + ( times.y - times.x ) / 2 ); // TODO remove when decide on actual heuristic

            Point < double > goal = chooseGoal( *closest );
            return calculateWaypoints( *closest, bestSpot, goal );
        }


        void publishArmPlan( std::vector < double > angles )
            {
            }

        void publishPlan( std::pair < Point < double >, Point < double > > &pr )
            {
            arm_path_t path;
            path.waypoints_num = 1;

            path.waypoints[0][0] = pr.first.x;
            path.waypoints[0][1] = pr.first.y;
            path.speed = 1;

            lcm->publish( channel::ARM_PATH, &path );

            path.waypoints[0][0] = pr.second.x;
            path.waypoints[0][1] = pr.second.y;
            }
	};
