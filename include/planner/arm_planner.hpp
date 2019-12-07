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
#include <common/point.hpp>
#include <common/message_channels.hpp>
#include <common/messages/arm_path_t.hpp>
#include <common/messages/ball_detections_t.hpp>
#include <common/messages/ball_detection_t.hpp>

using Eigen::Vector2d;
using Eigen::Vector4d;

class ArmPlanner
	{
	public:
		std::vector< Point < double > > goals;
		std::vector< Ball > balls;
        
        lcm::LCM *lcm;

        double armInnerRadius = 0.100;
        double armOuterRadius = 0.175;
        int emptyFrames = 0;

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

        // void updateBall( const ball_detection_t &ball )
        // {

        //     // add the incoming balls to our vector of balls and mark the time
        //     bool exists = false;
        //     for ( auto &existing: balls )
        //     {
        //         if ( ball.id == existing.id )
        //             existing = ball;
        //     }

        //     if ( !exists )
        //         balls.push_back( ball );
        // }

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

            double corrThreshold = 0.1;
            std::vector< Ball * > corresponded;
            std::cout << "detected: " << newBalls.detections.size() << std::endl;
            for (size_t i = 0; i < newBalls.detections.size(); ++i)
            {
                ball_detection_t detection = newBalls.detections[i];
                Eigen::Vector2d ballPosition( detection.position[0], detection.position[1] );

                Ball * closest;
                double closestDistance = DBL_MAX;
                for ( auto &ball: balls )
                {
                    if ( corresponded.end() == std::find( corresponded.cbegin(), corresponded.cend(), &ball ) ||
                        ball.color != detection.color )
                    {
                        continue;
                    }

                    Eigen::Vector4d predictionState = ball.predict_coordinate( detection.utime );
                    Eigen::Vector2d prediction( predictionState.x(), predictionState.y() );
                    double distance = (prediction - ballPosition).norm();
                    if ( distance < corrThreshold && distance < closestDistance )
                        {
                        closest = &ball;
                        closestDistance = distance;
                        }
                }
                if ( closestDistance == DBL_MAX )
                {
                    balls.push_back( Ball( 0, detection.color, detection.utime, Eigen::Vector2d( detection.position[0], detection.position[1] ) ) );
                }
                else 
                {
                    closest->update( detection );
                    corresponded.push_back( closest );
                }
            }
            
            for( auto it = balls.begin(); it != balls.end(); )
            {
                if ( corresponded.end() == std::find( corresponded.cbegin(), corresponded.cend(), &*it ) )
                {
                    if ( (*it).odds < -10 )
                        {
                        // purge ball
                        it = balls.erase( it );
                        }
                    else
                        {
                        (*it).odds -= -1;
                        ++it;
                        }
                }
                else
                {
                    ++it;
                }
            }



                /*
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
                    Vector4d prevBallPos = balls[0].predict_coordinate(detection.utime);

                    if(minDist == -1) bestDetection = detection;
                    sqrt(detection.position[0]*detection.position[0] + 
                                    detection.position[1]*detection.position[1]);

                }*/
            /*if(balls.empty()) { 
                balls.push_back( Ball(0, bestDetection.color, bestDetection.utime,
                    Eigen::Vector2d(bestDetection.position[0], bestDetection.position[1])) );
            }
            else {
                balls[0].update(bestDetection);
            }*/
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
        Vector4d projectBall( Ball &ball, double time )
        {
            ball.predict_coordinate(time);
        }

        // check to see if the ball will reach radius of us within
        // 2 seconds
        // give a range of the start end of reachable 

        double ballSpotHeuristic( Vector4d spot )
            {
            return 1.0;
            }

        double temporaryHeuristic( std::pair< Point < double >, Point < double > > spots )
            {
            return 1.0;
            }

        Vector2d chooseGoal ( Ball &ball )
            {
            return Vector2d( -0.05, 0.3 );
            }

        std::vector<Vector2d> calculateWaypoints( Ball &ball, Vector4d &spot, Vector2d &goal )
        {
            // calculate the angle between the spot and goal
            Vector2d spotPos = Vector2d(spot.x(), spot.y());
            // double thetaToGoal = calculateAngleRadians( Point < double >( 0.0, 0.0 ), Point< double >( 0.0, 0.5 );
            auto vecToGoal = (goal - spotPos);
            double thetaToGoal = atan(vecToGoal.y() / vecToGoal.x());
            double minimumVerticalVelocity = spot[3] / 3; // dependent on the elasticity of our putter

            return {spotPos};
        }   

        std::vector<Vector2d> calculatePlan( )
        {
            if(balls.empty()) return std::vector<Vector2d>();
            // Find closest ball
            Ball * closest;
            double closestDistance = DBL_MAX;
            for ( auto &ball: balls )
                {
                double distance = ball.getPos().norm();
                if ( distance < closestDistance )
                    {
                    closest = &ball;
                    closestDistance = distance;
                    }
                }

            // TODO: 
            // Project the ball out between time intervals
            std::pair<double, double> times = closest->projectTimeToReach(armOuterRadius, armInnerRadius);

            // a spot is a place to hit the ball from
            double bestSpotScore = DBL_MAX;
            Vector4d bestSpot;
            
            // pick best place to hit ball between
            float stepSize = (times.second - times.first) / 10.0;
            for ( float interval = times.first;  interval <= times.second; interval += stepSize )
                {
                Vector4d spot = closest->predict_coordinate((double)interval);
                double spotScore = ballSpotHeuristic( spot );
                if ( spotScore < bestSpotScore )
                    {
                    bestSpot = spot;
                    bestSpotScore = spotScore; 
                    }
                }

            Vector2d goal = chooseGoal( *closest );
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
