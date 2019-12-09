#pragma once

// Arm Planner Header
/*
 * Contains belief of world representation
 * Receives ball coordinates, velocities, and color from Kalman Filter
 * Receives goal coordinates and dimensions from vision
 * Outputs arm orientations to python arm controller
 */
#include <common/point.hpp>
#include <common/message_channels.hpp>
#include <common/messages/arm_path_t.hpp>
#include <common/messages/ball_detections_t.hpp>
#include <common/messages/ball_detection_t.hpp>
#include <planner/ball.hpp>
#include <planner/OuterLoopController.hpp>
#include <planner/Prediction.hpp>

#include <lcm/lcm-cpp.hpp>
#include <Eigen/Core>

#include <cmath>
#include <climits>
#include <cfloat>
#include <utility>
#include <vector>
#include <memory>
#include <condition_variable>
#include <mutex>
#include <thread>

using Eigen::Vector2d;
using Eigen::Vector4d;

class ArmPlanner
	{
	public:
        std::shared_ptr<std::condition_variable> cond_var;
        std::shared_ptr<std::mutex> mtx;
        lcm::LCM *lcm;
        OuterLoopController outer_loop_controller;

	std::vector< Point < double > > goals;
	std::vector< Ball > balls;
        

        double armInnerRadius = 0.100;
        double armOuterRadius = 0.175;
        int emptyFrames = 0;

        double bicepJointLength = 0.1;
        double elbowJointLength = 0.1;

	public:
	ArmPlanner(lcm::LCM* lcm_) : 
            cond_var {std::shared_ptr<std::condition_variable>(new std::condition_variable())},
            mtx {std::shared_ptr<std::mutex>(new std::mutex())}, lcm {lcm_},
            outer_loop_controller {OuterLoopController(lcm_)}
			{
                // std::thread([this]() {this->outer_loop_controller.runStateMachine();}).detach();
			}

        void addGoals( std::vector< Point< double > > newGoals )
            {
            goals = newGoals;
            }

        int currentId = 0;
        // TODO: Add multi-ball tracking
        void updateBalls(const ball_detections_t &newBalls )
        {
            std::lock_guard<std::mutex> lck(*mtx);
            // each detection is either a new ball, existing ball, or False
            double minDist = -1;
            ball_detection_t bestDetection;

            double corrThreshold = 0.5;
            std::vector< Ball * > corresponded;
            std::cout << "detected: " << newBalls.detections.size() << std::endl;
            for (size_t i = 0; i < newBalls.detections.size(); ++i)
            {
                ball_detection_t detection = newBalls.detections[i];
                Eigen::Vector2d detectionPosition( detection.position[0], detection.position[1] );
                std::cout << "Detection " << detectionPosition.x() << " " << detectionPosition.y() << std::endl;

                Ball * closest;
                double closestDistance = DBL_MAX;
                for ( auto &ball: balls )
                {
                    if ( corresponded.end() != std::find( corresponded.cbegin(), corresponded.cend(), &ball ) ||
                        ball.color != detection.color )
                    {
                        continue;
                    }

                    Eigen::Vector4d predictionState = ball.predict_coordinate( newBalls.utime );
                    Eigen::Vector2d prediction( predictionState.x(), predictionState.y() );
                    // std::cout << "b= " << ball.coordinate << std::endl;
                    // std::cout << "p= " << predictionState << std::endl;
                    double distance = (prediction - detectionPosition).norm();
                    if ( distance < corrThreshold && distance < closestDistance )
                        {
						std::cout << "b= " << ball.coordinate << std::endl;
						std::cout << "p= " << predictionState << std::endl;

                        closest = &ball;
                        closestDistance = distance;
                        }
                }
                if ( closestDistance == DBL_MAX )
                {
                    detection.utime = newBalls.utime;
                    balls.push_back( Ball( currentId, detection.color, detection.utime, Eigen::Vector2d( detection.position[0], detection.position[1] ) ) );
                    ++currentId;
                }
                else 
                {
                    detection.utime = newBalls.utime;
                    closest->update( detection );
                    corresponded.push_back( closest );
                }
            }
            cout << "enter purge code " << endl;
            Ball * bestBall;// = nullptr;
			int64_t bestTime = ULLONG_MAX;
            for( auto it = balls.begin(); it != balls.end(); )
            {
                if(it == balls.end()) break;
                std::cout << "checking new ball" << std::endl;
                if ( corresponded.end() == std::find( corresponded.cbegin(), corresponded.cend(), &*it ) )
                {
					std::cout << "ball not in corresponded" << std::endl;
                    //std::cout << "not corresponded";
                    std::cout << it->odds << std::endl;
                    if ( it->odds < -10 )
                        {
                        // purge ball
                        std::cout << "Purged ball... "<< std::endl;// << balls.size() << " left";
                        it = balls.erase( it );
                        std::cout << "Purged ball.2.. "<< std::endl;
                        }
                    else
                        {
                        std::cout << "decreasing odds" << std::endl;
                        it->odds -= 1;
                        ++it;
                        }
                }
                else
                {
					std::cout << "ball in corresponded" << std::endl;
                    if ( it->reachPrediction.ball_in_range_time_ < bestTime )
						{
						std::cout << "reachPrediction" << std::endl;
                        bestTime = it->reachPrediction.ball_in_range_time_;
						bestBall = &*it;
                        std::cout << "set best" << std::endl;
						}
                    ++it;
                }
            }
            std::cout << balls.size() << " balls left." << std::endl;
			//outer_loop_controller.update_target( bestBall->reachPrediction );
			if ( bestBall )
				{
				// std::cout << "position: " << bestBall->reachPrediction.ball_inrange_position_ 
                //     << std::endl;
				// publishPlan( bestBall->reachPrediction.ball_inrange_position_ );
				}

            std::cout << "corresponded: " << balls.size() << std::endl;
            cond_var->notify_all();
        }

           
             
        float convertUTimeToSeconds( int64_t utime )
            {
            return static_cast< float > ( utime ) / 10e6;
            }

        int64_t convertSecondsToUTime( float seconds )
            {
            return static_cast< int64_t > ( seconds * 10e6 );
            }

        double distanceToBase( Point< double > pt )
            {
            return sqrt( pt.x * pt.x + pt.y * pt.y );
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

            // Project the ball out between time intervals
            std::pair<double, double> times = closest->projectTimeToReach(armOuterRadius, armInnerRadius);
            if(times.first == -1) return {Vector2d(0,0)};

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
            // TODO: Rigtht or left when far 
            Vector2d goal = chooseGoal( *closest );
            return calculateWaypoints( *closest, bestSpot, goal );
        }


        void publishArmPlan( std::vector < double > angles )
            {
            }

        void publishPlan( Vector2d endpoint )
            {
            arm_path_t path;
            path.waypoints_num = 1;
            
            std::vector<std::vector< double > > waypoints( 1, std::vector<double>(2, 0));

            // Flip X and Y for Arm Coordinate system
            waypoints[0][0] = endpoint[1];
            waypoints[0][1] = endpoint[0];
            path.waypoints = waypoints;
            path.speed = 1.0;
            path.angles_num = 1;
            std::vector<std::vector< double > > angles( 1, std::vector<double>(4, 0));
            path.angles = angles;

            lcm->publish( "ARM_PATH", &path );

            // path.waypoints[0][0] = pr.second.x;
            // path.waypoints[0][1] = pr.second.y;
            }

        const std::vector<Ball>& getBalls()
        {
            return balls;
        }
	};
