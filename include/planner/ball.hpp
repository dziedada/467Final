#pragma once

// Ball Class
/*
 * Contains the dynamics of the Ball as well as a  
 * Standard Kalman filter to track it over time
 */

#include <common/point.hpp>
#include <common/messages/ball_detections_t.hpp>
#include <common/messages/ball_detection_t.hpp>

#include <Eigen/Core>

#include <planner/Prediction.hpp>

#include <cmath>
#include <iostream>
#include <vector>
#include <utility>
#include <list>
#include <deque>
#include <chrono>
#include <queue>
#include <limits>

using Eigen::Vector2d;
using Eigen::Vector4d;

using std::cout;
using std::endl;
using std::numeric_limits;
using std::chrono::duration_cast;
using std::chrono::seconds;
using std::chrono::microseconds;

int64_t seconds_to_microsends(double seconds)
{
	return static_cast<int64_t>(seconds * 10e6);
}

class Ball
	{
	private:
		int id;
		int color;
		int64_t utime; // previous utime
        int odds; // odds we think this ball is fake news
        // int64_t inputTime; // EKF system utime
        Vector2d meas;
		Vector2d coordinate;					// ( x, y )
		Vector2d prevDetectionPos;
		Vector2d velocity;					// ( v_x, v_y )
		Vector2d velocity_coord;
		Vector2d velocity_coord_avg;
		Vector2d coordinatePrediction;	// ( x_1, y_2 )
		// vel calculated by projected points

		std::vector<double> coordLine; // [m, b]

		std::deque<Vector2d> detectionHistory;
		std::deque< Vector2d > coordHistory;
		std::deque<double> speedHistory;
		std::deque< Vector2d > velocityHistory;
		std::deque< Vector2d > velocityCoordHistory;
		int HISTORY_SIZE = 10;
		Prediction reachPrediction;
        friend class ArmPlanner;
		friend class TrackingVis;

	public:

		Ball( int id_, int col, int64_t time, Vector2d coord, Vector2d vel = Vector2d(0, 0) )
				: id( id_ ), color( col ), utime( time ), odds( 0 ),
                coordinate( coord ), velocity( vel )
		{
			meas = coord;

			// Initialize coord history   
			for ( int i = 0; i < HISTORY_SIZE; ++i )
			{
				coordHistory.push_back(coord);
			}

			for (int i = 0; i < HISTORY_SIZE; ++i) {
				detectionHistory.push_back(coord);
			}

			for ( int i = 0; i < HISTORY_SIZE; ++i )
			{
				speedHistory.push_back(vel.norm());
			}

			for ( int i = 0; i < HISTORY_SIZE; ++i )
			{
				velocityHistory.push_back( Vector2d( 0.0, 0.0 ) );
			}
			for ( int i = 0; i < HISTORY_SIZE; ++i )
			{
				velocityCoordHistory.push_back( Vector2d( 0.0, 0.0 ) );
			}

			velocity_coord_avg = Vector2d(0.0, 0.0);
			reachPrediction.ball_in_range_time_ = numeric_limits<int64_t>::max();
			reachPrediction.ball_inrange_position_ = Vector2d( 0.0, 0.0 );
			reachPrediction.ball_inrange_velocity_ = Vector2d( 0.0, 0.0 );
			reachPrediction.goal_ = Vector2d( 0.0, 0.0 );
			reachPrediction.utime_ = duration_cast<microseconds>(std::chrono::system_clock::now()
				.time_since_epoch()).count();
		}

		std::vector<double> GetLinearFit(const std::deque<Vector2d>& data)
		{
		    double xSum = 0, ySum = 0, xxSum = 0, xySum = 0, slope, intercept;
		    for (long i = 0; i < data.size(); i++)
		    {
		        xSum += data[i].x();
		        ySum += data[i].y();
		        xxSum += data[i].x() * data[i].x();
		        xySum += data[i].x() * data[i].y();
		    }
		    slope = (data.size() * xySum - xSum * ySum) / (data.size() * xxSum - xSum * xSum);
		    intercept = (ySum - slope * xSum) / data.size();
		    std::vector<double> res;
		    res.push_back(slope);
		    res.push_back(intercept);
		    return res;
		}

		// projects point onto the line
		Vector2d projectPt(Vector2d pt, double slope, double intercept)
		{
			//https://en.wikibooks.org/wiki/Linear_Algebra/Orthogonal_Projection_Onto_a_Line
			pt.y() = pt.y() - intercept;
			Vector2d lineV(1, slope);
			// fix: the projected point p is on a line y = kx, not on y = kx + b
			Vector2d p = ( pt.dot(lineV) / (lineV.dot(lineV)) ) * lineV;
			p[1] += intercept;
			return p;
		}

		void calcAvgVelocity() {
			// velocity_coord_avg = (velocity_coord_avg * HISTORY_SIZE - velocityCoordHistory[0] 
			//	+ velocity_coord) / (double)HISTORY_SIZE;
			// TODO: use avg vel do prediction
			velocity_coord_avg = Vector2d(0, 0);
			for ( auto &it: velocityCoordHistory )
				{
				velocity_coord_avg.x() += it.x();
				velocity_coord_avg.y() += it.y();
				}
			velocity_coord_avg /= (double)HISTORY_SIZE;


		}

		void update(const ball_detection_t &detection)
		{
            odds += 1;
			double dT = (double)(detection.utime - utime) / (double)1000000;

            velocity = Vector2d( ( detection.position[0] - prevDetectionPos[0] ) / dT, ( detection.position[1] - prevDetectionPos[1] ) / dT );
            Vector2d detectionPos = Vector2d( detection.position[0], detection.position[1] );

			meas = detectionPos;

            prevDetectionPos = detectionPos;
			detectionHistory.pop_front();
			detectionHistory.push_back(detectionPos);

            velocityHistory.pop_front();
			velocityHistory.push_back( velocity );


			// speedHistory.pop_front();
			// speedHistory.push_back((coordHistory.back() - coordHistory.back()-1).norm()/dT);
			// use a line of best fit to get the coordinate prediction
			coordLine = GetLinearFit(detectionHistory);
			coordinate = projectPt(detectionPos, coordLine[0], coordLine[1]);
			
			velocity_coord = Vector2d( ( coordinate[0] - coordHistory[HISTORY_SIZE - 1][0] ) / dT, ( coordinate[1] - coordHistory[HISTORY_SIZE - 1][1] ) / dT );
			velocityCoordHistory.pop_front();
			velocityCoordHistory.push_back( velocity_coord );
			calcAvgVelocity();

			coordHistory.pop_front();
			// fix: push coord instead of detection
			coordHistory.push_back(coordinate);

			utime = detection.utime;
            //std::cout << "utime " << utime << " dT " << dT << std::endl;
		}
		
		void updatePrediction( const double outerRadius )
			{
			for ( double interval = 0.01; interval < 3.0; interval += 0.01 )
				{
				Vector4d prediction = predict_coordinate( interval );
				Vector2d position = Vector2d( prediction.x(), prediction.y() );
				//std::cout << "distance = " << position.norm() << std::endl;
				if ( position.norm() < outerRadius )
					{
					Vector2d velocity = Vector2d( prediction[2], prediction[3] );
					reachPrediction.utime_ = std::chrono::duration_cast<std::chrono::microseconds>(
							std::chrono::system_clock::now().time_since_epoch()).count();
					// Should be interval * 10e6 + utime_of_detection TODO
					reachPrediction.ball_in_range_time_ = utime + seconds_to_microsends(interval);
					//cout << "reachPrediction time " << reachPrediction.ball_in_range_time_ << std::endl;
					reachPrediction.ball_inrange_position_ = position;
					reachPrediction.ball_inrange_velocity_ = velocity;
					reachPrediction.goal_ = position;
					return;
					}
				}
			}

		bool operator==(const Ball &other)
		{
			return this->id == other.id;
		}

		// Check out realsense utime now

        Vector4d predict_coordinate(int64_t time)
        {
        	
        	double dT = (double)(time - utime) / (double)1000000;
        	//cout << "PREDICT COORD DT: " << dT << "\n"; 
        	return predict_coordinate(dT);
        }

        // predict out the ball in dt seconds
        // TODO refactor to predict state
		Vector4d predict_coordinate( double dT)
		{
			// Point History Estimation ( using last 5 points )
			Vector2d velocityEstimate( 0.0, 0.0 );
			for ( auto &it: velocityHistory )
				{
				velocityEstimate.x() += it.x();
				velocityEstimate.y() += it.y();
				}
			velocityEstimate /= (double)HISTORY_SIZE;

			//std::cout << "vel est final: " << velocityEstimate << std::endl;
			Vector2d pointEstimate = coordinate + (velocityEstimate * dT);
			// std::cout << "prediction: ";
   //          std::cout << "dT " << dT << " dx " << velocityEstimate.x() << " dy " << velocityEstimate.y() << endl;
            //return Vector4d(predState.at<double>(0), predState.at<double>(1), 
            			    //predState.at<double>(2), predState.at<double>(3)); 

			// TODO: IS this correct?

			return Vector4d( pointEstimate.x(), pointEstimate.y(),
							velocityEstimate.x(), velocityEstimate.y());
		}

		// check to see if the ball will reach radius of us within
		// 2 seconds
		// give a range of the start end of reachable
		std::pair<double, double> projectTimeToReach(double outerradius, double innerradius)
		{
			float start = -1, end = 2;
			for ( float interval = 0.0;  interval < 1.5;  interval += 0.05 )
			{
			    Vector4d stateProj = predict_coordinate( (double)interval );
			    Vector2d projection = Vector2d(stateProj.x(), stateProj.y());
			    if ( start == -1 && projection.norm() < outerradius )
		        {
		        	start = interval;
		        }
			    else if ( start != -1 &&  projection.norm() < innerradius )
		        {
		        	end = interval - 0.05;  
		        }
			}
			return std::make_pair(start, end);  
		}

		Vector2d getPos()
		{
			return coordinate;
		}

		Vector2d getVel()
		{
			return velocity;
		}
	};
