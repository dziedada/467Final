#pragma once

// Ball Class
/*
 * Contains the dynamics of the Ball as well as a  
 * Standard Kalman filter to track it over time
 */

#include <common/point.hpp>
#include <common/messages/ball_detections_t.hpp>
#include <common/messages/ball_detection_t.hpp>

#include <opencv2/video/tracking.hpp>
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

using cv::Mat;

using Eigen::Vector2d;
using Eigen::Vector4d;

using std::cout;
using std::endl;

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
		Vector2d coordinatePrediction;	// ( x_1, y_2 )

		std::vector<double> coordLine; // [m, b]

		std::deque< Vector2d > coordHistory;
		std::deque<double> speedHistory;
		std::deque< Vector2d > velocityHistory;
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

			for ( int i = 0; i < HISTORY_SIZE; ++i )
			{
				speedHistory.push_back(vel.norm());
			}

			for ( int i = 0; i < HISTORY_SIZE; ++i )
			{
				velocityHistory.push_back( Vector2d( 0.0, 0.0 ) );
			}

			reachPrediction.ball_in_range_time_ = 1000;
			reachPrediction.ball_inrange_position_ = Vector2d( 0.0, 0.0 );
			reachPrediction.ball_inrange_velocity_ = Vector2d( 0.0, 0.0 );
			reachPrediction.goal_ = Vector2d( 0.0, 0.0 );
			reachPrediction.utime_ = 0;
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
			return ( pt.dot(lineV) / (lineV.dot(lineV)) ) * lineV;
		}

		void update(const ball_detection_t &detection)
		{
            odds += 1;
			double dT = (double)(detection.utime - utime) / (double)1000000;

			// update measurement for gui
			meas = coordinate;

            velocity = Vector2d( ( detection.position[0] - prevDetectionPos[0] ) / dT, ( detection.position[1] - prevDetectionPos[1] ) / dT );
            Vector2d detectionPos = Vector2d( detection.position[0], detection.position[1] );
            prevDetectionPos = detectionPos;

            velocityHistory.pop_front();
			velocityHistory.push_back( velocity );

			coordHistory.pop_front();
			coordHistory.push_back(detectionPos);

			// speedHistory.pop_front();
			// speedHistory.push_back((coordHistory.back() - coordHistory.back()-1).norm()/dT);
			// use a line of best fit to get the coordinate prediction
			coordLine = GetLinearFit(coordHistory);
			coordinate = projectPt(detectionPos, coordLine[0], coordLine[1]);

			utime = detection.utime;
            std::cout << "utime " << utime << " dT " << dT << std::endl;
		}
		
		void updatePrediction( const double outerRadius )
			{
			for ( double interval = 0.05; interval < 10.0; interval += 0.05 )
				{
				Vector4d prediction = predict_coordinate( interval );
				Vector2d position = Vector2d( prediction.x(), prediction.y() );
				std::cout << "distance = " << position.norm() << std::endl;
				if ( position.norm() < outerRadius )
					{
					Vector2d velocity = Vector2d( prediction[2], prediction[3] );
					reachPrediction.utime_ = std::chrono::duration_cast<std::chrono::microseconds>(
							std::chrono::system_clock::now().time_since_epoch()).count();
					reachPrediction.ball_in_range_time_ = reachPrediction.utime_ + 10e6*interval;
					cout << "reachPrediction time " << reachPrediction.ball_in_range_time_ << std::endl;
					reachPrediction.ball_inrange_position_ = position;
					reachPrediction.ball_inrange_velocity_ = velocity;
					reachPrediction.goal_ = position;
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
