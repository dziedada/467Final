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
		Vector2d coordinate;					// ( x, y )
		Vector2d velocity;					// ( v_x, v_y )
		Vector2d coordinatePrediction;	// ( x_1, y_2 )
		cv::KalmanFilter kf;
		cv::Mat state; // [x, y, v_x, v_y]
		cv::Mat meas;  // [x, y]
		std::vector<double> kf_error_history;
		std::list< Vector2d > velocityHistory;//( 5, Vector2d( 0.0, 0.0 ), std::deque<Vector2d>);
		const int HISTORY_SIZE = 10;
		Prediction reachPrediction;
        friend class ArmPlanner;
		friend class TrackingVis;

	public:
		Ball( int id_, int col, int64_t time, Vector2d coord, Vector2d vel = Vector2d(0, 0) )
				: id( id_ ), color( col ), utime( time ), odds( 0 ),
                coordinate( coord ), velocity( vel )
		{
			// initialize a kalman filter
			int stateSize = 4; // [x, y, v_x, v_y]
			int measSize = 4;  // [x, y]
			int contrSize = 0; // no control input
			unsigned int type = CV_64F; // double type

			kf = cv::KalmanFilter(stateSize, measSize, contrSize, type);

			state = Mat(stateSize, 1, type);  // [x,y,v_x,v_y]
    		meas = Mat(measSize, 1, type);    // [z_x,z_y,z_vx,z_vy]

			// Transition matrix
		    // Note: set dT at each processing step!
		    // [ 1 0 dT 0  ]
		    // [ 0 1 0  dT ]
		    // [ 0 0 1  0  ]
		    // [ 0 0 0  1  ]
			cv::setIdentity(kf.transitionMatrix);

			// Measure Matrix H
		    // [ 1 0 0 0]
		    // [ 0 1 0 0]
		    kf.measurementMatrix = Mat::zeros(measSize, stateSize, type);
		    kf.measurementMatrix.at<double>(0) = 1.0;
		    kf.measurementMatrix.at<double>(5) = 1.0;
		    kf.measurementMatrix.at<double>(10) = 1.0;
		    kf.measurementMatrix.at<double>(15) = 1.0;

		    // Process noise covariance matrix Q
		    // TODO: Tune

		    /*kf.processNoiseCov.at<double>(0) = 1e-2;
		    kf.processNoiseCov.at<double>(5) = 1e-2;
		    kf.processNoiseCov.at<double>(10) = 1e-5;
		    kf.processNoiseCov.at<double>(15) = 1e-5;*/
		    cv::setIdentity(kf.processNoiseCov, cv::Scalar(1));
		    kf.processNoiseCov.at<double>(10) = 1;
		    kf.processNoiseCov.at<double>(15) = 1;

		    // Measures Noise Covariance Matrix R
    		cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1));
		    //kf.processNoiseCov.at<double>(10) = 1;
		    //kf.processNoiseCov.at<double>(15) = 1;

    		// First detection!
            // >>>> Initialization of Prior
            std::cout << "First detection: " << coord << std::endl;
    		cv::setIdentity(kf.errorCovPre, cv::Scalar(1e-2));
            /*kf.errorCovPre.at<double>(0) = 1e-2; // 1 cm
            kf.errorCovPre.at<double>(5) = 1e-2; // 1 cm*/
            kf.errorCovPre.at<double>(10) = 1e-1;
            kf.errorCovPre.at<double>(15) = 1e-1;

            state.at<double>(0) = coord.x();
            state.at<double>(1) = coord.y();
            state.at<double>(2) = vel.x();
            state.at<double>(3) = vel.y();
            // <<<< Initialization

            kf.statePost = state;    
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

		void update(const ball_detection_t &detection)
		{
            odds += 1;
			double dT = (double)(detection.utime - utime) / (double)1000000;

            velocity = Vector2d( ( detection.position[0] - coordinate[0] ) / dT, ( detection.position[1] - coordinate[1] ) / dT );
            coordinate = Vector2d( detection.position[0], detection.position[1] );
			
			velocityHistory.pop_front();
			velocityHistory.push_back( velocity );

			utime = detection.utime;
            std::cout << "utime " << utime << " dT " << dT << std::endl;

			// update Matrix A for correct velocity estimate 
			kf.transitionMatrix.at<double>(2) = dT;
            kf.transitionMatrix.at<double>(7) = dT;

            // for error checking
            Mat prediction = kf.predict();
            coordinatePrediction = Vector2d(prediction.at<double>(0),prediction.at<double>(1));

            // state estimate
            meas.at<double>(0) = coordinate.x();
            meas.at<double>(1) = coordinate.y();
            meas.at<double>(2) = velocity.x();
            meas.at<double>(3) = velocity.y();

            //state = kf.predict();
            state = kf.correct(meas);
            Vector2d estimatePt(state.at<double>(0),state.at<double>(1));
            Vector2d estimateVel(state.at<double>(2),state.at<double>(3));
            /*std::cout << "ball: " << coordinate.x() << ", " << coordinate.y() << std::endl;
            std::cout << "meas: " << detection.position[0] << " " << detection.position[1] << std::endl;
            std::cout << "predictPos:  " << coordinatePrediction << std::endl;
            std::cout << "updatepos: " << estimatePt << std::endl;
            std::cout << "updateVel: " << estimateVel << std::endl;
            std::cout << "DT: " << dT << std::endl;*/
            kf_error_history.push_back( (estimatePt - coordinatePrediction).norm() );
            //std::cout << "error: " << (estimatePt - coordinatePrediction).norm() << std::endl;
		}
		
		void updatePrediction( const double outerRadius )
			{
			for ( double interval = 0.05; interval < 10; interval += 0.05 )
				{
				Vector4d prediction = predict_coordinate( interval );
				Vector2d position = Vector2d( prediction.x(), prediction.y() );
				if ( position.norm() < outerRadius )
					{
					Vector2d velocity = Vector2d( prediction[2], prediction[3] );
					reachPrediction.utime_ = std::chrono::duration_cast<std::chrono::microseconds>(
							std::chrono::system_clock::now().time_since_epoch()).count();
					reachPrediction.ball_in_range_time_ = reachPrediction.utime_ + 10e6*interval;
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

        Ball operator=( const Ball &other )
        {
	        this->id = other.id;
	        this->color = other.color;
	        this->utime = other.utime;
	        this->coordinate = other.coordinate;
	        this->velocity = other.velocity;
	        this->coordinatePrediction = other.coordinatePrediction;
        }

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
			// EKF Estimation
			// update Matrix A for correct velocity estimate 
			kf.transitionMatrix.at<double>(2) = dT;
            kf.transitionMatrix.at<double>(7) = dT;
            auto predState = kf.predict();
			
			// Point History Estimation ( using last 5 points )
			Vector2d velocityEstimate( 0.0, 0.0 );
			for ( auto &it: velocityHistory )
				{
				velocityEstimate.x() += it.x();
				velocityEstimate.y() += it.y();
				//std::cout << "vel est: " << velocityEstimate << std::endl;
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
			return Vector2d(state.at<double>(0), state.at<double>(1));
		}

		Vector2d getVel()
		{
			return Vector2d(state.at<double>(2), state.at<double>(3));
		}
	};
