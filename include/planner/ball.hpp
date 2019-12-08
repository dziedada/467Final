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

#include <cmath>
#include <iostream>
#include <vector>
#include <utility>

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
		Vector2d coordinate_prediction;	// ( x_1, y_2 )
		cv::KalmanFilter kf;
		cv::Mat state; // [x, y, v_x, v_y]
		cv::Mat meas;  // [x, y]
		std::vector<double> kf_error_history;

        friend class ArmPlanner;
		friend class TrackingVis;

	public:
		Ball( int id_, int col, int64_t time, Vector2d coord, Vector2d vel = Vector2d(0, 0) )
				: id( id_ ), color( col ), utime( time ), odds( 0 ),
                coordinate( coord ), velocity( vel )
		{
			// initialize a kalman filter
			int stateSize = 4; // [x, y, v_x, v_y]
			int measSize = 2;  // [x, y]
			int contrSize = 0; // no control input
			unsigned int type = CV_64F; // double type

			kf = cv::KalmanFilter(stateSize, measSize, contrSize, type);

			state = Mat(stateSize, 1, type);  // [x,y,v_x,v_y]
    		meas = Mat(measSize, 1, type);    // [z_x,z_y]

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

		    // Process noise covariance matrix Q
		    // TODO: Tune

		    kf.processNoiseCov.at<double>(0) = 1e-5;
		    kf.processNoiseCov.at<double>(5) = 1e-5;
		    kf.processNoiseCov.at<double>(10) = 1e-5;
		    kf.processNoiseCov.at<double>(15) = 1e-5;
		    // cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));

		    // Measures Noise Covariance Matrix R
    		cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-3));


    		// First detection!
            // >>>> Initialization of Prior
            std::cout << "First detection: " << coord << std::endl;
            kf.errorCovPre.at<double>(0) = 1e-5; // 1 cm
            kf.errorCovPre.at<double>(5) = 1e-5; // 1 cm
            kf.errorCovPre.at<double>(10) = 1e-5;
            kf.errorCovPre.at<double>(15) = 1e-5;

            state.at<double>(0) = coord.x();
            state.at<double>(1) = coord.y();
            state.at<double>(2) = vel.x();
            state.at<double>(3) = vel.y();
            // <<<< Initialization

            kf.statePost = state;    
		}

		void update(const ball_detection_t &detection)
		{
            odds += 1;
            std::cout<< "detection time: " << detection.utime << std::endl;
            std::cout<< "prev time: " << utime << std::endl;

			double dT = (double)(detection.utime - utime) / (double)1000000;
			utime = detection.utime;

			// update Matrix A for correct velocity estimate 
			kf.transitionMatrix.at<double>(2) = dT;
            kf.transitionMatrix.at<double>(7) = dT;

            // for error checking
            Mat prediction = kf.predict();
            Vector2d predictPt(prediction.at<double>(0),prediction.at<double>(1));

            // state estimate
            meas.at<double>(0) = detection.position[0];
            meas.at<double>(1) = detection.position[1];

            //state = kf.predict();
            state = kf.correct(meas);
            Vector2d estimatePt(state.at<double>(0),state.at<double>(1));
            Vector2d estimateVel(state.at<double>(2),state.at<double>(3));
            std:: cout << "meas: " << detection.position[0] << " " << detection.position[1] << std::endl;
            std::cout << "predictPos:  " << predictPt << std::endl;
            std::cout << "updatepos: " << estimatePt << std::endl;
            std::cout << "updateVel: " << estimateVel << std::endl;
            std::cout << "DT: " << dT << std::endl;
            kf_error_history.push_back( (estimatePt - predictPt).norm() );
            std::cout << "error: " << (estimatePt - predictPt).norm() << std::endl;
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
	        this->coordinate_prediction = other.coordinate_prediction;
        }

        Vector4d predict_coordinate(int64_t time)
        {
        	double dT = (double)(time - utime) / (double)1000000;
        	return predict_coordinate(dT);
        }

        // predict out the ball in dt seconds
        // TODO refactor to predict state
		Vector4d predict_coordinate( double dT)
		{
			// update Matrix A for correct velocity estimate 
			kf.transitionMatrix.at<double>(2) = dT;
            kf.transitionMatrix.at<double>(7) = dT;
            auto predState = kf.predict();
            //cout << "dx " << predState.at<double>(2) << " dy " << predState.at<double>(3) << endl;
            return Vector4d(predState.at<double>(0), predState.at<double>(1), 
            			    predState.at<double>(2), predState.at<double>(3));
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
