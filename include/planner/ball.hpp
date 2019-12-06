// Ball Class
/*
 * Contains the dynamics of the Ball as well as a  
 * Standard Kalman filter to track it over time
 */

#include <common/point.hpp>
#include <cmath>
#include <opencv2/video/tracking.hpp>
#include <Eigen/Core>
#include <vector>

using Eigen::Vector2d;
using Eigen::Vector4d;
using cv::Mat;

class Ball
	{
	private:
		int id;
		int color;
		int64_t utime; // previous utime
        // int64_t inputTime; // EKF system utime
		Vector2d coordinate;					// ( x, y )
		Vector2d velocity;					// ( v_x, v_y )
		Vector2d coordinate_prediction;	// ( x_1, y_2 )
		cv::KalmanFilter kf;
		cv::Mat state; // [x, y, v_x, v_y]
		cv::Mat meas;  // [x, y]
		std::vector<double> kf_error_history;

        friend class ArmPlanner;
	public:
		Ball( int id_, int col, int64_t time, Vector2d coord, Vector2d vel = Vector2d(0, 0) )
				: id( id_ ), color( col ), utime( time ), coordinate( coord ),
				  velocity( vel )
		{
			// initialize a kalman filter
			int stateSize = 4; // [x, y, v_x, v_y]
			int measSize = 2;  // [x, y]
			int contrSize = 0; // no control input
			unsigned int type = CV_64F; // double type

			kf = cv::KalmanFilter(stateSize, measSize, contrSize, type)

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
		    kf.measurementMatrix.at<double>(0) = 1.0f;
		    kf.measurementMatrix.at<double>(5) = 1.0f;

		    // Process noise covariance matrix Q
		    // TODO: Tune
		    cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));

		    // Measures Noise Covariance Matrix R
    		cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
		}

		void update(const ball_detection_t &detection)
		{
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

            state = kf.correct(meas);
            Vector2d estimatePt(state.at<double>(0),state.at<double>(1));

            kf_error_history.push_back( (estimatePt - predictPt).norm() );
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

        Vector4d predict_coordinate(double utime)
        {
        	double dT = (double)(detection.utime - utime) / (double)1000000;
        	return predict_coordinate(dT);
        }

        // predict out the ball in dt seconds
		Vector4d predict_coordinate( double dT)
		{
			// update Matrix A for correct velocity estimate 
			kf.transitionMatrix.at<double>(2) = dT;
            kf.transitionMatrix.at<double>(7) = dT;
            return kf.predict();
		}


		Vector2d getPos()
		{
			return Vector2d(state[0], state[1]);
		}
	};

// Other Helper Functions ( trig )
double calculateAngleRadians( Point < doube > &pt1, Point < double > &pt2 )
    {
    return asin( ( pt1.x - pt2.x ) / ( pt2.y - pt1.y ) );
    }
