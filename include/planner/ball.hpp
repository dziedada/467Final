// Ball Class
/*
 * Track ball coordinate, velocity, color
 */

#include <common/point.hpp>
#include <cmath>

class Ball
	{
	private:
		int id;
		int color;
		int64_t utime; // system utime
        // int64_t inputTime; // EKF system utime
		Point< double > coordinate;					// ( x, y )
		Point< double > velocity;					// ( v_x, v_y )
		Point< double > coordinate_prediction;	// ( x_1, y_2 )

        friend class ArmPlanner;
	public:
		Ball( int id_, int col, int64_t time, Point< double > coord, Point< double > vel = Point< double >( 0, 0 ) )
				: id( id_ ), color( col ), utime( time ), coordinate( coord ),
				  velocity( vel )
			{
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

		bool predict_coordinate( )
			{
			}
        
	};

// Other Helper Functions ( trig )
double calculateAngleRadians( Point < doube > &pt1, Point < double > &pt2 )
    {
    return asin( ( pt1.x - pt2.x ) / ( pt2.y - pt1.y ) );
    }
