// Ball Class
/*
 * Track ball coordinate, velocity, color
 */

#include "point.h"

class Ball
	{
	private:
		int id;
		int color;
		int64_t utime;
		Point coordinate;					// ( x, y )
		Point velocity;					// ( v_x, v_y )
		Point coordinate_prediction;	// ( x_1, y_2 )
	public:
		Ball( int id_, int col, int64_t time, Point coord, Point vel = Point( 0, 0 ) )
				: id( id_ ), color( col ), utime( time ), coordinate( coord ),
				  velocity( vel ), color( col )
			{
			}
		bool predict_coordinate( )
			{
			}
	};
