// Arm Planner Header
/*
 * Contains belief of world representation
 * Receives ball coordinates, velocities, and color from Kalman Filter
 * Receives goal coordinates and dimensions from vision
 * Outputs arm orientations to python arm controller
 */
#include "ball.h"

class ArmPlanner
	{
	private:
		vector< Point > goals;
		vector< Ball > balls;

	public:
		ArmPlanner( )
			{
			}
	};
