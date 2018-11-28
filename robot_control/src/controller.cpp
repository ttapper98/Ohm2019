#include <ros/ros.h>
#include <ros/console.h>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <isc_joy/xinput.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <ohm_igvc_msgs/Target.h>
#include <ohm_igvc_srvs/waypoint.h>
#include <serial/serial.h>

class robotController {
	public:
		robotController();
	private:
		
}
