#include <ros/ros.h>
#include <ros/console.h>
#include <stdlib.h>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <isc_joy/xinput.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <ohm_igvc_msgs/Target.h>
#include <ohm_igvc_srvs/waypoint.h>
#include <serial/serial.h>

class robotController 
{
	public:

		enum state
		{
			AUTOMATIC, 
			LOW_VOLTAGE, 
			KILL, 
			PAUSE, 
			MANUAL
		};

		robotController();
		void update_state();	

	private:
		bool automatic;	//sets up booleans to be input into switching case
		bool low_voltage;
		bool kill;
		bool pause;
		bool manual;

		STATE robostate; 
		STATE prev_state;
		//make another variable for state before switching case of type state
		//check if statements possibly else if
		//signals make it second type of state
}


robotController::update_state()
{
	
	switch(robostate)
	{
		case AUTOMATIC:
			if (manual)
			{
				robostate = MANUAL;
			}
			
			else if (kill)
			{
				robostate = KILL;
			}
			
			else if (low_voltage)
			{
				robostate = LOW_VOLTAGE;
			}
			
			else if (pause)
			{
				robostate = PAUSE;
				prev_state = AUTOMATIC;
			}
		break;
		
		case LOW_VOLTAGE:
			if (!low_voltage)
			{
				robostate = MANUAL;
			}
		break;

		case KILL:
			if (!kill)
			{
				robostate = MANUAL;
			}
			
			else if (!kill && pause)
			{
				robostate = PAUSE;
				prev_state = MANUAL;
			}
			
			else if (low_voltage)
			{
				robostate = LOW_VOLTAGE;
			}
		break;

		case PAUSE:
			if (!pause)
			{
				robostate = prev_state;
			}

			else if (kill)
			{
				robostate = KILL;
			}

			else if (low_voltage)
			{
				robostate = LOW_VOLTAGE;
			}
		break;
		
		case MANUAL:
			if (automatic)
			{
				robostate = AUTOMATIC;
			}
			else if (kill)
			{
				robostate = KILL;
			}
			else if (low_voltage)
			{
				robostate = LOW_VOLTAGE;
			}
			else if (pause)
			{
				robostate = PAUSE;
				prev_state = MANUAL;
			}
		break;
		
		default:
			robostate = MANUAL;
	}

	
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "controller");
	ros::NodeHandle nh;
	
	ros::Rate rate(2); //determine a rate that works for this ask matt!!!

	/*while(ros::ok())
	{
		robotController::robotController(state);

		rate.sleep();
	}*/
}
