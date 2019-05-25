#include <ros/ros.h>
#include <limits>
#include <serial/serial.h>
#include <serial/utils/serial_listener.h>
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <vector>

#include <sensor_msgs/BatteryState.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <vn300/Status.h>

#define float_NaN std::numeric_limits<float>::quiet_NaN()

class ArduinoStateComm
{
    public:
        // Description: Constructor for the ArduinoStateComm object
        // Parameters: A ROS node handle
        ArduinoStateComm();

		bool isConnected();

		std::string getPortName();

        // Description: Function disconnects the Arduino.
        void disconnect();

        // Description: Function connects the Arduino.
        void connect();

        // Description: Function sends a command to the Arduino.
        // Parameters: A string with the command to send
        void sendCommand(std::string command);

        // Description: Callback function for the robot state topic
        void robotStateReceivedCallback(const std_msgs::String::ConstPtr &message);

        // Description: Callback function for the GPS status topic
        void gpsStatusReceivedCallback(const vn300::Status::ConstPtr &message);

        // Description: Function reads the from the Arduino
        void readArduino(std::string token);

        // Description: Function publishes values read from the Arduino
        void publishArduinoInfo(bool kill, bool pause, std::vector<float> cellVoltages, float voltage);
  
    private:
		bool connected;
		// Private members for the Arduino port name, serial port object, and listener
		std::string arduinoPortName;
		serial::Serial arduinoSerialPort;
        serial::utils::SerialListener serialListener;

        // Private members for the subscribers for the robot state and GPS
        ros::Subscriber robotStateSub;
        ros::Subscriber gpsStatusSub;

        // Private member for the publisher for kill state
        ros::Publisher arduinoKillStatePub;
		// Private member for the publisher for pause state
		ros::Publisher arduinoPauseStatePub;
        // Private member for the publisher for the batteries
        ros::Publisher arduinoBatteryPub;  
}; // END of class Arduino state communication

ArduinoStateComm::ArduinoStateComm()
{
	ros::NodeHandle nodeHandle;
	ros::NodeHandle nh_private("~");

	nh_private.param("port", arduinoPortName, std::string("/dev/ttyACM0"));

    // Subscribe to topic. 1 = topic name, 2 = queue size, 3 = callback function, 4 = object to call function on
    robotStateSub = nodeHandle.subscribe("robotState", 1, &ArduinoStateComm::robotStateReceivedCallback, this);
    gpsStatusSub = nodeHandle.subscribe("gpsStatus", 1, &ArduinoStateComm::gpsStatusReceivedCallback, this);

    // Publish to topic.arduinoInfoPub
    arduinoKillStatePub = nodeHandle.advertise<std_msgs::String>("arduino_kill_state", 1);
	arduinoPauseStatePub = nodeHandle.advertise<std_msgs::String>("arduino_pause_state", 1);
    arduinoBatteryPub = nodeHandle.advertise<sensor_msgs::BatteryState>("arduino_batteries", 1);

	connected = false;
} // END of ArduinoStateComm constructor

bool ArduinoStateComm::isConnected() 
{
	return connected;
}

std::string ArduinoStateComm::getPortName()
{
	return arduinoPortName;
}

void ArduinoStateComm::disconnect()
{
    if(serialListener.isListening())
    {
        serialListener.stopListening();
    } // END of if the serial listener is listening

} // END of disconnect() function

void ArduinoStateComm::connect()
{
    if(arduinoPortName.empty())
    {
        ROS_ERROR("Arduino serial port name is empty.");
        return;
	} // END of if the port name is empty

	
	if(connected) disconnect();

	serial::Timeout timeout = serial::Timeout::simpleTimeout(10);

    // Create and configure new serial port
	arduinoSerialPort.setPort(arduinoPortName);
	arduinoSerialPort.setBaudrate(9600);
	arduinoSerialPort.setBytesize(serial::eightbits);
	arduinoSerialPort.setParity(serial::parity_even);
	arduinoSerialPort.setTimeout(timeout);

    arduinoSerialPort.open();

    serialListener.setChunkSize(64); // this is the number of bytes it reads at a time
	serialListener.setTokenizer(serial::utils::SerialListener::delimeter_tokenizer("\r\n"));
	serialListener.setDefaultHandler(boost::bind(&ArduinoStateComm::readArduino, this, _1));
	serialListener.startListening(arduinoSerialPort);

	ROS_INFO("Connected to Arduino.");

	connected = true;
} // END of connect() function

void ArduinoStateComm::sendCommand(std::string command)
{
	ROS_INFO("Sending Arduino command: %s", command.c_str());
	arduinoSerialPort.write(command+"\r\n");
} // END of sendCommand() function

void ArduinoStateComm::robotStateReceivedCallback(const std_msgs::String::ConstPtr &message)
{
    if(message->data == "auto") { // Send robot state to Arduino
		sendCommand("a");
	}
	else if(message->data == "manual")
    {
        sendCommand("m");
    } // END of if robot state is auto or manual
} // END of robotStateReceivedCallback() function

void ArduinoStateComm::gpsStatusReceivedCallback(const vn300::Status::ConstPtr &message)
{
    // Send GPS status to Arduino
    switch(message->fix)
    {
        case 0:
            sendCommand("O");
            break;
        case 1:
            sendCommand("S");
            break;
            case 2:
        case 3:
            sendCommand("F");
            break;
        default:
            break;
    } // END of switch
} // END of gpsStatusReceivedCallback() function

void ArduinoStateComm::readArduino(std::string token)
{
    // As of 2-26-19: Each token is a string of numbers separated by commas
    //      1st = kill, 2nd = pause, 3-8 = cell voltage, 9 = temp
	std::vector<std::string> data;
	boost::split(data, token, boost::is_any_of(","));

	if(!data.empty()) 
  {
		bool killState = boost::lexical_cast<bool>(data[0]);
		bool pauseState = boost::lexical_cast<bool>(data[1]);

		std::vector<float> cellValues;
    
		double total_voltage = 0.0;
		for(auto cell = data.begin() + 2; cell != data.end() - 1; ++cell) {
			cellValues.push_back(boost::lexical_cast<double>(*cell) * (5.0 / 1024.0));
			total_voltage += cellValues.back();
		}

		// double temp = boost::lexical_cast<double>(data.back());
		publishArduinoInfo(killState, pauseState, cellValues, total_voltage);
	} else {
		ROS_WARN("Did not get any info from the Arduino!");
	}
} // END of readArduino() function

void ArduinoStateComm::publishArduinoInfo(bool kill, bool pause, std::vector<float> cellVoltages, float voltage)
{
	// set the state machine signals
	std_msgs::String killSignal;
	std_msgs::String pauseSignal;

	killSignal.data = kill;
	pauseSignal.data = pause;

	// set the battery state message
	sensor_msgs::BatteryState batteryState;

	batteryState.power_supply_status = batteryState.POWER_SUPPLY_STATUS_DISCHARGING;
	batteryState.power_supply_health = batteryState.POWER_SUPPLY_HEALTH_GOOD;
	batteryState.power_supply_technology = batteryState.POWER_SUPPLY_TECHNOLOGY_LIPO;
	batteryState.voltage = voltage;
	batteryState.cell_voltage = cellVoltages;

	batteryState.current = float_NaN;
	batteryState.capacity = float_NaN;
	batteryState.charge = float_NaN;
	batteryState.design_capacity = float_NaN;
	batteryState.percentage = float_NaN;
	batteryState.present = true;

	// publish
    arduinoKillStatePub.publish(killSignal);
	arduinoPauseStatePub.publish(pauseSignal);
    arduinoBatteryPub.publish(batteryState);
} // END of publishArduinoInfo() function

// Main function
int main(int argc, char **argv)
{
    ros::init(argc, argv, "arduino_state_comm");
    ros::NodeHandle nh("~");

    // Create object used to communicate states with the Arduino.
    ArduinoStateComm arduinoObj;

	while(!arduinoObj.isConnected())
	{
		ROS_INFO("Connecting to arduino on %s", arduinoObj.getPortName().c_str());

		try 
		{
    		arduinoObj.connect();
    		arduinoObj.sendCommand("Start");
		} catch(std::exception &e)
		{
			ROS_ERROR("Failed to connect to the Arduino: %s", e.what());
		}
		
		if(!ros::ok()) break;
	}

	if(ros::ok() && arduinoObj.isConnected()) {	
		ros::spin();
	}

	arduinoObj.disconnect();

} // END of main() function
