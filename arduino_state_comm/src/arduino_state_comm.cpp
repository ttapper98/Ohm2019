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

#define float_NaN std::numeric_limits<float>::quiet_NaN

class ArduinoStateComm
{
    public:
        // Description: Constructor for the ArduinoStateComm object
        // Parameters: A ROS node handle
        ArduinoStateComm();

        // Description: Function disconnects the Arduino.
        void arduinoDisconnect();

        // Description: Function connects the Arduino.
        void arduinoConnect();

        // Description: Function sends a command to the Arduino.
        // Parameters: A string with the command to send
        void arduinoSendCommand(string command);

        // Description: Callback function for the robot state topic
        void robotStateReceived_callback(const ohm_igvc_msgs::RobotState& message);

        // Description: Callback function for the GPS status topic
        void gpsStatusReceived_callback(const vn300::Status& message);

        // Description: Function reads the from the Arduino
        void readArduino();

        // Description: Function publishes values read from the Arduino
        void publishArduinoInfo(int values[]);

        // Public members for the Arduino port name, serial port object, and listener
        std::string arduinoPortName;
		serial::Serial arduinoSerialPort;
        serial::utils::SerialListener serialListener;

    private:
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

    // Subscribe to topic. 1 = topic name, 2 = queue size, 3 = callback function, 4 = object to call function on
    robotStateSub = nodeHandle.subscribe("robotState", 1, &ArduinoStateComm::robotStateReceived_callback, this);
    gpsStatusSub = nodeHandle.subscribe("gpsStatus", 1, &ArduinoStateComm::gpsStatusReceived_callback, this);

    // Publish to topic.arduinoInfoPub
    arduinoKillStatePub = nodeHandle.advertise<std_msgs::String>("arduino_kill_state", 1);
	arduinoPauseStatePub = nodeHandle.advertise<std_msgs::String>("arduino_pause_state", 1);
    arduinoBatteryPub = nodeHandle.advertise<sensor_msgs::BatteryState>("arduino_batteries", 1);
} // END of ArduinoStateComm constructor

void ArduinoStateComm::arduinoDisconnect()
{
    if(serialListener.isListening())
    {
        serialListener.stopListening();
    } // END of if the serial listener is listening
} // END of arduinoDisconnect() function

void ArduinoStateComm::arduinoConnect()
{
    if(arduinoPortName.empty())
    {
        ROS_ERROR("Arduino serial port name is empty.");
        return;
	} // END of if the port name is empty

	arduinoDisconnect();

    // Create and configure new serial port
	Serial port(arduinoPortName);
	port->setBaudrate(9600);
	port->setBytesize(serial::eightbits);
	port->setParity(serial::parity_even);
	port->setTimeout(serial::Timeout::simpleTimeout(10));

    port->open();

    serialListener.setChunkSize(2);
    serialListener.startListening(port);
	arduinoSerialPort = port;
	ROS_INFO("Connected to Arduino.");
} // END of arduinoConnect() function

void ArduinoStateComm::arduinoSendCommand(string command)
{
	ROS_INFO("Sending Arduino command: %s", command.c_str());
	arduinoSerialPort->write(command+"\r\n");
} // END of arduinoSendCommand() function

void ArduinoStateComm::robotStateReceived_callback(const std_msgs::String::ConstPtr &message)
{
    if(message->data == "auto") { // Send robot state to Arduino
		arduinoSendCommand("a");
	}
	else if(message->data == "manual")
    {
        arduinoSendCommand("m");
    } // END of if robot state is auto or manual
} // END of robotStateReceived_callback() function

void ArduinoStateComm::gpsStatusReceived_callback(const vn300::Status::ConstPtr &message)
{
    // Send GPS status to Arduino
    switch(message->fix)
    {
        case 0:
            arduinoSendCommand("O");
            break;
        case 1:
            arduinoSendCommand("S");
            break;
            case 2:
        case 3:
            arduinoSendCommand("F");
            break;
        default:
            break;
    } // END of switch
} // END of gpsStatusReceived_callback() function

void ArduinoStateComm::readArduino()
{
    // Create buffer filter pointer with delimiter of \r\n
    // As of 2-26-19: Each token is a string of numbers separated by commas
    //      1st = kill, 2nd = pause, 3-8 = cell voltage, 9 = temp
    BufferedFilterPtr bufferFilter = serialListener.createBufferedFilter(SerialListener::delimeter_tokenizer("\r\n"));

    double convertedValues[9];

    string inputLine = bufferFilter->wait(50);

	std::vector<std::string> data;

	boost::split(data, inputLine, boost::is_any_of(","));

	if(!data.is_empty()) {
		bool killState = boost::lexical_cast<bool>(data[0]);
		bool pauseState = boost::lexical_cast<bool>(data[1]);

		std::vector<float> cellValues;
		for(auto cell = data.begin() + 2; cell != data.end() - 1; ++cell) {
			cellValues.push_back(boost::lexical_cast<double>(*cell))
		}

		// double temp = boost::lexical_cast<double>(data.back());

		publishArduinoInfo(killState, pauseState, cellValues, temp);
	} else {
		ROS_WARNING("Did not get any info from the Arduino!");
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
    ArduinoStateComm arduinoObj();

    // Get the serial port name from parameters or use default
    nh.param("arduino_serial_port", arduinoObj.arduinoPortName, std::string("/dev/ttyACM0"));

    arduinoObj.arduinoConnect();
    arduinoObj.arduinoSendCommand("Start");

    ros::Rate rate(10);
    while(ros::ok())
    {
        arduinoObj.readArduino();
        ros::spin();
		rate.sleep();
    } // END of while ros ok
} // END of main() function
