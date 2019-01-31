#include <iostream>
#include <sstream>
#include <math.h>
#include <HD\hd.h>
#include <HDU\hduVector.h>
#include <HDU\hduMatrix.h>
#include "stdafx.h"
#include "ros.h"
#include <string>
#include <stdio.h>
#include <stdint.h>
#include <bitset>
#include <windows.h>
#include <map>
#include <vector>
#include <algorithm>
#include <numeric>
#include <iterator>
#include <functional>
#include <memory>
#include <signal.h>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <iterator>
#include "Windows.h"
#include <mutex>

// ROS messages
#include "phantom_omni/OmniState.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"


using namespace std;
using namespace ros;
using std::string;

int launches = 0;

void printline()
{
	std::cout << "***********************************************************************\n";
	return;
}

long long startTime_;

// this should really be a class with functions moved into it (e.g. computeDampingForce)
typedef struct
{
	float	   transform[16];
	bool	   button1;
	bool	   button2;
	float	   velocity[3];
	vector<vector<float>>  velocity_buffer; // 3xN
	float	   forceScaling; // damping force in Newtons per m/s
	float	   force[3];
	int		   updateRate;
	int		   updatesSinceLastPublish;
} OmniState;
const int velocity_buffer_length = 64; // number of past velocity readings to average for haptic force calculation
OmniState omniState_;
OmniState omniStateSnapshot_;

HDSchedulerHandle hapticCallbackHandle;

HHD hHD;
	
// Computes a damping force using the Karnopp model (viscous damping, Coulomb friction, and static friction)
bool computeDampingForce(OmniState *omniState){
	// fills omniState.force[3] array

	double vThreshold = 0.005; // [m/s] velocity threshold
	//double fs = 0.0;		  // [N] static friction force
	//double fc = 0.1;		  // [N] Coulomb friction
	double fc = 0.0;		  // [N] Coulomb friction

	double bv = omniState->forceScaling; // [N per m/s] viscous friction
	double dT = 1.0 / omniState->updateRate; // [s] time between force updates
	//double deltaFmax = 5.0;   // [N/s] max change in force per second
	//double dFmax = deltaFmax * dT; // [N] max change in force

	double vMag = sqrt(pow(omniState->velocity[0], 2) + pow(omniState->velocity[1], 2) + pow(omniState->velocity[2], 2)) / 1000.0;

	for (int ii = 0; ii < 3; ii++){
		double v = omniState->velocity[ii] / 1000.0; // convert from mm/s to m/s
		double Fcv = -(bv * v + copysign(fc, v)); // compute sum of Coulomb and viscous forces
		double F = 0.0;

		// check if vThreshold is exceeded
		if (abs(vMag) > vThreshold){
			F = Fcv;
		}
		else{
			// use whichever force is greater
			//F = (abs(Fcv) > fs) ? Fcv : copysign(fs, Fcv);
			
			F = 0.0;
		}

		double dF = F - omniState->force[ii];

		//// ensure dFmax is not exceeded
		//if (abs(dF) > dFmax) {
		//	dF = copysign(dFmax, dF);
		//}

		// compute final force
		omniState->force[ii] += dF;
	}

	return true;
}

HDCallbackCode HDCALLBACK hapticDampingCallback(void *data){

	omniState_.updatesSinceLastPublish++;

	// start frame
	hdBeginFrame(hHD);

	// get update rate
	hdGetIntegerv(HD_INSTANTANEOUS_UPDATE_RATE, &omniState_.updateRate);
	//std::cout << omniState_.updateRate << " Hz" << std::endl;

	// get current button states
	int nButtons = 0;
	hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
	omniState_.button1 = nButtons & HD_DEVICE_BUTTON_1;
	omniState_.button2 = nButtons & HD_DEVICE_BUTTON_2;

	// get current pose
	hdGetFloatv(HD_CURRENT_TRANSFORM, omniState_.transform);

	// set damping force
	bool forceEnabled = hdIsEnabled(HD_FORCE_OUTPUT);
	if (omniState_.forceScaling > 0.0){
		// enable force if not already
		if (!forceEnabled){
			hdEnable(HD_FORCE_OUTPUT);
		}

		// get current velocity
		float velocity_temp[3];
		hdGetFloatv(HD_CURRENT_VELOCITY, velocity_temp);

		// add current velocity to buffer and compute mean
		if (omniState_.velocity_buffer.size() != 3){ omniState_.velocity_buffer.resize(3); } // ensure size is 3
		for (int ii = 0; ii<omniState_.velocity_buffer.size(); ii++){
			omniState_.velocity_buffer[ii].insert(omniState_.velocity_buffer[ii].begin(), velocity_temp[ii]); // prepend x/y/z component
			omniState_.velocity_buffer[ii].resize(velocity_buffer_length); // remove any old values from the end
			omniState_.velocity[ii] = accumulate(omniState_.velocity_buffer[ii].begin(), omniState_.velocity_buffer[ii].end(), 0.0) / double(velocity_buffer_length); // compute mean
		}

		// compute and set damping force
		computeDampingForce(&omniState_);
		hdSetFloatv(HD_CURRENT_FORCE, omniState_.force);
	}
	else{
		// disable force
		if (forceEnabled){
			HDfloat force[3] = { 0.0, 0.0, 0.0 };
			hdSetFloatv(HD_CURRENT_FORCE, force);
			hdDisable(HD_FORCE_OUTPUT);
		}
	}

	hdEndFrame(hHD);
	// end frame

	return HD_CALLBACK_CONTINUE;
	//return HD_CALLBACK_DONE;
}

void force_callback(const std_msgs::Float32 & scaling)
{
	if (scaling.data > 0.0){
		omniState_.forceScaling = scaling.data;
	}
	else{
		omniState_.forceScaling = 0.0;
	}
}

void init_haptic_device()
{
	hHD = hdInitDevice(HD_DEFAULT_DEVICE);
	HDErrorInfo error;
	error = hdGetError();

	if (HD_DEVICE_ERROR(error))  // checking the device status
	{
		string sss = hdGetErrorString(error.errorCode);
		printline();
		std::cout << "Error in device initialization. \n\n";
		std::cout << sss << "\n\n";
	}
	else // connection management with rosserial server
	{
		printline();
		if (launches < 1) { std::cout << "Device Successfully Connected and Initialized. \nPhantom Omni should now be reading and writing ROS messages.\n\n"; }
		else { std::cout << "Device will now restart...\n\n" << "Restart Number " << launches << " completed sucessfully. \nPhantom Omni should now be reading and writing ROS messages.\n\n"; }
		launches++;
	}
	hdSetSchedulerRate(1600);
	hapticCallbackHandle = hdScheduleAsynchronous(hapticDampingCallback, (void*)0, HD_DEFAULT_SCHEDULER_PRIORITY);
	hdStartScheduler();
}

//HDCallbackCode HDCALLBACK DeviceStateCallback(void *pUserdata)
//{
//	std::cout << "s " << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count() - startTime_ << std::endl;
//
//	//try
//	//{
//		// start frame
//		hdBeginFrame(hHD);
//
//		// get current button states
//		int nButtons = 0;
//		hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
//		omniState_.button1 = nButtons & HD_DEVICE_BUTTON_1;
//		omniState_.button2 = nButtons & HD_DEVICE_BUTTON_2;
//
//		// get current pose
//		hdGetFloatv(HD_CURRENT_TRANSFORM, omniState_.transform);
//		
//		hdEndFrame(hHD);
//		// end frame
//	
//		HDErrorInfo error;
//		error = hdGetError();
//
//		if (HD_DEVICE_ERROR(error))  // checking the device status
//		{
//			printline();
//			std::cout << "An exception has occurred in communication with the Omni Controller. \nAn error from the controller API follows...\n\n";
//			std::string sss = hdGetErrorString(error.errorCode);
//			std::cout << sss << "\n\n";
//		//	throw 1;
//		}
//	//}
//	//catch (int param)
//	//{
//	//	init_haptic_device();
//	//}
//
//	return HD_CALLBACK_DONE;
//}

int _tmain(int argc, _TCHAR * argv[])
{
	printline();
	printline();
	cout << "Phantom Omni ROS Node" << endl;
	printline();
	printline();

	cout << endl << "Enter the desired ROS namespace to use:" << endl << "/Omni";
	string omni_id;
	getline(cin, omni_id);
	cout << endl;
	string rosNamespace = "Omni" + omni_id;
	//string rosNamespace = "Omni2";


	// timer
	double publishRate = 200.0; // [hz] rate to publish omni state
	long publishInterval = (long)(1000.0 / publishRate); // [ms]
	long long lastPublishTime = 0;
	startTime_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();

	// ROS node initialization
	NodeHandle nh;
	char *ros_master = "192.168.0.1";
	printline();
	printf("Connecting to server at %s\n\n", ros_master);
	nh.initNode(ros_master);

	// Setting up publishers
	phantom_omni::OmniState state_msg;
	string topic_state = rosNamespace + "/state";
	Publisher state_pub(topic_state.c_str(), &state_msg);
	nh.advertise(state_pub); 

	// Setting up subscriber
	string topic_force = rosNamespace + "/force_scaling";
	ros::Subscriber<std_msgs::Float32> omni_force_sub(topic_force.c_str(), &force_callback);
	nh.subscribe(omni_force_sub);

	nh.spinOnce();

	init_haptic_device();

	while (1){
		if (nh.connected()){
			// update omni state
			omniStateSnapshot_ = omniState_;
			if (omniStateSnapshot_.updatesSinceLastPublish > 10){
				omniState_.updatesSinceLastPublish = 0;

				//double v = sqrt(pow(omniStateSnapshot_.velocity[0],2) + pow(omniStateSnapshot_.velocity[1],2) + pow(omniStateSnapshot_.velocity[2],2));
				//std::cout << "V = " << v << " F = " << omniStateSnapshot_.force[0] << ", " << omniStateSnapshot_.force[1] << ", " << omniStateSnapshot_.force[2] << "; " << omniStateSnapshot_.updateRate << " Hz" << std::endl;

				// assemble OmniState message
				std::copy(std::begin(omniStateSnapshot_.transform), std::end(omniStateSnapshot_.transform), std::begin(state_msg.transform));

				//for (int ii = 0; ii < 16; ii++){
				//	state_msg.transform[ii] = omniStateSnapshot_.transform[ii];
				//}
				state_msg.button1 = omniStateSnapshot_.button1;
				state_msg.button2 = omniStateSnapshot_.button2;

				// Publish to the ROS network
				state_pub.publish(&state_msg);
			}
		}
		else{
			printf("Connection lost \nAttempting to reconnect...");
			while (!nh.connected()){
				// attempt to reconnect
				nh.initNode(ros_master);
				nh.spinOnce();
				Sleep(20);
			}
			// once connection is re-established...
			printf("Connected to server at %s\n\n", ros_master);
		}
		nh.spinOnce();
	}
	hdStopScheduler();
	hdUnschedule(hapticCallbackHandle);
	hdDisableDevice(hdGetCurrentDevice());
	system("pause");
	return 0;
}
