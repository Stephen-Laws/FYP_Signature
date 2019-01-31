#pragma once
#ifndef MAXONMOTOR_H
#define MAXONMOTOR_H

#include "Definitions.h" // Maxon Motor Header file
# define M_PI           3.14159265358979323846  /* pi */
using namespace std;

class maxonMotor
{
private:
	//char* portName;
	char* portNameM1;
	char* portNameM2;
	DWORD errorCode;
	//WORD nodeID;
	WORD nodeID1, nodeID2, nodeID3, nodeID4;
	//HANDLE keyHandle;
	HANDLE keyHandleM1, keyHandleM2, keyHandleM3, keyHandleM4;

private:
	void CloseDevice(HANDLE keyHandle);
	void EnableDevice(HANDLE keyHandle, WORD nodeID);
	void DisableDevice(HANDLE keyHandle, WORD nodeID);
	void SetMode(HANDLE keyHandle, WORD nodeID, char mode);
	void SetSensorType(HANDLE keyHandle, WORD nodeID, WORD type);
	void SetSpeed(HANDLE keyHandle, WORD nodeID, int targetVel);
	void GetCurrentPosition(HANDLE keyHandle, WORD nodeID, long& pos);
	void Halt(HANDLE keyHandle, WORD nodeID);
	void GetCurrentSpeed(HANDLE keyHandle, WORD nodeID, long& vel);
	void GetAbsEncInfo(HANDLE keyHandle, WORD nodeID);
	void MovetoPosition(HANDLE keyHandle, WORD nodeID, int pos);
public:
	maxonMotor(char*);
	HANDLE ActivateMotorMaster(char* PortName, WORD nodeID);
	HANDLE ActivateMotorSub(char* PortName, WORD nodeID);
	void ActivateAll();
	void CloseAll();
	void DisableAll();
	void SetModeAll(char mode);
	void SetAllVelocityMode();
	void SetAllPPM();
	void SetSpeeds(int *targetVels);
	void GetMotorPositions(long* CurrentPositions);
	void GetMotorSpeeds(long* CurrentVelocities);
	void HaltAll();
	void Info(HANDLE keyHandle, WORD nodeID);
	void ReadEncsAbsolute(long* absPositions);
	void EnableAll();
	void MoveToPos(double* targetPos, long* initOffset);
	void Home(long* initOffset);
	void clearFaults();
};

void convertPos(double* targetPos, long* targetInt, long* initOffset, long currYaw) {
	int count = 0;
	int exit = 0;
	double targDouble;
	targDouble = targetPos[0] / (2 * M_PI) * 4096 - initOffset[0];
	targetInt[0] = (long)round(targDouble);
	long tmp = currYaw- targetInt[0];
	while (exit == 0) {
		if (tmp > 2048) {
			tmp = tmp - 4096;
			count++;
		}
		//else if (tmp - (targetInt[0]+initOffset[0]) > 2048) { 
		//	count++; exit = 1; 
		//}
		else if (tmp < -2048) {
			tmp = tmp + 4096;
			count--;
		}
		//else if ((targetInt[0]+initOffset[0]) - tmp > 2048) {
		//	count--;
		//	exit = 1;
		//}
		else { exit = 1; }
	}
	targDouble = targetInt[0] + count * 4096;
	targetInt[0] = (long)round(targDouble);
	targDouble = targetPos[1] / (2 * M_PI) * 4096 - initOffset[1]; //pitch
	targetInt[1] = (long)round(targDouble);
	targDouble = targetPos[2] / (25.4) * 2000 - initOffset[2]; //linear
	targetInt[2] = (long)round(targDouble);
	targDouble = targetPos[3] / (12) * 2000 - initOffset[3]; //transverse
	targetInt[3] = (long)round(targDouble);
	//for (int i = 0; i < 4; i++) { cout << targetInt[i] << " "; }
	//cout << endl;
}

maxonMotor::maxonMotor(char* portM1) {
	portNameM1 = portM1;
	nodeID1 = 1;
	nodeID2 = 2;
	nodeID3 = 3;
	nodeID4 = 4;
}

void maxonMotor::CloseDevice(HANDLE keyHandle) {//Close requested device
	DWORD errorCode;
	//cout << "Closing Device!" << endl;
	if (keyHandle != 0)
		VCS_CloseDevice(keyHandle, &errorCode);
	VCS_CloseAllDevices(&errorCode);
}

//Activate requested motor and return handle
HANDLE maxonMotor::ActivateMotorMaster(char *portName, WORD nodeID) {
	CloseAll(); //Check all motors initially closed before activating
	char* protocolStackName = const_cast<char*>("MAXON SERIAL V2");
	char* interfaceName = const_cast<char*>("USB");
	char* deviceName = const_cast<char*>("EPOS2");
	DWORD errorCode = 0x00;
	DWORD timeout_ = 500;
	DWORD baudrate_ = 1000000;
	HANDLE keyHandle;

	keyHandle = VCS_OpenDevice(deviceName, protocolStackName, interfaceName, portName, &errorCode);
	if (keyHandle == 0) {cout << "Open device failure, error code=" << errorCode << endl;}
	else {cout << "Open device success!" << endl;}

	if (!VCS_SetProtocolStackSettings(keyHandle, baudrate_, timeout_, &errorCode)) {
		cout << "Set protocol stack settings failed!, error code=" << errorCode << endl;
		CloseDevice(keyHandle);
	}
	EnableDevice(keyHandle, nodeID);
	return keyHandle;
}
HANDLE maxonMotor::ActivateMotorSub(char *portName, WORD nodeID) {
	//CloseAll(); //Check all motors initially closed before activating
	char* subProtocolStackName = const_cast<char*>("CANopen");
	char* subDeviceName = const_cast<char*>("EPOS2");
	DWORD errorCode = 0x00;
	DWORD timeout_ = 500;
	DWORD baudrate_ = 1000000;
	HANDLE keyHandle;

	keyHandle = VCS_OpenSubDevice(keyHandleM1, subDeviceName, subProtocolStackName, &errorCode);
	if (keyHandle == 0) { cout << "Open device failure, error code=" << errorCode << endl; }
	else { cout << "Open device success!" << endl; }

	if (!VCS_SetProtocolStackSettings(keyHandle, baudrate_, timeout_, &errorCode)) {
		cout << "Set protocol stack settings failed!, error code=" << errorCode << endl;
		CloseDevice(keyHandle);
	}
	EnableDevice(keyHandle, nodeID);
	return keyHandle;
}
//Set motor state to enable
void maxonMotor::EnableDevice(HANDLE keyHandle, WORD nodeID){
	BOOL IsInFault = FALSE;
	DWORD errorCode;
	if (VCS_GetFaultState(keyHandle, nodeID, &IsInFault, &errorCode)){
		if (IsInFault && !VCS_ClearFault(keyHandle, nodeID, &errorCode)){
			cout << "Clear fault failed! , error code=" << errorCode << endl;
			return;
		}
		int IsEnabled = FALSE;
		if (VCS_GetEnableState(keyHandle, nodeID, &IsEnabled, &errorCode)){
			if (!IsEnabled && !VCS_SetEnableState(keyHandle, nodeID, &errorCode)){
				cout << "Set enable state failed!, error code=" << errorCode << endl;
			}
			//else{cout << "Enable succeeded!" << endl;}
		}
	}
	else{cout << "Get fault state failed!, error code, error code=" << errorCode << endl;}
}
//Set motor to disable state
void maxonMotor::DisableDevice(HANDLE keyHandle, WORD nodeID){
	BOOL IsInFault = FALSE;
	if (VCS_GetFaultState(keyHandle, nodeID, &IsInFault, &errorCode)){
		if (IsInFault && !VCS_ClearFault(keyHandle, nodeID, &errorCode)){
			cout << "Clear fault failed!, error code=" << errorCode << endl;
			return;
		}
		int IsEnabled = FALSE;
		if (VCS_GetEnableState(keyHandle, nodeID, &IsEnabled, &errorCode)){
			if (IsEnabled && !VCS_SetDisableState(keyHandle, nodeID, &errorCode)){
				cout << "Set disable state failed!, error code=" << errorCode << endl;
			}
			//else{cout << "Set disable state succeeded!" << endl;}
		}
	}
	else{cout << "Get fault state failed!, error code=" << errorCode << endl;}
}

//Set motor mode;
//1 = profile position mode
//3 = position velocity mode
//6 = homing mode 
//7 = inerpolated position mode
//-1 = position mode
//-2 = velocity mode
//-3 = current mode
//-5 = master end mode
//-6 = step direction mode
void maxonMotor::SetMode(HANDLE keyHandle, WORD nodeID, char mode){
	DWORD errorCode;
	if (!VCS_SetOperationMode(keyHandle, nodeID, mode, &errorCode)) {
		cout << "Set Operation Mode error, error code =" << errorCode << endl;
	}
}

void maxonMotor::SetSpeed(HANDLE keyHandle, WORD nodeID, int targetVel){
	if (!VCS_SetVelocityMust(keyHandle, nodeID, targetVel, &errorCode)) {
			cout << "Set Velocity Failed!, error code: " << errorCode; 
		}
}

void maxonMotor::MovetoPosition(HANDLE keyHandle, WORD nodeID, int pos) {
	DWORD errorCode;
	if (!VCS_MoveToPosition(keyHandle, nodeID, pos, TRUE, TRUE, &errorCode)) {
		cout << "Move to Position failed" << endl;
	}

}

void maxonMotor::GetCurrentPosition(HANDLE keyHandle, WORD nodeID, long& pos){
	DWORD errorCode = 0;
	if (!VCS_GetPositionIs(keyHandle, nodeID, &pos, &errorCode)) {
		cout << " error while getting current position , error code=" << errorCode << endl;
	}
}

void maxonMotor::SetSensorType(HANDLE keyHandle, WORD nodeID, WORD type) {
	DWORD errorCode = 0;
	if (!VCS_SetSensorType(keyHandle, nodeID, type, &errorCode)) {
		cout << "Error setting Encoder type, code: " << errorCode << endl;
	}
}

void maxonMotor::GetAbsEncInfo(HANDLE keyHandle, WORD nodeID) {
	WORD DataRate, NbMultiTurn, NbSingleTurn;
	BOOL invPol;
	DWORD errorCode;
	VCS_GetSsiAbsEncoderParameter(keyHandle, nodeID, &DataRate, &NbMultiTurn, &NbSingleTurn, &invPol, &errorCode);
	cout << "Data rate: " << DataRate << endl;
	cout << "Number multi turn bits: " << NbMultiTurn << endl;
	cout << "Number single turn bits: " << NbSingleTurn << endl;
	cout << "Inverse Polarity: " << invPol << endl;

}

void maxonMotor::GetCurrentSpeed(HANDLE keyHandle, WORD nodeID, long& vel) {
	DWORD errorCode = 0;
	if (!VCS_GetVelocityIs(keyHandle, nodeID, &vel, &errorCode)) {
		cout << "Error getting velocity, code: " << endl;
	}
}
void maxonMotor::Halt(HANDLE keyHandle, WORD nodeID){
	DWORD errorCode = 0;
	if (!VCS_HaltPositionMovement(keyHandle, nodeID, &errorCode)){
		cout << "Halt position movement failed!, error code=" << errorCode << endl;
	}
}


void maxonMotor::Info(HANDLE keyHandle, WORD nodeID) {//Display information about requested motor 
	WORD motorType, encType; // 
	DWORD encRes; //Encoder resolution
	BOOL encPol;
	DWORD maxVel, maxAcc; //max velocity & acceleration
	VCS_GetMotorType(keyHandle, nodeID, &motorType, &errorCode);
	VCS_GetSensorType(keyHandle, nodeID, &encType, &errorCode);
	VCS_GetIncEncoderParameter(keyHandle, nodeID, &encRes, &encPol, &errorCode);
	VCS_GetMaxProfileVelocity(keyHandle, nodeID, &maxVel, &errorCode);
	VCS_GetMaxAcceleration(keyHandle, nodeID, &maxAcc, &errorCode);
	cout << "Motor type: " << motorType << endl;
	cout << "Encoder type: " << encType << endl;
	cout << "Encoder Resolution: " << encRes << "   Polarity: " << encPol << endl;
	cout << "Max Velocity: " << maxVel << endl;
	cout << "Max Acceleration: " << maxAcc << endl;
}


//Public functions that set parameters of all motors 
void maxonMotor::SetSpeeds(int* targetVels) {
	SetSpeed(keyHandleM1, nodeID1, targetVels[0]);
	SetSpeed(keyHandleM2, nodeID2, targetVels[1]);
	SetSpeed(keyHandleM3, nodeID3, targetVels[2]);
	SetSpeed(keyHandleM4, nodeID4, targetVels[3]);
}

void maxonMotor::ActivateAll() {//Public - activate all motors
	keyHandleM1 = ActivateMotorMaster(portNameM1, nodeID1);
	keyHandleM2 = ActivateMotorSub(portNameM1, nodeID2);
	keyHandleM3 = ActivateMotorSub(portNameM1, nodeID3);
	keyHandleM4 = ActivateMotorSub(portNameM1, nodeID4);
	cout << "Motors Activated!" << endl;
}

void maxonMotor::SetModeAll(char mode) {//Set all motors to same mode
	SetMode(keyHandleM1, nodeID1, mode);
	SetMode(keyHandleM2, nodeID2, mode);
	SetMode(keyHandleM3, nodeID3, mode);
	SetMode(keyHandleM4, nodeID4, mode);
	clearFaults();
	EnableAll();
}

void maxonMotor::SetAllVelocityMode() {
	SetModeAll(-2);
	cout << "Velocity Mode initiated" << endl;
}

void maxonMotor::SetAllPPM() {
	SetModeAll(1);
	cout << "PPM Initiated" << endl;
}

void maxonMotor::MoveToPos(double* targetPos, long* initOffset) {
	long targetInt[4] = { 0,0,0,0 };
	long currYaw;
	SetAllPPM();
	EnableAll();
	GetCurrentPosition(keyHandleM1, nodeID1, currYaw);
	convertPos(targetPos, targetInt, initOffset, currYaw);
	MovetoPosition(keyHandleM1, nodeID1, targetInt[0]);
	MovetoPosition(keyHandleM2, nodeID2, targetInt[1]);
	MovetoPosition(keyHandleM3, nodeID3, targetInt[2]);
	MovetoPosition(keyHandleM4, nodeID4, targetInt[3]);
}
void maxonMotor::GetMotorPositions(long* CurrentPositions) {
	long pos = 0;
	GetCurrentPosition(keyHandleM1, nodeID1, pos); 
	CurrentPositions[0] = pos; //Pitch in radians
	GetCurrentPosition(keyHandleM2, nodeID2, pos); 
	CurrentPositions[1] = pos;//Yaw in radians 
	GetCurrentPosition(keyHandleM3, nodeID3, pos);
	CurrentPositions[2] = pos;//Transverse in mm
	GetCurrentPosition(keyHandleM4, nodeID4, pos);
	CurrentPositions[3] = pos;//Linear in mm 
}

void maxonMotor::GetMotorSpeeds(long* CurrentVelocities) {
	long vel = 0;
	GetCurrentSpeed(keyHandleM1, nodeID1, vel);
	CurrentVelocities[0] = vel;
	GetCurrentSpeed(keyHandleM2, nodeID2, vel);
	CurrentVelocities[1] = vel;
	GetCurrentSpeed(keyHandleM3, nodeID3, vel);
	CurrentVelocities[2] = vel;
	GetCurrentSpeed(keyHandleM4, nodeID4, vel);
	CurrentVelocities[3] = vel;
}

void maxonMotor::DisableAll() {
	DisableDevice(keyHandleM1, nodeID1);
	DisableDevice(keyHandleM2, nodeID2);
	DisableDevice(keyHandleM3, nodeID3);
	DisableDevice(keyHandleM4, nodeID4);
	cout << "All motors disabled" << endl;
	
}
void maxonMotor::CloseAll() {
	CloseDevice(keyHandleM1);
	CloseDevice(keyHandleM2);
	CloseDevice(keyHandleM3);
	CloseDevice(keyHandleM4);
	VCS_CloseAllDevices(&errorCode);
}

void maxonMotor::HaltAll() {
	Halt(keyHandleM1, nodeID1);
	Halt(keyHandleM2, nodeID2);
	Halt(keyHandleM3, nodeID3);
	Halt(keyHandleM4, nodeID4);
}

void maxonMotor::clearFaults() {
	DWORD errorCode;
	VCS_ClearFault(keyHandleM1, nodeID1, &errorCode);
	VCS_ClearFault(keyHandleM2, nodeID2, &errorCode);
	VCS_ClearFault(keyHandleM3, nodeID3, &errorCode);
	VCS_ClearFault(keyHandleM4, nodeID4, &errorCode);
}
void maxonMotor::EnableAll() {
	EnableDevice(keyHandleM1, nodeID1);
	EnableDevice(keyHandleM2, nodeID2);
	EnableDevice(keyHandleM3, nodeID3);
	EnableDevice(keyHandleM4, nodeID4);
	cout << "All motors enabled" << endl;
}
void maxonMotor::ReadEncsAbsolute(long* absPositions) {
	long pos = 0;
	SetSensorType(keyHandleM1, nodeID1, 5); //5 == SSI absolute, grey coded
	GetAbsEncInfo(keyHandleM1, nodeID1);
	SetSensorType(keyHandleM2, nodeID2, 5);
	GetAbsEncInfo(keyHandleM2, nodeID2);
	GetCurrentPosition(keyHandleM1, nodeID1, pos);
	absPositions[0] = pos;
	GetCurrentPosition(keyHandleM2, nodeID2, pos);
	absPositions[1] = pos;
	SetSensorType(keyHandleM1, nodeID1, 1); //1 = inc encoder 3 channel
	SetSensorType(keyHandleM2, nodeID2, 1);
}

void maxonMotor::Home(long* initOffset) {
	double targetPos[4] = { 0, -M_PI / 2, 120, 0 }; //'Home' position
	SetAllPPM();
	MoveToPos(targetPos, initOffset);
	EnableAll();
}
#endif //MAXONMOTOR_H


