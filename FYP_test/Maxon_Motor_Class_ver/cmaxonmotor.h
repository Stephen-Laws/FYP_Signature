#ifndef CMAXONMOTOR_H
#define CMAXONMOTOR_H

#include "Definitions.h" // Maxon Motor Header file

#define TRUE 1
#define FALSE 0


class CMaxonMotor
{
private:
    char* PortName;
    unsigned int ErrorCode;
    unsigned short nodeID;
    void *keyHandle;


public:
    CMaxonMotor();
    CMaxonMotor(char[], unsigned int );

    void initializeDevice();

    void closeDevice();
    void EnableDevice();
    void DisableDevice();
    void Move(long TargetPosition);
    void GetCurrentPosition(int& CurrentPosition);
    void Halt();
    void activate_device();
};

#endif // CMAXONMOTOR_H

