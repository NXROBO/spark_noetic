#ifndef SDK_ARM_COMMON_SERIAL__
#define SDK_ARM_COMMON_SERIAL__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sdk_sagittarius_arm_common.h"

namespace sdk_sagittarius_arm
{
    class CSDarmCommonSerial : public CSDarmCommon
    {
    public:
        CSDarmCommonSerial(const std::string &serialname, const std::string &baudrate, int &timelimit, ros::NodeHandle& nh, ros::NodeHandle& pnh, bool free_torque);
        virtual ~CSDarmCommonSerial();

    protected:
        /*Override functions*/
        virtual int InitDevice();
        virtual int CloseDevice();

        //virtual int SendDeviceReq(const char *req, std::vector<unsigned char> *resp);

        virtual int SendArmAllServer(float v1, float v2, float v3, float v4, float v5, float v6);

        virtual int SendArmAllServerTime(short difftime, float v1, float v2, float v3, float v4, float v5, float v6);
        virtual int SendArmAllServerCB(float v1, float v2, float v3, float v4, float v5, float v6);
        virtual int SendArmLockOrFree(unsigned char onoff);
        virtual int SendArmEndAction(unsigned char onoff, short value);
        virtual int SetArmVel(unsigned short vel);
        virtual int SetArmAcc(unsigned char acc);
        virtual int SetArmTorque(int torque[]);
        virtual int SendGetServoRealTimeInfo(unsigned char id);

        virtual int GetDataGram(unsigned char *receiveBuffer, int bufferSize, int *length);

        virtual int SendSerialData2Arm(char *buf, int length);

        virtual unsigned char CheckSum(unsigned char *buf);
    private:
        int                          mFd;
        size_t                       mBytesReceived;
        std::string                  mSerialName;
        int                          mBaudrate;
        int                          mTimeLimit;
        int                          mExitFreeTorque;
    };

} /*namespace sdk_sagittarius_arm*/

#endif /*SDK_ARM_COMMON_SERIAL__*/
