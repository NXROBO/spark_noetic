#ifndef SDK_ARM_COMMON_H_
#define SDK_ARM_COMMON_H_

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <string.h>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <dynamic_reconfigure/server.h>
#include <sdk_sagittarius_arm/SDKSagittariusArmConfig.h>
#include <sensor_msgs/JointState.h>
#include "sdk_sagittarius_arm_constants.h"
#include "parser_base.h"

/*Fixed received buffer size*/
#define RECV_BUFFER_SIZE      1024
typedef struct 
{
    uint8_t flag;                                                         // 控制位。
    uint8_t servo_id;                                                     // 舵机ID。
    int16_t speed;                                                       // 实时速度
    int16_t payload;                                                      // 实时负载
    uint8_t voltage;                                                     // 实时电压
    uint16_t current;                                                     // 实时电流
}Str_ServoState;

namespace sdk_sagittarius_arm
{
    class CSDarmCommon
    {
    public:
        CSDarmCommon(ros::NodeHandle& nh, ros::NodeHandle& pnh);
        virtual ~CSDarmCommon();
        virtual int  Init();
        int          LoopOnce();
        void StartReceiveSerail();

        void PublishJointStates(unsigned char *buf);
        void         CheckJointNum(sdk_sagittarius_arm::SDKSagittariusArmConfig &config);
        void         UpdateConfig(sdk_sagittarius_arm::SDKSagittariusArmConfig &newConfig, uint32_t level = 0);
        virtual bool RebootDevice();
        sensor_msgs::JointState joint_states;
        Str_ServoState servo_state;

        double       GetExpectedFreq() const
        {
            return dExpectedFreq;
        }
        /*发送控制所有舵机的角度*/
        /**
                             * \param [in]  (v1, v2, v3, v4, v5, v6) v1~v6：1~6号舵机的弧度
                             * \param [out] 是否发送成功
                             */
        virtual int SendArmAllServer(float v1, float v2, float v3, float v4, float v5, float v6) = 0;
        virtual int SendArmAllServerTime(short difftime, float v1, float v2, float v3, float v4, float v5, float v6)= 0;
        virtual int SendArmAllServerCB(float v1, float v2, float v3, float v4, float v5, float v6) = 0;
        virtual int SendArmEndAction(unsigned char onoff, short value) = 0;
        virtual int SetArmVel(unsigned short vel)=0;
        virtual int SetArmAcc(unsigned char acc)=0;
        virtual int SetArmTorque(int torque[])=0;
        virtual int SendSerialData2Arm(char *buf, int length)=0;
        virtual int SendGetServoRealTimeInfo(unsigned char id)=0;

        /*发送舵机的锁与释放命令到机械臂*/
        /**
                             * \param [in]  (onoff) 0：锁住； 1：释放
                             * \param [out] 是否发送成功
                             */
        virtual int SendArmLockOrFree(unsigned char onoff) = 0;
    protected:
        virtual int InitDevice() = 0;
        virtual int InitArm();
        virtual int StopArmLock();
        virtual int CloseDevice() = 0;

        void LoopRcv();
        virtual unsigned char CheckSum(unsigned char *buf);





        /*从串口读取数据 */
        /*
                             * \param [in]  receiveBuffer 接收缓冲区.
                             * \param [in]  bufferSize 最大的接收字节 (0 terminated).
                             * \param [out] length 接收到的数据长度.
                             * */
        virtual int GetDataGram(unsigned char* receiveBuffer, int bufferSize, int *length) = 0;

        virtual bool DestroyThread(boost::thread **th);
        virtual void print_hex(unsigned char *buf, int len);

    protected:
        diagnostic_updater::Updater mDiagUpdater;

    private:
        ros::NodeHandle mNodeHandler;
        ros::NodeHandle mPrivNodeHandler;
        ros::Publisher  mScanPublisher;
        ros::Publisher  mDataPublisher;
        ros::Publisher motionPlan_pub;
        // Parser
        CParserBase    *mParser;

        bool            mPublishData;
        double          dExpectedFreq;
        boost::thread   *mThrcv;
        unsigned char   mStoreBuffer[RECV_BUFFER_SIZE];
        unsigned char   mRecvBuffer[RECV_BUFFER_SIZE];
        unsigned char   mFrameBuffer[RECV_BUFFER_SIZE];
        int             mDataLength;

        // Diagnostics
        diagnostic_updater::DiagnosedPublisher<sensor_msgs::LaserScan>* mDiagPublisher;

        // Dynamic Reconfigure
        SDKSagittariusArmConfig  mConfig;
        dynamic_reconfigure::Server<sdk_sagittarius_arm::SDKSagittariusArmConfig> mDynaReconfigServer;
    };
} // sdk_sagittarius_arm

#endif // SDK_ARM_COMMON_H_
