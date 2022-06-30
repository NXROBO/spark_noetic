#ifndef PARSER_BASE__
#define PARSER_BASE__

#include "sdk_sagittarius_arm/SDKSagittariusArmConfig.h"
#include "sensor_msgs/LaserScan.h"

namespace sdk_sagittarius_arm
{
    enum ExitCode
    {
        ExitSuccess = 0,
        ExitError   = 1,
        ExitFatal   = 2
    };

    class CParserBase
    {
    public:
        CParserBase();
        virtual ~CParserBase();

        virtual int Parse(char *data,
                          size_t data_length,
                          SDKSagittariusArmConfig &config,
                          sensor_msgs::LaserScan &msg) = 0;
    };
} /*namespace sdk_sagittarius_arm*/

#endif /*PARSER_BASE__*/
