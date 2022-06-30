#ifndef SDK_ARM_PARSER__
#define SDK_ARM_PARSER__

#include <sdk_sagittarius_arm/parser_base.h>

namespace sdk_sagittarius_arm
{
    class CSDKarmParser : public CParserBase
    {
    public:
        CSDKarmParser();
        virtual ~CSDKarmParser();
        virtual int Parse(char *data, size_t data_length, SDKSagittariusArmConfig &config, sensor_msgs::LaserScan &msg);

    };
} /*namespace sdk_sagittarius_arm*/

#endif /*SDK_ARM_PARSER__*/
