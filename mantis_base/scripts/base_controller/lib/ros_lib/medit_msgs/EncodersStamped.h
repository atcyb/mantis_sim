#ifndef _ROS_mantis_msgs_EncodersStamped_h
#define _ROS_mantis_msgs_EncodersStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "mantis_msgs/Encoders.h"

namespace mantis_msgs
{

  class EncodersStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef mantis_msgs::Encoders _encoders_type;
      _encoders_type encoders;

    EncodersStamped():
      header(),
      encoders()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->encoders.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->encoders.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "mantis_msgs/EncodersStamped"; };
    virtual const char * getMD5() override { return "162975f999a4d19c3bdc0165594516da"; };

  };

}
#endif
