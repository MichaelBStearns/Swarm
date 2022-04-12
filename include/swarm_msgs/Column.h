#ifndef _ROS_swarm_msgs_Column_h
#define _ROS_swarm_msgs_Column_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "swarm_msgs/Square.h"

namespace swarm_msgs
{

  class Column : public ros::Msg
  {
    public:
      swarm_msgs::Square row[10];

    Column():
      row()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 10; i++){
      offset += this->row[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 10; i++){
      offset += this->row[i].deserialize(inbuffer + offset);
      }
     return offset;
    }

    virtual const char * getType() override { return "swarm_msgs/Column"; };
    virtual const char * getMD5() override { return "67820cc83c22bec62d23e6552a65e66b"; };

  };

}
#endif
