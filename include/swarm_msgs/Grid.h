#ifndef _ROS_swarm_msgs_Grid_h
#define _ROS_swarm_msgs_Grid_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "swarm_msgs/Column.h"

namespace swarm_msgs
{

  class Grid : public ros::Msg
  {
    public:
      swarm_msgs::Column column[25];

    Grid():
      column()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 25; i++){
      offset += this->column[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 25; i++){
      offset += this->column[i].deserialize(inbuffer + offset);
      }
     return offset;
    }

    virtual const char * getType() override { return "swarm_msgs/Grid"; };
    virtual const char * getMD5() override { return "c5b445e2bca73a30cc767bc84f07b7c1"; };

  };

}
#endif
