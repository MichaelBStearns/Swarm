#ifndef _ROS_swarm_msgs_Square_h
#define _ROS_swarm_msgs_Square_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace swarm_msgs
{

  class Square : public ros::Msg
  {
    public:
      int8_t pheromones[5];

    Square():
      pheromones()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 5; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_pheromonesi;
      u_pheromonesi.real = this->pheromones[i];
      *(outbuffer + offset + 0) = (u_pheromonesi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pheromones[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 5; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_pheromonesi;
      u_pheromonesi.base = 0;
      u_pheromonesi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->pheromones[i] = u_pheromonesi.real;
      offset += sizeof(this->pheromones[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "swarm_msgs/Square"; };
    virtual const char * getMD5() override { return "98b485e50aa2d6a27ada82a8008715c5"; };

  };

}
#endif
