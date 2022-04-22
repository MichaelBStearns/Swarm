#ifndef _ROS_swarm_msgs_Plus_h
#define _ROS_swarm_msgs_Plus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"

namespace swarm_msgs
{

  class Plus : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      typedef bool _pheromone_type;
      _pheromone_type pheromone;
      typedef const char* _state_type;
      _state_type state;
      float display[5];

    Plus():
      header(),
      pose(),
      pheromone(0),
      state(""),
      display()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->pose.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_pheromone;
      u_pheromone.real = this->pheromone;
      *(outbuffer + offset + 0) = (u_pheromone.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->pheromone);
      uint32_t length_state = strlen(this->state);
      varToArr(outbuffer + offset, length_state);
      offset += 4;
      memcpy(outbuffer + offset, this->state, length_state);
      offset += length_state;
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_displayi;
      u_displayi.real = this->display[i];
      *(outbuffer + offset + 0) = (u_displayi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_displayi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_displayi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_displayi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->display[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->pose.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_pheromone;
      u_pheromone.base = 0;
      u_pheromone.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->pheromone = u_pheromone.real;
      offset += sizeof(this->pheromone);
      uint32_t length_state;
      arrToVar(length_state, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_state; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_state-1]=0;
      this->state = (char *)(inbuffer + offset-1);
      offset += length_state;
      for( uint32_t i = 0; i < 5; i++){
      union {
        float real;
        uint32_t base;
      } u_displayi;
      u_displayi.base = 0;
      u_displayi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_displayi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_displayi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_displayi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->display[i] = u_displayi.real;
      offset += sizeof(this->display[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return "swarm_msgs/Plus"; };
    virtual const char * getMD5() override { return "dde45becb55d5fc2659b958bc95bb82a"; };

  };

}
#endif
