// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rc_serial_driver
{
//     struct SendPacket
//     {
//         uint8_t header = 0x5A;
//         float linear_x;
//         float linear_y;
//         float angular_z;
//         uint16_t checksum = 0; // This is for the CRC checksum. Remember to sizeof(SendPacket) represent the real length of the data

//     } __attribute__((packed));

    // struct SendPacket {
    // // this is hgy  version r2
    // //   uint8_t header = 0x5A;
    //   uint8_t header = 43;
    //   float cmd_vx = 0.0f;
    //   float cmd_vy = 0.0f;
    //   float desire_yaw ;
    //   float measure_yaw;
    //   //0对应的是速度，1对应的是有角度度的情况
    //   uint8_t if_angle_flag = 1;
    //   float x_dot = 0.0f;
    //   float y_dot = 0.0f;
    //   //0是高，默认状态，找球，1是找球，2是放球，2之后立即切0   
    //   uint8_t roboarm_state = 0;
    //   uint8_t d = 0;
    //   uint8_t tail = 43;
    //   // uint16_t checksum

    // } __attribute__((packed));

    struct SendPacket{
        uint8_t header = 43;
        float cmd_vx;
        float cmd_vy;
        float desire_yaw;
        // fetch ball mode 
        // 0 是没有，
        // 1是停止
        // 2是吃球，正常找球对应的模式
        // 3是盘球第一种模式，对应两种模式
        // 4是盘球第二种模式，对应两种模式
        // 5是shoot
        // 6是从存好球发射


        // 要球，2->3 或者 2->4
        // 不要球 2-6
        // 7是老汤伸缩： 缩
        // 8是老汤伸缩： 开


        uint8_t mode;
        uint8_t tail = 43;
    } __attribute__((packed));

    struct ReceivePacket {
      uint8_t header;
      uint8_t attached;
      uint8_t tail;
    } __attribute__((packed));

    inline ReceivePacket fromVector(const std::vector<uint8_t> &data)
    {
        ReceivePacket packet;
        std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
        return packet;
    }

    inline std::vector<uint8_t> toVector(const SendPacket &data)
    {
        std::vector<uint8_t> packet(sizeof(SendPacket));
        std::copy(
            reinterpret_cast<const uint8_t *>(&data),
            reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
        return packet;
    }

}

#endif