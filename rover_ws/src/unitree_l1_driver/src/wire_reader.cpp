// SPDX-License-Identifier: BSD-3-Clause
// Passive serial diagnostic using Unitree's shipped MAVLink dialect.
#include "mavlink/SysMavlink/mavlink.h"
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

static void number(double x) { if(std::isfinite(x))std::cout<<x;else std::cout<<"null"; }
int main(int argc,char** argv) {
  try {
    std::string port="/dev/ttyUSB0";
    for(int i=1;i<argc;++i) {
      if(std::string(argv[i])=="--port" && i+1<argc)port=argv[++i];
      else throw std::runtime_error("Usage: l1_wire_reader --port /dev/ttyUSB0");
    }
    int fd=open(port.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK);
    if(fd<0)throw std::runtime_error(std::string("open: ")+strerror(errno));
    if(flock(fd,LOCK_EX|LOCK_NB)<0)throw std::runtime_error("Port in use by another test");
    termios cfg{};
    if(tcgetattr(fd,&cfg)<0)throw std::runtime_error("tcgetattr failed");
    cfmakeraw(&cfg);cfsetispeed(&cfg,B2000000);cfsetospeed(&cfg,B2000000);
    cfg.c_cflag|=CLOCAL|CREAD;cfg.c_cflag&=~(CSTOPB|CRTSCTS);cfg.c_cc[VMIN]=0;cfg.c_cc[VTIME]=0;
    if(tcsetattr(fd,TCSANOW,&cfg)<0)throw std::runtime_error("tcsetattr failed");
    std::cout<<"L1JSON {\"type\":\"ready\",\"mode\":\"wire\"}\n"<<std::flush;
    uint64_t bytes=0,frames=0,bad_crc=0,imu=0,range=0,aux=0;
    double max_byte_gap=0;
    using Clock=std::chrono::steady_clock;
    auto start=Clock::now(),last=start,last_byte=start;
    mavlink_message_t msg{};mavlink_status_t status{};
    mavlink_ret_lidar_auxiliary_data_packet_t a{};
    for(;;) {
      pollfd p{fd,POLLIN,0};int result=poll(&p,1,50);
      if(result<0 && errno!=EINTR)throw std::runtime_error("poll failed");
      if(p.revents&(POLLERR|POLLHUP|POLLNVAL))throw std::runtime_error("Serial disconnected/error");
      if(p.revents&POLLIN) {
        uint8_t buf[8192];ssize_t n=read(fd,buf,sizeof(buf));
        if(n<0 && errno!=EAGAIN && errno!=EINTR)throw std::runtime_error("Serial read failed");
        if(n>0) {
          auto now=Clock::now();max_byte_gap=std::max(max_byte_gap,std::chrono::duration<double>(now-last_byte).count());last_byte=now;
          bytes+=n;
          for(ssize_t i=0;i<n;++i) {
            uint8_t r=mavlink_frame_char(MAVLINK_COMM_0,buf[i],&msg,&status);
            if(r==MAVLINK_FRAMING_BAD_CRC)++bad_crc;
            if(r!=MAVLINK_FRAMING_OK)continue;
            ++frames;
            if(msg.msgid==MAVLINK_MSG_ID_RET_IMU_ATTITUDE_DATA_PACKET)++imu;
            if(msg.msgid==MAVLINK_MSG_ID_RET_LIDAR_DISTANCE_DATA_PACKET)++range;
            if(msg.msgid==MAVLINK_MSG_ID_RET_LIDAR_AUXILIARY_DATA_PACKET) {
              ++aux;mavlink_msg_ret_lidar_auxiliary_data_packet_decode(&msg,&a);
            }
          }
        }
      }
      auto now=Clock::now();
      if(std::chrono::duration<double>(now-last).count()<1)continue;
      last=now;double elapsed=std::chrono::duration<double>(now-start).count();
      double gap=std::max(max_byte_gap,std::chrono::duration<double>(now-last_byte).count());
      std::cout<<"L1JSON {\"type\":\"wire\",\"elapsed\":"<<elapsed<<",\"bytes\":"<<bytes
        <<",\"frames\":"<<frames<<",\"bad_crc\":"<<bad_crc<<",\"imu_frames\":"<<imu
        <<",\"range_frames\":"<<range<<",\"aux_frames\":"<<aux<<",\"max_byte_gap\":"<<gap;
      if(aux) {
        std::cout<<",\"status_raw\":"<<int(a.lidar_work_status)<<",\"apd_voltage_NOT_supply\":";number(a.apd_voltage);
        std::cout<<",\"imu_temperature_raw\":";number(a.imu_temperature);
        std::cout<<",\"apd_temperature_raw\":";number(a.apd_temperature);
        std::cout<<",\"up_optical_q_raw\":";number(a.up_optical_q);
        std::cout<<",\"down_optical_q_raw\":";number(a.down_optical_q);
        std::cout<<",\"slow_motor_period_us\":"<<a.sys_rotation_period
          <<",\"fast_motor_period_us\":"<<a.com_rotation_period;
      }
      std::cout<<"}\n"<<std::flush;
    }
  }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
