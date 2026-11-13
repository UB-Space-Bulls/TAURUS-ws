// SPDX-License-Identifier: BSD-3-Clause
// SDK reads may block forever. The Python parent owns deadlines and termination.
#include "unitree_lidar_sdk.h"
#include <fcntl.h>
#include <sys/file.h>
#include <cerrno>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>

using namespace unitree_lidar_sdk;
static void number(std::ostream& o, double x) {
  if (std::isfinite(x)) o << x; else o << "null";
}
static void array(std::ostream& o, const float* p, int n) {
  o << '[';
  for (int i=0; i<n; ++i) { if(i) o << ','; number(o,p[i]); }
  o << ']';
}
static void emit(const std::string& s) { std::cout << "L1JSON " << s << '\n' << std::flush; }
int main(int argc, char** argv) {
  try {
    std::string port="/dev/ttyUSB0";
    int scans=18; bool points=false, passive=false;
    for(int i=1;i<argc;++i) {
      std::string a=argv[i];
      if(a=="--port" && i+1<argc) port=argv[++i];
      else if(a=="--scans" && i+1<argc) scans=std::stoi(argv[++i]);
      else if(a=="--points") points=true;
      else if(a=="--passive") passive=true;
      else throw std::runtime_error("Unknown/incomplete argument: "+a);
    }
    if(scans<1 || scans>180) throw std::runtime_error("scans must be 1..180");
    // Advisory lock shared with our raw reader. Check fuser for other drivers.
    int guard=open(port.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK);
    if(guard<0) throw std::runtime_error("open "+port+": "+strerror(errno));
    if(flock(guard,LOCK_EX|LOCK_NB)<0) throw std::runtime_error("Port in use by another test");
    auto* sdk=createUnitreeLidarReader();
    if(!sdk || sdk->initialize(scans,port,2000000,0,0.001,0,50,0)!=0)
      throw std::runtime_error("SDK initialization failed");
    if(!passive) sdk->setLidarWorkingMode(NORMAL);
    emit("{\"type\":\"ready\",\"mode\":\"sdk\"}");
    std::cerr << "SDK " << sdk->getVersionOfSDK() << "; port " << port << '\n';
    for(;;) {
      auto type=sdk->runParse(); // continuous draining, no 1 ms ROS timer throttle
      if(type==NONE) { usleep(100); continue; }
      // Epoch seconds need full double precision; 10 significant digits rounds
      // subsecond stamps away and makes a live IMU look frozen.
      std::ostringstream o; o << std::setprecision(17);
      if(type==IMU) {
        const auto& m=sdk->getIMU();
        o << "{\"type\":\"imu\",\"id\":" << m.id << ",\"stamp\":"; number(o,m.stamp);
        o << ",\"q\":"; array(o,m.quaternion,4);
        o << ",\"gyro\":"; array(o,m.angular_velocity,3);
        o << ",\"accel\":"; array(o,m.linear_acceleration,3); o << '}'; emit(o.str());
      } else if(type==POINTCLOUD) {
        const auto& c=sdk->getCloud(); size_t valid=0;
        double rmin=INFINITY,rmax=0;
        for(const auto& p:c.points) {
          if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||
             !std::isfinite(p.intensity)||!std::isfinite(p.time)) continue;
          ++valid; double r=std::sqrt(double(p.x)*p.x+double(p.y)*p.y+double(p.z)*p.z);
          rmin=std::min(rmin,r);rmax=std::max(rmax,r);
        }
        o << "{\"type\":\"cloud\",\"id\":" << c.id << ",\"stamp\":";number(o,c.stamp);
        o << ",\"count\":" << c.points.size() << ",\"valid\":" << valid
          << ",\"invalid\":" << c.points.size()-valid << ",\"range_min\":";number(o,rmin);
        o << ",\"range_max\":";number(o,rmax);
        if(points) {
          o << ",\"points\":[";bool first=true;
          for(const auto& p:c.points) {
            if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)||
               !std::isfinite(p.intensity)||!std::isfinite(p.time)) continue;
            if(!first)o<<',';
            first=false;
            o<<'['<<p.x<<','<<p.y<<','<<p.z<<','<<p.intensity<<','<<p.time<<','<<p.ring<<']';
          } o<<']';
        } o<<'}';emit(o.str());
      } else if(type==AUXILIARY) {
        o<<"{\"type\":\"aux\",\"dirty_percent\":";number(o,sdk->getDirtyPercentage());
        o<<'}';emit(o.str());
      } else if(type==VERSION) {
        std::cerr<<"Firmware "<<sdk->getVersionOfFirmware()<<'\n';
      }
    }
  } catch(const std::exception& e) { std::cerr<<e.what()<<'\n';return 2; }
}
