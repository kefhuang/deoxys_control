#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

struct Sample {
  std::array<double, 7> q, dq, dq_d, ddq_d, command;
  std::array<double, 42> jacobian;
  double dt;
};

int main(int argc, char** argv) {
  if (argc != 5) {
    std::cerr << "usage: joint-velocity-benchmark ROBOT_IP JOINT_1_BASED SPEED OUTPUT.csv\n";
    return 2;
  }
  const size_t joint = std::stoul(argv[2]) - 1;
  const double speed = std::stod(argv[3]);
  if (joint >= 7 || speed <= 0.0 || speed > 0.2) return 2;
  constexpr std::array<double, 7> q_min{{-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973}};
  constexpr std::array<double, 7> q_max{{ 2.8973, 1.7628, 2.8973,-0.0698, 2.8973, 3.7525, 2.8973}};
  std::vector<Sample> samples; samples.reserve(4000);
  bool error = false;
  try {
    franka::Robot robot(argv[1]);
    franka::Model model = robot.loadModel();
    auto initial = robot.readOnce();
    if (std::min(initial.q[joint]-q_min[joint], q_max[joint]-initial.q[joint]) < 0.20)
      throw std::runtime_error("selected joint is within 0.20 rad of its limit");
    double t = 0.0, previous_command = 0.0;
    robot.control([&](const franka::RobotState& s, franka::Duration p) {
      const double dt = p.toSec(); t += dt;
      // 0.2 s ramp +, 0.4 hold, 0.2 ramp through zero, 0.4 hold -, 0.2 ramp to zero.
      double cmd = 0.0;
      if (t < .2) cmd = speed * t/.2;
      else if (t < .6) cmd = speed;
      else if (t < 1.0) cmd = speed - 2.0*speed*(t-.6)/.4;
      else if (t < 1.4) cmd = -speed;
      else if (t < 1.6) cmd = -speed + speed*(t-1.4)/.2;
      cmd = std::max(-speed, std::min(speed, cmd));
      std::array<double,7> command{}; command[joint] = cmd;
      samples.push_back({s.q,s.dq,s.dq_d,s.ddq_d,command,
                         model.zeroJacobian(franka::Frame::kEndEffector,s),dt});
      previous_command = cmd;
      if (t >= 1.6) return franka::MotionFinished(franka::JointVelocities(command));
      return franka::JointVelocities(command);
    }, franka::ControllerMode::kJointImpedance, true, 1000.0);
  } catch (const franka::Exception& e) { error=true; std::cerr << e.what() << '\n'; }
  if (samples.empty()) return 1;
  std::ofstream out(argv[4]); out << std::setprecision(17);
  out << "k,dt"; for(const char* n:{"q","dq","dq_d","ddq_d","dq_command"}) for(int i=1;i<=7;++i) out<<','<<n<<i;
  out << ",sigma_min,condition_number,command_acceleration,command_jerk\n";
  double prev_a=0, prev_c=0, max_dt=0, sigma_min_all=1e9, cond_max=0;
  std::array<double,7> mq{},mc{},md{},mdd{},ma{},mj{},mind{}; mind.fill(1e9);
  std::vector<double> dts;
  for(size_t k=0;k<samples.size();++k){auto&s=samples[k]; dts.push_back(s.dt);max_dt=std::max(max_dt,s.dt);
    Eigen::Map<const Eigen::Matrix<double,6,7>> J(s.jacobian.data()); Eigen::JacobiSVD<Eigen::Matrix<double,6,7>> svd(J);
    double smin=svd.singularValues().minCoeff(), cond=svd.singularValues().maxCoeff()/smin; sigma_min_all=std::min(sigma_min_all,smin);cond_max=std::max(cond_max,cond);
    double a=k?(s.command[joint]-prev_c)/s.dt:0, jerk=k>1?(a-prev_a)/s.dt:0; prev_c=s.command[joint];prev_a=a;
    out<<k<<','<<s.dt;for(auto*x:{&s.q,&s.dq,&s.dq_d,&s.ddq_d,&s.command})for(double v:*x)out<<','<<v;out<<','<<smin<<','<<cond<<','<<a<<','<<jerk<<'\n';
    for(int i=0;i<7;++i){mq[i]=std::max(mq[i],std::abs(s.q[i]));mc[i]=std::max(mc[i],std::abs(s.command[i]));md[i]=std::max(md[i],std::abs(s.dq_d[i]));mdd[i]=std::max(mdd[i],std::abs(s.ddq_d[i]));mind[i]=std::min(mind[i],std::min(s.q[i]-q_min[i],q_max[i]-s.q[i]));}ma[joint]=std::max(ma[joint],std::abs(a));mj[joint]=std::max(mj[joint],std::abs(jerk));
  }
  std::sort(dts.begin(),dts.end()); double p99=dts[std::min(dts.size()-1,size_t(.99*dts.size()))]; size_t late=std::count_if(dts.begin(),dts.end(),[](double x){return x>0.0015;});
  std::cout<<"samples="<<samples.size()<<" max_dt="<<max_dt<<" p99_dt="<<p99<<" late_gt_1.5ms="<<late<<" sigma_min="<<sigma_min_all<<" condition_max="<<cond_max<<" reflex_or_error="<<error<<'\n';
  for(int i=0;i<7;++i)std::cout<<"J"<<i+1<<" max_abs_q="<<mq[i]<<" max_abs_dq_command="<<mc[i]<<" max_abs_dq_d="<<md[i]<<" max_abs_ddq_d="<<mdd[i]<<" max_abs_command_acceleration="<<ma[i]<<" max_abs_command_jerk="<<mj[i]<<" min_distance_to_joint_limit="<<mind[i]<<'\n';
  return error?1:0;
}
