/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.
Copyright (c) 2024, 1X Technologies. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <ocs2_sqp/SqpMpc.h>
#include <rclcpp/rclcpp.hpp>

#include <humanoid_centroidal_mpc/CentroidalMpcInterface.h>
#include <mujoco_sim_interface/MujocoSimInterface.h>

#include <humanoid_centroidal_mpc/command/CentroidalMpcTargetTrajectoriesCalculator.h>
#include <humanoid_centroidal_mpc/mrt/CentroidalMpcMrtJointController.h>
#include "humanoid_common_mpc_ros2/ros_comm/Ros2ProceduralMpcMotionManager.h"
#include "humanoid_common_mpc_ros2/visualization/HumanoidVisualizer.h"

using namespace ocs2;
using namespace ocs2::humanoid;

int main(int argc, char** argv) {
  std::vector<std::string> programArgs;
  programArgs = rclcpp::remove_ros_arguments(argc, argv);
  if (programArgs.size() < 6) {
    throw std::runtime_error("No robot name, config folder, target command file, or description name specified. Aborting.");
  }

  const std::string robotName(argv[1]);//g1
  const std::string taskFile(argv[2]);//path to task.info
  const std::string referenceFile(argv[3]);//path to reference.info
  const std::string urdfFile(argv[4]);//path to g1_23dof.urdf
  const std::string gaitFile(argv[5]);//gait schedule (stance/swing timing)
  const std::string mjxFile(argv[6]);//MUJUCO scene XML file
  //初始化ros2环境
  rclcpp::init(argc, argv);
  //Build the MPC problem
  // Robot interface 这里读取文件并构建全部的OCP
  CentroidalMpcInterface interface(taskFile, urdfFile, referenceFile);

  // MPC 这里是OCS2求解器，黑盒。这里把interface配置好的设定和问题喂给求解器
  SqpMpc mpc(interface.mpcSettings(), interface.sqpSettings(), interface.getOptimalControlProblem(), interface.getInitializer());

  // Launch MPC ROS node 创建一个ROS2节点 名字是 g1_centroidal_mpc
  rclcpp::Node::SharedPtr nodeHandle = std::make_shared<rclcpp::Node>(robotName + "_centroidal_mpc");

  //配置ROS2的服务质量，设置队列深度为1,模式为尽力而为。
  auto qos = rclcpp::QoS(1);
  qos.best_effort();

  //创建可视化器，用于向rviz2发送机器人的姿态关节数据
  std::shared_ptr<HumanoidVisualizer> humanoidVisualizer(
      new HumanoidVisualizer(taskFile, interface.getPinocchioInterface(), interface.getMpcRobotModel(), nodeHandle));
  
  //创建目标轨迹计算器
  // Reference and motion management for Procedural MPC
  CentroidalMpcTargetTrajectoriesCalculator mpcTargetTrajectoriesCalculator(
      referenceFile, interface.getMpcRobotModel(), interface.getPinocchioInterface(), interface.getCentroidalModelInfo(),
      interface.mpcSettings().timeHorizon_);
  
  //定义一个C++ Lambda表达式，匿名函数。作用是作为一个桥梁和回调函数。
  //一旦上层给出一个4维的控制速度，它就调用计算器将其转化为MPC需要的未来时间段内的状态轨迹
  ProceduralMpcMotionManager::VelocityTargetToTargetTrajectories targetTrajectoriesFunc =
      [&mpcTargetTrajectoriesCalculator](const vector4_t& velocityTarget, scalar_t initTime, scalar_t finalTime,
                                         const vector_t& initState) mutable {
        return mpcTargetTrajectoriesCalculator.commandedVelocityToTargetTrajectories(velocityTarget, initTime, initState);
      };
  
  //创建ROS2运动管理器。将步态文件和速度转换函数传进去。
  auto ros2ProceduralMpcMotionManager = std::make_shared<Ros2ProceduralMpcMotionManager>(
      gaitFile, referenceFile, interface.getSwitchedModelReferenceManagerPtr(), interface.getMpcRobotModel(), targetTrajectoriesFunc);

  //让运动管理器订阅ROS2的手柄/键盘速度输入话题
  ros2ProceduralMpcMotionManager->subscribe(nodeHandle, qos);
  //将参考管理器和运动管理器注入到MPC求解器的内部。
  //确保求解器在计算每一步时，知道最新的步态和目标速度
  mpc.getSolverPtr()->setReferenceManager(interface.getReferenceManagerPtr());
  mpc.getSolverPtr()->addSynchronizedModule(ros2ProceduralMpcMotionManager);

  // Init Sim state
  robot::model::RobotDescription robotDescription(urdfFile);//解析URDF文件
  robot::model::RobotState initState(robotDescription, 2);//创建一个机器人在仿真中的状态变量initState
  initState.setConfigurationToZero();//将所有有关关节角度的配置清零

  const vector_t& initMpcState = interface.getInitialState();//从interface里提取MPC设定的初始状态
  const auto& mpcModel = interface.getMpcRobotModel();
  initState.setRootPositionInWorldFrame(mpcModel.getBasePosition(initMpcState));//并把这个绝对坐标设置给机器人的基座根节点

  vector_t mpcJointAngles = mpcModel.getJointAngles(initMpcState);
  // Todo set non zero orientation;
  std::vector<robot::joint_index_t> mpcJointIndices = robotDescription.getJointIndices(interface.modelSettings().mpcModelJointNames);
  for (size_t i = 0; i < mpcJointIndices.size(); i++) {
    initState.setJointPosition(mpcJointIndices[i], mpcJointAngles[i]);
  }//一个for循环。从 MPC 的初始状态中提取出各个关节的角度，并通过索引一一对应，赋值给仿真机器人对应的关节。
  
  //在终端打印出机器人初始的基座三维坐标
  std::cerr << "initState: " << initState.getRootPositionInWorldFrame().transpose() << std::endl;

  //组装mujoco仿真配置
  robot::mujoco_sim_interface::MujocoSimConfig config;

  config.scenePath = mjxFile;
  config.verbose = true;
  config.initStatePtr_ = std::make_shared<robot::model::RobotState>(std::move(initState));

  //正式实例化 MuJoCo 仿真接口
  robot::mujoco_sim_interface::MujocoSimInterface robotInterface(config, urdfFile);

  //创建 mpcJointController。这个对象是控制的核心中枢（属于 MRT - 模型参考跟踪组件）。
  //它一端连着 MPC 求解器，一端连着可视化，准备开始计算具体的关节控制量。
  CentroidalMpcMrtJointController mpcJointController(robotInterface.getRobotDescription(), interface.modelSettings(),
                                                     interface.getMpcRobotModel(), mpc, interface.getPinocchioInterface(),
                                                     interface.mpcSettings().mpcDesiredFrequency_, humanoidVisualizer);
  //打印提示信息，说明核心控制器已准备就绪。
  std::cout << "MPC MRT joint controller is set up. " << std::endl;

  // size_t mrtDeltaTMicroSeconds_ = 1000000 / (interface.mpcSettings().mrtDesiredFrequency_);
  //计算控制循环的周期（微秒）。控制主循环每 2 毫秒（500 Hz）必须执行一次。
  size_t mrtDeltaTMicroSeconds_ = 1000000 / (500);
  
  //初始化mujoco仿真，将当前仿真数据同步到接口。
  robotInterface.initSim();
  robotInterface.updateInterfaceStateFromRobot();
  //开启一个独立的异步线程去跑 MPC 优化求解
  mpcJointController.startMpcThread(robotInterface.getRobotState());

  //一个死循环等待。
  //因为第一次 MPC 求解需要一点时间，这里每 100 毫秒检查一次，直到 MPC 线程成功计算出第一个控制策略（Policy）才放行。
  while (!mpcJointController.ready()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  std::cout << "Initial MPC policy received. " << std::endl;

  // Wait to allow MPC policy to initialize
  //稳妥起见，让线程再小睡 200 毫秒确保完全就绪，然后正式开启 MuJoCo 的物理世界运动仿真（这时候机器人在物理世界里就会由于重力往下掉了）。
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  robotInterface.startSim();
  
  //刷新一次 ROS2 消息队列，处理可能已经收到的任何速度或步态指令
  rclcpp::spin_some(nodeHandle);


  //实时闭环控制主循环
  while (true) {
    //进入实机/仿真运行的绝对死循环。
    //第一步：根据当前的绝对系统时间，加上刚刚算好的 2000 微秒，设定好下一次循环必须开始的精确目标时间。
    auto targetTimeForNextIteration = std::chrono::steady_clock::now() + std::chrono::microseconds(mrtDeltaTMicroSeconds_);
    //感知。从 MuJoCo 物理引擎中读取机器人当前的最新状态
    robotInterface.updateInterfaceStateFromRobot();
    //决策。
    //将刚才读到的机器人实际状态传给控制器。
    //控制器会查看异步 MPC 线程算出来的最新策略，进行高速的内插和求导，计算出每一个关节当前应该给多大的扭矩、位置和速度，
    //并将结果写入到 robotJointAction 容器中。
    mpcJointController.computeJointControlAction(0.0, robotInterface.getRobotState(), robotInterface.getRobotJointAction());
    //执行。将刚刚计算出的各关节控制指令（Action）下发并应用到 MuJoCo 仿真机器人上。
    robotInterface.applyJointAction();
    
    //再次刷新 ROS2 消息队列，确保手柄的控制指令和 rviz2 的数据收发不会卡死。
    rclcpp::spin_some(nodeHandle);

    //获取当前时间，与第一步设定好的目标时间做对比。
    //超过，说明计算花的时间太长。每超过，睡眠来补偿
    auto currentTime = std::chrono::steady_clock::now();
    if (currentTime > targetTimeForNextIteration) {
      auto delay = std::chrono::duration_cast<std::chrono::microseconds>(currentTime - targetTimeForNextIteration).count();

      std::cerr << "Warning: MRT loop running slow by " << delay << " microseconds." << std::endl;
    } else {
      // Sleep in case sim loop is faster than specified
      std::this_thread::sleep_until(targetTimeForNextIteration);
    }
  }

  std::cout << "ende..." << std::endl;

  return 0;
}
