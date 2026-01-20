#ifndef MOTOR_CONTROL_ROS2__UNITREE_MOTOR_HPP_
#define MOTOR_CONTROL_ROS2__UNITREE_MOTOR_HPP_

#include "motor_control_ros2/motor_base.hpp"
#include "motor_control_ros2/hardware/serial_interface.hpp"
#include <memory>
#include <vector>

namespace motor_control {

// 宇树协议常量
constexpr int SERIAL_FRAME_TIMEOUT = 10000; // 10ms

#pragma pack(1)

// 宇树电机命令帧结构 (34 bytes)
struct UnitreeMotorCommandFrame {
    uint8_t  head[2];     // 0xFE, 0xEE
    uint8_t  mode;        // 0:Idle, 5:OpenLoop, 10:ClosedLoop
    uint8_t  temp;        // 温度
    uint8_t  MDR;         // Driver Mode
    uint8_t  merr;        // Motor Error
    uint16_t T;           // Torque * 256
    uint16_t W;           // Velocity * 128 + 16000
    uint32_t Pos;         // Position * 16383.5 / (2*PI)
    uint16_t K_P;         // Kp * 2048
    uint16_t K_W;         // Kd * 1024
    uint8_t  foot_force[2];
    uint8_t  foot_speed[2];
    uint8_t  foot_acc[2];
    uint8_t  foot_pos[2];
    uint32_t crc;
};

// 宇树电机反馈帧结构 (78 bytes)
struct UnitreeMotorResponseFrame {
    uint8_t  head[2];     // 0xFE, 0xEE
    uint8_t  mode;
    uint8_t  temp;
    uint8_t  MDR;
    uint8_t  merr;
    uint16_t T;           // Torque
    uint16_t W;           // Velocity
    uint32_t Pos;         // Position
    uint16_t foot_force;
    uint16_t foot_speed;
    uint16_t foot_acc;
    uint8_t  reserved[56]; // 保留/未使用
    uint32_t crc;
};

#pragma pack()

/**
 * @brief 宇树电机驱动类
 * 
 * 支持 A1 和 GO-8010 电机。
 * 通过 RS485 串口通信，力位混合控制。
 */
class UnitreeMotor : public MotorBase {
public:
  /**
   * @brief 构造函数
   * @param joint_name 关节名称
   * @param type 电机类型 (UNITREE_A1 或 UNITREE_GO8010)
   * @param serial_port 共享的串口指针
   * @param motor_id 电机 ID (0, 1, 2)
   * @param direction 电机方向 (1 或 -1)
   * @param offset 零点偏移
   */
  UnitreeMotor(const std::string& joint_name, MotorType type,
               std::shared_ptr<hardware::SerialInterface> serial_port,
               uint8_t motor_id, int direction = 1, float offset = 0.0f);
               
  // 实现基类纯虚函数
  // 注意：宇树电机协议非常特殊，通常不是通过标准的 "updateFeedback" (CAN风格) 来更新
  // 而是通过串口收发。这里我们可能需要稍微调整接口用法，或者由管理器调用专门的 updateSerial()。
  // 为了兼容 Frame work，我们保留接口但可能不完全使用 can_id 参数。
  void updateFeedback(uint32_t can_id, const uint8_t* data, size_t len) override;
  void getControlFrame(uint32_t& can_id, uint8_t* data, size_t& len) override;
  void enable() override;
  void disable() override;
  
  /**
   * @brief 设置力位混合控制命令
   * 所有参数为输出轴参数，内部会自动处理减速比转换
   */
  void setHybridCommand(float pos, float vel, float kp, float kd, float torque);
  
  /**
   * @brief 构建并发送命令帧
   * @return 成功返回 true
   */
  bool sendCommand();
  
  /**
   * @brief 接收并解析反馈帧
   * @return 成功返回 true
   */
  bool receiveFeedback();
  
  /**
   * @brief 获取内部命令帧结构（用于调试）
   */
  const UnitreeMotorCommandFrame& getCommandFrame() const { return cmd_frame_; }

private:
  std::shared_ptr<hardware::SerialInterface> serial_port_;
  uint8_t motor_id_;
  int direction_;
  float offset_;
  
  UnitreeMotorCommandFrame cmd_frame_;
  UnitreeMotorResponseFrame res_frame_;
  
  // 待发送的控制参数（电机侧）
  float rotor_p_des_;
  float rotor_v_des_;
  float rotor_kp_;
  float rotor_kd_;
  float rotor_t_ff_;
  
  uint32_t crc32_core(uint32_t* ptr, uint32_t len);
};

} // namespace motor_control

#endif // MOTOR_CONTROL_ROS2__UNITREE_MOTOR_HPP_
