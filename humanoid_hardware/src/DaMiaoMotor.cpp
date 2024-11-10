#include "humanoid_hardware/DaMiaoMotor.hpp"

void DaMiaoMotor::init(std::string name, uint16_t can_id, uint16_t feedback_can_id) {
    name_ = name;
    cmd_can_id_ = can_id;
    feedback_can_id_ = feedback_can_id;
}

void DaMiaoMotor::encode_cmd_msg(uint8_t data[8]) {
    uint16_t pos_temp = float_to_uint(cmd_msg_.target_pos, P_MIN, P_MAX, 16);
    uint16_t vel_temp = float_to_uint(cmd_msg_.target_vel, V_MIN, V_MAX, 12);
    uint16_t kp_temp = float_to_uint(cmd_msg_.kp, KP_MIN, KP_MAX, 12);
    uint16_t kd_temp = float_to_uint(cmd_msg_.kd, KD_MIN, KD_MAX, 12);
    uint16_t torq_temp = float_to_uint(cmd_msg_.target_torq, T_MIN, T_MAX, 12);

    data[0] = (pos_temp >> 8);
    data[1] = pos_temp;
    data[2] = (vel_temp >> 4);
    data[3] = ((vel_temp & 0xF) << 4) | (kp_temp >> 8);
    data[4] = kp_temp;
    data[5] = (kd_temp >> 4);
    data[6] = ((kd_temp & 0xF) << 4) | (torq_temp >> 8);
    data[7] = torq_temp;
}

void DaMiaoMotor::decode_sensor_feedback(uint8_t data[8]) {
    sensor_feedback_.can_id = (data[0]) & 0x0F;
    sensor_feedback_.state = (data[0]) >> 4;
    sensor_feedback_.pos_int = (data[1] << 8) | data[2];
    sensor_feedback_.vel_int = (data[3] << 4) | (data[4] >> 4);
    sensor_feedback_.torq_int = ((data[4] & 0xF) << 8) | data[5];
    sensor_feedback_.pos = uint_to_float(sensor_feedback_.pos_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
    sensor_feedback_.vel = uint_to_float(sensor_feedback_.vel_int, V_MIN, V_MAX, 12);     // (-45.0,45.0)
    sensor_feedback_.torq = uint_to_float(sensor_feedback_.torq_int, T_MIN, T_MAX, 12);   // (-18.0,18.0)
    sensor_feedback_.t_mos = (float)(data[6]);
    sensor_feedback_.t_rotor = (float)(data[7]);
}

void DaMiaoMotor::set_cmd(float target_pos, float target_vel, float target_torq) {
    cmd_msg_.target_pos = target_pos;
    cmd_msg_.target_vel = target_vel;
    cmd_msg_.target_torq = target_torq;
}

void DaMiaoMotor::set_cmd(float target_pos, float target_vel, float target_torq, float kp, float kd) {
    cmd_msg_.target_pos = target_pos;
    cmd_msg_.target_vel = target_vel;
    cmd_msg_.target_torq = target_torq;
    cmd_msg_.kp = kp;
    cmd_msg_.kd = kd;
}

