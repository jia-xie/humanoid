/******************************************************************************
 * File:        DaMiaoMotor.hpp
 * Author:      Jia Xie
 * Date:        Nov 8 2024
 * Description: helper function for dm motor

 *
 *****************************************************************************/
#include <cstdint>
/**
 * this wrapper provides function to encode and decode can message for dm motor
 */
class DaMiaoMotor
{
public:
    // Structure for command message
    struct cmd_msg
    {
        float target_pos;
        float kp;
        float target_vel;
        float kd;
        float target_torq;
    };

    // Structure for sensor feedback
    struct sensor_feedback
    {
        uint8_t can_id;
        uint8_t state;
        uint16_t pos_int;
        uint16_t vel_int;
        uint16_t torq_int;
        float pos;
        float vel;
        float torq;
        float t_mos;
        float t_rotor;
    };
    
    void init(uint16_t cmd_can_id, uint16_t feedback_can_id);
    void set_cmd(float target_pos, float target_vel, float target_torq);
    void set_cmd(float target_pos, float target_vel, float target_torq, float kp, float kd);
    void encode_cmd_msg(uint8_t data[8]);
    void decode_sensor_feedback(uint8_t data[8]);

    uint16_t getCmdCanId() const { return cmd_can_id_; }
    uint16_t getFeedbackCanId() const { return feedback_can_id_; }
    const cmd_msg &getCmdMsg() const { return cmd_msg_; }
    const sensor_feedback &getSensorFeedback() const { return sensor_feedback_; }

private:
    uint16_t cmd_can_id_;
    uint16_t feedback_can_id_;

    const float P_MIN = -12.5;
    const float P_MAX = 12.5;
    const float V_MIN = -45.0;
    const float V_MAX = 45.0;
    const float T_MIN = -18.0;
    const float T_MAX = 18.0;
    const float KP_MIN = 0.0;
    const float KP_MAX = 500.0;
    const float KD_MIN = 0.0;
    const float KD_MAX = 5.0;

    cmd_msg cmd_msg_;
    sensor_feedback sensor_feedback_;

    int float_to_uint(float x, float x_min, float x_max, int bits)
    {
        float span = x_max - x_min;
        float offset = x_min;
        return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
    }

    float uint_to_float(int x_int, float x_min, float x_max, int bits)
    {
        /// converts unsigned int to float, given range and number of bits ///
        float span = x_max - x_min;
        float offset = x_min;
        return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
    }

};