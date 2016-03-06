#include "UavcanMotorConfig.hpp"
#include "motor_param.hpp"

int main(int argc, char **argv)
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <node-id> <target-id>" << std::endl;
        return 1;
    }

    const int node_id = std::stoi(argv[1]);
    const int target_id = std::stoi(argv[2]);

    ros::init(argc, argv, "motor_send_config_once");
    ros::NodeHandle nh;
    UavcanMotorConfig uc_motor_config(node_id);

    /* Send control config (parameters and PID gains) */
    cvra::motor::config::LoadConfiguration::Request config_msg;
    get_config_param(nh, config_msg);
    uc_motor_config.send_config(target_id, config_msg);

    /* Send stream config */
    cvra::motor::config::FeedbackStream::Request stream_msg;

    get_stream_param(nh, stream_msg, CURRENT_PID_STREAM);
    uc_motor_config.send_stream_config(target_id, stream_msg);

    get_stream_param(nh, stream_msg, VELOCITY_PID_STREAM);
    uc_motor_config.send_stream_config(target_id, stream_msg);

    get_stream_param(nh, stream_msg, POSITION_PID_STREAM);
    uc_motor_config.send_stream_config(target_id, stream_msg);

    get_stream_param(nh, stream_msg, INDEX_STREAM);
    uc_motor_config.send_stream_config(target_id, stream_msg);

    get_stream_param(nh, stream_msg, MOTOR_ENCODER_STREAM);
    uc_motor_config.send_stream_config(target_id, stream_msg);

    get_stream_param(nh, stream_msg, MOTOR_POSITION_STREAM);
    uc_motor_config.send_stream_config(target_id, stream_msg);

    get_stream_param(nh, stream_msg, MOTOR_TORQUE_STREAM);
    uc_motor_config.send_stream_config(target_id, stream_msg);

    /* Send control enable */
    cvra::motor::config::EnableMotor::Request enable_msg;
    get_enable_param(nh, enable_msg);
    uc_motor_config.send_enable(target_id, enable_msg);

    return 0;
}
