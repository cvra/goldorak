time timestamp
string node_name

float32 position      # [rad]
float32 velocity      # [rad/s]
float32 acceleration  # [rad/s^2]
float32 torque        # [Nm]
float32 voltage       # [V]

uint8 MODE_CONTROL_TRAJECTORY   = 0
uint8 MODE_CONTROL_POSITION     = 1
uint8 MODE_CONTROL_VELOCITY     = 2
uint8 MODE_CONTROL_TORQUE       = 3
uint8 MODE_CONTROL_VOLTAGE      = 4
uint8 mode
