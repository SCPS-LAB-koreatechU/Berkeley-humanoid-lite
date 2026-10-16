# Mirrors unitree_hg IMUState_. All zero until an IMU driver is configured.
float32[4] quaternion     # w, x, y, z
float32[3] gyroscope      # [rad/s]
float32[3] accelerometer  # [m/s^2]
float32[3] rpy            # [rad]
int16 temperature
