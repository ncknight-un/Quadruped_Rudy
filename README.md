# Quadruped_Rudy
Built from scratch Quadruped, using a raspberry pi and dynamixel servos, to simulate walkng in a limited torque and resource environment,

Executables: 
~ Calibration: Using calibrate.launch.xml, you can read the encoder positions at a zero'd joint positions to calibrate each motors encoding to a joint position of 0.
# Include the local Dynamixel SDK headers
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/DynamixelSDK/cpp/include
)