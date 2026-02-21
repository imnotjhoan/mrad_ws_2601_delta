 ros2 launch delta_bringup gz_spawn.launch.py
# CARRO TUNEADO PARA RACE TRACK SIN OBSTACULOS
ros2 run delta_gap_following gap_distance_node --ros-args \
  -p circle_radius:=0.2 \
  -p max_linear_vel:=2.0 \
  -p brake_turn_angle:=1.5 \
  -p min_distance:=0.3 \
  -p max_distance:=12.0 \
  -p robot_radius:=0.1 \
  -p width_weight:=1.0 \
  -p depth_weight:=1.5
  
ros2 run delta_nav ttc_break_node --ros-args \
  -p ttc_threshold:=1.2 \
  -p min_distance_threshold:=0.9 \
  -p forward_angle_range:=10.0 \
  -p rear_angle_range:=10.0 \
  -p max_range:=2.0 \
  -p min_range:=0.1 \
  -p publish_rate:=100.0
  
  ros2 run delta_gap_following control_gap_ttc --ros-args \
  -p forward_velocity:=1.5 \
  -p kp:=0.8 \
  -p brake_turn_angle:=0.8 \
  -p start_flag:=false \
  -p pub_logger:=true
  
  ros2 run delta_nav start_controller_node 

  
# CARRO TUNEADO PARA RACE TRACK CON OBSTACULOS 

ros2 run delta_gap_following gap_distance_node --ros-args   -p circle_radius:=0.1   -p max_linear_vel:=1.6   -p brake_turn_angle:=1.5   -p min_distance:=0.3   -p max_distance:=12.0   -p robot_radius:=0.1   -p width_weight:=1.5   -p depth_weight:=1.0

ros2 run delta_nav ttc_break_node --ros-args   -p ttc_threshold:=0.85   -p min_distance_threshold:=1.2   -p forward_angle_range:=18.0   -p rear_angle_range:=10.0   -p max_range:=2.0   -p min_range:=0.1   -p publish_rate:=100.0

      self.declare_parameter('kp', 1.0)                    # Proportional gain for PD controller
        self.declare_parameter('forward_vel', 1.4)      # Constant forward velocity (m/s)
        self.declare_parameter('brake_turn_angle', 0.9) #Puede ser 1.0 si fv = 2.0
        self.declare_parameter('start_flag', False)               #Flag to signal that the car has received its first forward input from Joystickz
        self.declare_parameter('pub_logger', True)      #Flag to pub logger info




