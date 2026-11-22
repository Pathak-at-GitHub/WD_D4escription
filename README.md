# WD_Description (Band_drone) – Gazebo Harmonic SDF

Run (from this folder):
    export GZ_SIM_RESOURCE_PATH=$PWD
    gz sdf -k WD_Description/model.sdf        # syntax check
    gz sim -r world.sdf
    gz topic -t /wd/rotor_1/cmd_vel -m gz.msgs.Double -p 'data: 200'   # rad/s
ROS 2:  ros2 launch ./sim.launch.py   then   ros2 topic pub /wd/rotor_1/cmd_vel std_msgs/msg/Float64 "{data: 200}"

Model facts
- Poses/axes come from the STEP assembly transforms; mesh scale 0.001 (STL in mm). Model frame = SolidWorks frame (Y up);
  world.sdf rolls it +90 deg about X.
- Joints: body4 -fixed- body3_1/body3_2 -fixed- motor_1..4 ; rotor_N_joint = revolute (continuous) about child Y.
- Blade 1,4 = CW mesh; 2,3 = CCW mesh. Blades have no collision. Joint controller only spins blades – no aerodynamic thrust.
- Mass/inertia are computed from mesh volume at 1040 kg/m^3 (ABS) => ~25.8 g total. Edit DENSITY / MASS_OVERRIDE_KG in gen_sdf.py and run it.
