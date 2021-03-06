<launch>
    <arg name="camera_width" />
    <arg name="camera_height" />
    <arg name="motion_type" />
    <arg name="rotation_type" />
    <arg name="number_of_repetitions" />
    <arg name="time_limit" />
    <arg name="calibration_duration" />
    <arg name="robot_position" />
    <arg name="calibration_output_file" default="" />
    <arg name="calib_eval" value="$(eval arg('calibration_output_file') != '')" />
    <node pkg="sound_play" type="soundplay_node.py" name="sound_play" />
    <node pkg="wm_voice_generator" type="wm_voice_component_short.py" name="wm_voice_generator" />
    <node pkg="rehabilitation_framework" type="Exercises.py" name="rehabilitation_framework" required="true" args="--calibrate_only --calibration_output_file $(arg calibration_output_file)" output="screen" if="$(eval arg('calib_eval') == 1)" />
    <node pkg="rehabilitation_framework" type="Exercises.py" name="rehabilitation_framework" required="true" output="screen" unless="$(eval arg('calib_eval') == 1)" />
</launch>
