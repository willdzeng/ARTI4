# export HUSKY_DESCRIPTION=$(catkin_find --without-underlays --first-only husky_description urdf/description.xacro 2>/dev/null)
export GAZEBO_MODEL_PATH=$(catkin_find --without-underlays --first-only track_generator model)/:${GAZEBO_MODEL_PATH}
