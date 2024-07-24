#!/usr/bin/env bash

scriptDirectory="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
pythonScriptRootPath=`realpath $scriptDirectory/../share/BipedalLocomotionFramework/python`

echo $pythonScriptRootPath

echo "Starting Joint Torque Tracking"

echo "Welcome to the JointTorqueTrackingTest. I will first move the robot and then I will store the plot the data in a file called image.png"

echo "I'm going to move the robot. Watch out!"

blf-joint-torque-tracking --from blf-joint-torque-tracking-options.ini

echo "The trajectory is terminated."

echo "Plot data"

python3 "$pythonScriptRootPath"/blf_joint_torque_tracking_plot_dataset.py --dataset `ls -t Dataset* | head -1`


echo "Done. Thanks for using the script."

