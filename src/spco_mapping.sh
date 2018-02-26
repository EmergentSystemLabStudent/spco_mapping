#! /bin/sh

echo -n "trialname?(output_folder) >"
read trialname

gnome-terminal --command 'python learning.py '$trialname
roslaunch spco_mapping colormap.launch hoge:=$trialname
