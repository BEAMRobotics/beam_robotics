
bag=$1

set -- "${@:2}"

if [[ -f $bag ]]
then
  image_topic=$2
  set -- "${@:2}"

  echo $@
  tmp=${image_topic#*/}
  image_namespace=${tmp%/*}

  image_color=$image_namespace/image_color
  image_downscaled=$image_namespace/downscaled/image_color

  rosparam set /use_sim_time true

  rosbag play $bag --clock & 
  rosrun nodelet nodelet standalone image_proc/debayer image_raw:=$image_topic image_color:=$image_color &  
  rosrun nodelet nodelet standalone image_proc/resize image:=$image_color _scale_width:=0.25 _scale_height:=0.25 ~image:=$image_downscaled & 
  rosbag record $image_downscaled $@

  rosnode kill $bag
else
  echo "Path to bag does not exist. Exiting"
  echo "Usage: ./debayeranddownsample.sh [path_to_bag] [raw_image_topic] [rest of topics to keep]"
fi
