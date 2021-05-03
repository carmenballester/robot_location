#solo vale en los PCs del laboratorio Y con los pendrives de ASUS
if [ $# -ne 1 ]
  then
    echo "argumentos: numero_turtlebot"
  else
    #mi_ip=$(ip -o -4 addr list | grep "wlx" | awk '{print $4}' | cut -d/ -f1)
	mi_ip=192.168.1.106
    echo $mi_ip
	let offset=$1+4
    ip_turtlebot=192.168.1.$offset
	echo $ip_turtlebot
    export ROS_MASTER_URI=http://$ip_turtlebot:11311
    export ROS_HOSTNAME=$mi_ip    
fi


