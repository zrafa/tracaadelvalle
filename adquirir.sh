#!/bin/bash


limpiar () {
   for job in $(jobs -p);do 
	   echo ..finalizando.. $job 
           sudo pkill -P $job
           sudo kill $job
   done  > /dev/null  2>&1
   exit 
}

trap limpiar SIGINT 

if ! [[ -d $1 ]]; then
	echo "ERROR. Directorio no existe. uso: ./adquirir.sh NOMBRE_DIR/"
	exit 1
fi

if ! [ "`ls ${1}/* | wc -l`" -eq 0 ] ; then 
	echo "ERROR. Directorio DEBE estar VACIO. 
		uso: ./adquirir.sh NOMBRE_DIR/"
	exit 1
fi

# 10hz
echo -e '\xB5\x62\x06\x08\x06\x00\x64\x00\x01\x00\x01\x00\x7A\x12' > /dev/ttyACM0
./caja_adquisidora/gps_time/gps_time >> ${1}/gps.txt &

./caja_adquisidora/lidar-servo/lidar_servo_pc >> ${1}/lidar.txt &
(cd ./caja_adquisidora/IMU/ && sudo python imu.py) >> ${1}/imu.txt &
(cd ./caja_adquisidora/IMU/ && sudo python magnetometro.py) >> ${1}/magnetometro.txt &
(sudo v4l2-ctl -d /dev/video0 --set-ctrl=focus_auto=0 && v4l2-ctl -d /dev/video0 --set-ctrl=focus_absolute=60 && ./caja_adquisidora/cam_udp_streaming/cliente-udp-image-streaming/cliente_v4l2 --848 -w ${1}/ ) >> /dev/null &

echo "
	Iniciando adquisición de datos...
	Presionar Ctrl+C para finalizar"

wait

