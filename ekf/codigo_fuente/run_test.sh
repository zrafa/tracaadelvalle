#!/bin/bash
# Compila y corre la prueba completa: EKF + 4 publicadores + ground truth.
# Requiere: g++, libzmq3-dev (libzmq5), y tinyekf.h en esta carpeta.
set -e
DURACION=${1:-90}
g++ -O2 -std=c++17 -Wall -pthread ekf_zmq.cpp     -o ekf_zmq      -lzmq
g++ -O2 -std=c++17 -Wall -pthread publicadores.cpp -o publicadores -lzmq

./ekf_zmq --fin-por-silencio &      # primero el EKF (zmq reintenta la conexión solo)
EKF=$!
sleep 0.5
./publicadores "$DURACION"
wait $EKF
