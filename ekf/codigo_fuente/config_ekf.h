// ============================================================================
//  config_ekf.h  --  Parámetros de configuración de ekf_zmq.cpp
//  (Solo se movieron de ekf_zmq.cpp; los valores y su significado no cambian.)
// ============================================================================
#ifndef CONFIG_EKF_H
#define CONFIG_EKF_H

#include <stddef.h>      // size_t
#include "protocolo.h"   // PI

// ============================================================================
//  CONFIGURACIÓN
// ============================================================================

// ---- Reordenamiento ----
static const double HISTORIA_S = 5.0;    // cuánto pasado se guarda para poder corregir datos tardíos [s]
                                         // (debe superar el mayor retraso de cualquier sensor)
static const size_t CAP_COLA   = 20000;  // capacidad máxima de la cola receptor -> EKF

// ---- Ruido de PROCESO (continuo: se multiplica por dt, así no depende de cuántos eventos lleguen) ----
static const double RUIDO_POS = 0.05;    // random walk en x,y   [m / sqrt(s)]
static const double RUIDO_V   = 0.10;    // aceleración desconocida [(m/s) / sqrt(s)]
static const double RUIDO_PSI = 0.05;   // giro de rumbo NO explicado por el modelo [rad / sqrt(s)]
                                         // (más alto = el filtro sigue más de cerca al INS)
//  (SIGMA_DELTA, del protocolo, entra en el ruido de psi)

// ---- Compuerta de innovación ----
// Se rechaza una medición si innovación^2 / S > COMPUERTA. 9.0 = 3 sigmas. 0 = desactivada.
static const double COMPUERTA = 0.0;         // para pos y vel. 0 = nunca se rechaza (9.0 = rechaza a 3 sigmas)
static const double COMPUERTA_RUMBO = 0.0;  // para el rumbo del INS. 0 = nunca se rechaza

// ---- Arranque en frío: el filtro no sabe nada, así que parte con incertidumbre enorme ----
static const double ARR_STD_XY  = 100.0; // [m]
static const double ARR_STD_PSI = PI;    // [rad]
static const double ARR_STD_V   = 5.0;   // [m/s]
static const double MARGEN_INICIO_S = 0.5; // el estado inicial se fecha 0.5 s antes del primer evento

// ---- Evaluación contra ground truth (solo pruebas) ----
static const double CONVERGENCIA_S = 5.0;   // no se mide el error hasta 5 s después de tener pos+vel+rumbo
static const double IMPRIMIR_CADA_S = 5.0;
static const double SILENCIO_FIN_S = 3.0;

// Índices de las filas de z
enum { Z_X = 0, Z_Y = 1, Z_V = 2, Z_PSI = 3 };

#endif
