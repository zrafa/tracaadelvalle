// ============================================================================
//  pub_sub.h  --  Recepción de eventos por zmq (SUB) + cola receptor -> EKF
//
//  Uso desde el EKF:
//     std::thread receptor(hilo_receptor, ctx);      // arranca la recepción
//     cola_sacar_todo(lote, 100);                    // espera y retira los mensajes recibidos
//     g_salir = true; receptor.join();               // para terminar
//  La implementación está en pub_sub.cpp.
// ============================================================================
#ifndef PUB_SUB_H
#define PUB_SUB_H

#include <stddef.h>
#include <atomic>
#include <vector>

enum Tipo { T_GIRO = 0, T_VEL, T_RUMBO, T_POS, T_GT, T_N };
extern const char *const NOMBRE_TIPO[T_N];

struct Mensaje {
    Tipo   tipo;
    double t;              // timestamp del dato (cuándo se midió)
    double a, b, c, d;     // valores (según el tipo, ver protocolo.h)
    double t_rx;           // cuándo lo recibimos (solo para estadísticas de retraso)
};

// Estadísticas y bandera de salida (definidas en pub_sub.cpp)
extern size_t               g_cola_max;        // máxima ocupación vista
extern size_t               g_cola_perdidos;   // mensajes descartados por cola llena
extern std::atomic<long>    g_malformados;
extern std::atomic<bool>    g_salir;

// El EKF duerme hasta que haya algo (o pase timeout_ms) y se lleva TODO lo que haya en un solo lock.
void cola_sacar_todo(std::vector<Mensaje> &out, int timeout_ms);

// Hilo receptor: crea su socket SUB sobre el contexto zmq dado y encola lo que llega.
void hilo_receptor(void *ctx);

#endif
