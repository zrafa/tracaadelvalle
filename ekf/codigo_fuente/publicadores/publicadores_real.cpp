// ============================================================================
//  publicadores.cpp  --  Replay de datos grabados para ekf_zmq
//
//  Lee cuatro archivos de texto y publica los datos en sus tópicos ZMQ.
//  Solo se publican los mensajes cuyo timestamp (en ms) cae dentro de
//  [TS_MIN, TS_MAX].
//
//  Archivos de entrada (columnas separadas por espacios):
//     rumbo_ms.txt     ->  timestamp_ms  rumbo
//     vel_ms_mps.txt   ->  timestamp_ms  velocidad
//     posiciones.txt   ->  timestamp_ms  x  y
//     posiciones2.txt  ->  timestamp_ms  x  y     (se fusiona con posiciones.txt)
//
//  Las dos fuentes de posición se intercalan por timestamp y se publican
//  en el mismo tópico TOP_POS.
//
//  El timestamp de publicación (formato idéntico al publicadores.cpp original)
//  va en SEGUNDOS (epoch). Por eso se divide ts_ms / 1000.0.
//
//  El primer instante del rango (TS_MIN) se mapea a "ahora + 2 s" y cada
//  mensaje se publica cuando el reloj real alcanza su offset relativo:
//  así se preserva el orden temporal ENTRE tópicos, no solo dentro de cada uno.
//
//  Compilar:  g++ -O2 -std=c++17 -Wall -pthread publicadores.cpp -o publicadores -lzmq
//  Usar:      ./publicadores
// ============================================================================

#include <zmq.h>

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <fstream>
#include <functional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "protocolo.h"

// ---- Rango temporal a publicar (milisegundos) ----
static const long long TS_MIN = 1616495245484LL;
static const long long TS_MAX = 1616495488297LL;

// ---- Archivos de entrada ----
static const char *ARCHIVO_RUMBO = "rumbo_ms.txt";
static const char *ARCHIVO_VEL   = "vel_ms_mps.txt";
static const char *ARCHIVO_POS   = "posiciones.txt";
static const char *ARCHIVO_POS2  = "posiciones2.txt";

static std::atomic<bool> g_salir{false};

// ============================================================================
//  Dato leído de un archivo
// ============================================================================
struct Dato {
    long long ts_ms;
    double    a;   // rumbo / velocidad / x
    double    b;   // solo se usa si el archivo tiene 3 columnas (y de posiciones)
};

static bool leer_archivo(const char *path, int ncols, std::vector<Dato> &out)
{
    std::ifstream f(path);
    if (!f) {
        fprintf(stderr, "No pude abrir '%s'\n", path);
        return false;
    }
    out.clear();
    std::string linea;
    long long total = 0;

    while (std::getline(f, linea)) {
        if (linea.empty()) continue;
        std::istringstream ss(linea);
        long long ts;
        double v1 = 0.0, v2 = 0.0;
        if (!(ss >> ts >> v1)) continue;
        if (ncols >= 3 && !(ss >> v2)) continue;

        total++;
        if (ts >= TS_MIN && ts <= TS_MAX) out.push_back({ts, v1, v2});
    }

    // Aseguramos orden temporal dentro de cada archivo
    std::sort(out.begin(), out.end(),
              [](const Dato &x, const Dato &y) { return x.ts_ms < y.ts_ms; });

    printf("[%-16s] leidos %7lld | dentro del rango: %6zu\n", path, total, out.size());
    return true;
}

// ============================================================================
//  Reloj de replay
// ============================================================================
static std::chrono::steady_clock::time_point g_t0_real;
static const double MARGEN_ARRANQUE_S = 2.0;   // margen para que se conecten los SUB

static void dormir_hasta_ms(long long ts_ms)
{
    auto objetivo = g_t0_real + std::chrono::microseconds((ts_ms - TS_MIN) * 1000LL);
    std::this_thread::sleep_until(objetivo);
}

// ============================================================================
//  Hilo de replay de un tópico
// ============================================================================
static void hilo_replay(void *ctx, const char *nombre, int puerto,
                        const char *topico, int ncols,
                        const std::vector<Dato> &datos)
{
    void *pub = zmq_socket(ctx, ZMQ_PUB);
    int linger = 3000;
    zmq_setsockopt(pub, ZMQ_LINGER, &linger, sizeof linger);

    char ep[64];
    snprintf(ep, sizeof ep, "tcp://*:%d", puerto);
    if (zmq_bind(pub, ep) != 0) {
        fprintf(stderr, "[%s] no pude hacer bind en %s (¿ya hay otro publicador?)\n", nombre, ep);
        zmq_close(pub);
        return;
    }

    char buf[200];
    long enviados = 0;
    for (const Dato &d : datos) {
        if (g_salir) break;
        dormir_hasta_ms(d.ts_ms);
        double te = d.ts_ms / 1000.0;         // ms -> s (mismo formato que el original)

        int len;
        if (ncols >= 3) {
            len = snprintf(buf, sizeof buf, "%s %.6f %.8f %.8f", topico, te, d.a, d.b);
        } else {
            len = snprintf(buf, sizeof buf, "%s %.6f %.6f", topico, te, d.a);
        }
        zmq_send(pub, buf, len, 0);
        enviados++;
    }

    printf("[%-5s] termino: %ld mensajes publicados\n", nombre, enviados);
    zmq_close(pub);
}

static void manejador(int) { g_salir = true; }

int main(int argc, char **argv)
{
    (void)argc; (void)argv;
    signal(SIGINT, manejador);

    std::vector<Dato> rumbo, vel, pos1, pos2, pos;
    if (!leer_archivo(ARCHIVO_RUMBO, 2, rumbo)) return 1;
    if (!leer_archivo(ARCHIVO_VEL,   2, vel))   return 1;
    if (!leer_archivo(ARCHIVO_POS,   3, pos1))  return 1;
    if (!leer_archivo(ARCHIVO_POS2,  3, pos2))  return 1;

    // Fusionar las dos fuentes de posición en un único stream ordenado
    pos.reserve(pos1.size() + pos2.size());
    pos.insert(pos.end(), pos1.begin(), pos1.end());
    pos.insert(pos.end(), pos2.begin(), pos2.end());
    std::sort(pos.begin(), pos.end(),
              [](const Dato &x, const Dato &y) { return x.ts_ms < y.ts_ms; });
    printf("[pos (merge)]     total fusionado: %6zu\n", pos.size());

    if (rumbo.empty() && vel.empty() && pos.empty()) {
        fprintf(stderr, "No hay datos en el rango [%lld, %lld].\n", TS_MIN, TS_MAX);
        return 1;
    }

    // Referencia temporal: TS_MIN se mapea a "ahora + MARGEN"
    g_t0_real = std::chrono::steady_clock::now()
              + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                    std::chrono::duration<double>(MARGEN_ARRANQUE_S));

    printf("Reproduciendo rango [%lld, %lld]  (%.3f s de datos)...\n",
           TS_MIN, TS_MAX, (TS_MAX - TS_MIN) / 1000.0);

    void *ctx = zmq_ctx_new();

    std::vector<std::thread> hilos;
    if (!rumbo.empty())
        hilos.emplace_back(hilo_replay, ctx, "rumbo", PUERTO_RUMBO, TOP_RUMBO, 2, std::cref(rumbo));
    if (!vel.empty())
        hilos.emplace_back(hilo_replay, ctx, "vel",   PUERTO_VEL,   TOP_VEL,   2, std::cref(vel));
    if (!pos.empty())
        hilos.emplace_back(hilo_replay, ctx, "pos",   PUERTO_POS,   TOP_POS,   3, std::cref(pos));
    for (auto &h : hilos) h.join();

    zmq_ctx_term(ctx);
    printf("Listo.\n");
    return 0;
}