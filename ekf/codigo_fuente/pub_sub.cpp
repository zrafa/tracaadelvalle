// ============================================================================
//  pub_sub.cpp  --  Recepción de eventos por zmq (SUB) + cola receptor -> EKF
//  (Código movido de ekf_zmq.cpp sin cambios. Interfaz en pub_sub.h)
// ============================================================================
#include <zmq.h>

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <vector>

#include "pub_sub.h"
#include "config_ekf.h"    // CAP_COLA
#include "protocolo.h"     // TOP_*, PUERTO_*, HOST

// ============================================================================
//  Mensajes y COLA  (receptor -> EKF)
// ============================================================================
const char *const NOMBRE_TIPO[T_N] = { "giro", "vel", "rumbo", "pos", "gt" };

static std::mutex              g_mx;
static std::condition_variable g_cv;
static std::deque<Mensaje>     g_cola;
size_t                         g_cola_max = 0;       // máxima ocupación vista
size_t                         g_cola_perdidos = 0;  // mensajes descartados por cola llena
std::atomic<long>              g_malformados{0};
std::atomic<bool>              g_salir{false};

static double ahora_epoch(void)
{
    using namespace std::chrono;
    return duration<double>(system_clock::now().time_since_epoch()).count();
}

// El receptor llama a esto. Lock corto, nunca espera al EKF.
static void cola_poner(const Mensaje &m)
{
    {
        std::lock_guard<std::mutex> lk(g_mx);
        if (g_cola.size() >= CAP_COLA) {      // EKF muy atrasado: tiramos lo más viejo
            g_cola.pop_front();
            g_cola_perdidos++;
        }
        g_cola.push_back(m);
        if (g_cola.size() > g_cola_max) g_cola_max = g_cola.size();
    }
    g_cv.notify_one();                         // <- "llegó un dato nuevo", despierta al EKF
}

// El EKF duerme hasta que haya algo (o pase timeout_ms) y se lleva TODO lo que haya en un solo lock.
void cola_sacar_todo(std::vector<Mensaje> &out, int timeout_ms)
{
    std::unique_lock<std::mutex> lk(g_mx);
    g_cv.wait_for(lk, std::chrono::milliseconds(timeout_ms), [] { return !g_cola.empty(); });
    out.assign(g_cola.begin(), g_cola.end());
    g_cola.clear();
}

// ============================================================================
//  Hilo receptor (zmq)
// ============================================================================

// "topico timestamp v1 v2 ..."  ->  Mensaje.   false si está mal formado.
static bool parsear(const char *s, Mensaje &m)
{
    char top[16];
    double t, a = 0, b = 0, c = 0, d = 0;
    int n = sscanf(s, "%15s %lf %lf %lf %lf %lf", top, &t, &a, &b, &c, &d);
    int necesarios;

    if      (!strcmp(top, TOP_GIRO))  { m.tipo = T_GIRO;  necesarios = 3; }
    else if (!strcmp(top, TOP_VEL))   { m.tipo = T_VEL;   necesarios = 3; }
    else if (!strcmp(top, TOP_RUMBO)) { m.tipo = T_RUMBO; necesarios = 3; }
    else if (!strcmp(top, TOP_POS))   { m.tipo = T_POS;   necesarios = 4; }
    else if (!strcmp(top, TOP_GT))    { m.tipo = T_GT;    necesarios = 6; }
    else return false;

    if (n < necesarios) return false;
    m.t = t; m.a = a; m.b = b; m.c = c; m.d = d;
    return true;
}

void hilo_receptor(void *ctx)
{
    // Un socket zmq NO es thread-safe: se crea y se usa solo en este hilo.
    void *sub = zmq_socket(ctx, ZMQ_SUB);

    int timeout_ms = 100;   // para poder revisar g_salir
    int linger = 0;
    int hwm = 100000;       // el buffer interno de zmq también es amplio
    zmq_setsockopt(sub, ZMQ_RCVTIMEO, &timeout_ms, sizeof timeout_ms);
    zmq_setsockopt(sub, ZMQ_LINGER,   &linger,     sizeof linger);
    zmq_setsockopt(sub, ZMQ_RCVHWM,   &hwm,        sizeof hwm);

    // Un único SUB conectado a todos los publicadores. El prefijo lleva espacio: "vel " no matchea "velx".
    const char *topicos[] = { TOP_GIRO " ", TOP_VEL " ", TOP_RUMBO " ", TOP_POS " ", TOP_GT " " };
    const int   puertos[] = { PUERTO_GIRO, PUERTO_VEL, PUERTO_RUMBO, PUERTO_POS, PUERTO_GT };
    for (int i = 0; i < 5; i++) {
        zmq_setsockopt(sub, ZMQ_SUBSCRIBE, topicos[i], strlen(topicos[i]));
        char ep[64];
        snprintf(ep, sizeof ep, "tcp://%s:%d", HOST, puertos[i]);
        zmq_connect(sub, ep);   // si el publicador aún no existe, zmq reintenta solo
    }

    char buffer[256];
    while (!g_salir) {
        int n = zmq_recv(sub, buffer, sizeof(buffer) - 1, 0);
        if (n < 0) {
            if (errno == ETERM) break;
            continue;                                   // timeout o interrupción
        }
        if (n > (int)sizeof(buffer) - 1) { g_malformados++; continue; }   // mensaje truncado
        buffer[n] = '\0';

        Mensaje m;
        if (!parsear(buffer, m)) { g_malformados++; continue; }
        m.t_rx = ahora_epoch();
        cola_poner(m);
    }
    zmq_close(sub);
}
