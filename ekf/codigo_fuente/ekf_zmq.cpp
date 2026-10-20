// ============================================================================
//  EKF para un móvil terrestre (modelo de bicicleta) con tinyekf.h  -- VERSIÓN 3
//  Eventos asincrónicos por ZeroMQ, tiempo real, dt dinámico, datos fuera de orden.
//
//  ENTRADAS: LO QUE HAY QUE MANDARLE AL EKF (detalle en protocolo.h)
//    Mensaje zmq de texto: "<topico> <timestamp> <valores...>"
//    timestamp = instante de la MEDICION [s], double (epoch). Ejes ENU: x = este, y = norte.
//      giro   t delta    delta: giro de ruedas delanteras [rad], 0 = rectas, + = izquierda
//      vel    t v        v: velocidad [m/s]
//      rumbo  t psi      psi: rumbo del INS [rad], 0 = ESTE, antihorario +, rango libre
//      pos    t x y      x, y: posicion [m]
//
//  --- DESDE ACA: REPRESENTACION INTERNA DEL FILTRO (no es lo que se le manda) ---
//  ESTADO        x = [ x, y, psi, v ]
//                x, y : centro del eje trasero, ENU local        [m]
//                psi  : rumbo interno, 0 = Este, antihorario +   [rad]
//                v    : velocidad longitudinal del centro del eje [m/s]
//
//  ENTRADA       delta : giro de las ruedas delanteras (llega como evento "giro")
//
//  MEDICIONES    (cada una llega como un evento con su timestamp)
//                pos (x,y) ~ cada 5-10 s, con retraso | vel ~ 5-10 Hz | rumbo (INS) ~ 20 Hz
//
//  ARQUITECTURA
//
//     [zmq SUB] -> hilo_receptor ---> COLA (mutex + condition_variable) ---> hilo principal (EKF)
//                  parsea y encola     FIFO de llegada, no bloquea           consume y filtra
//
//  QUÉ CAMBIA RESPECTO DE LA VERSIÓN 2
//   1. Ya no hay un lazo a paso fijo: cada EVENTO dispara "predecir hasta su timestamp
//      + actualizar con ese sensor". dt = t_evento - t_evento_anterior (dinámico).
//   2. Hilo receptor + cola protegida. El EKF duerme hasta que llega algo (cv.wait).
//   3. DATOS FUERA DE ORDEN (ej. la posición llega 1-2 s tarde con timestamp viejo):
//      se usa "retroceder y rehacer" (rollback + replay):
//        - el filtro guarda un HISTORIAL de eventos ordenados por timestamp (últimos
//          HISTORIA_S segundos) y, junto a cada uno, una copia del estado (x, P) tras procesarlo.
//        - si llega un evento con timestamp más viejo que el último procesado, se lo inserta
//          en su lugar, se restaura el estado previo a ese punto y se reprocesan los eventos
//          posteriores.
//      Ventaja: el EKF responde AL INSTANTE a cada dato nuevo (tiempo real), sin esperar 4 s
//      "por las dudas", y cuando llega un dato tardío el pasado se corrige.
//   4. Se quitó el magnetómetro: el rumbo viene solo del INS, con la MISMA convención que psi
//      (0 = este, antihorario +), así que entra directo al filtro (solo se normaliza a [-pi,pi]).
//   5. El sensor de posición entrega x,y ENU directamente (ya no hay lat/lon).
//
//  Compilar:  g++ -O2 -std=c++17 -Wall -pthread ekf_zmq.cpp -o ekf_zmq -lzmq
//  Usar:      ./ekf_zmq                      (corre hasta Ctrl-C)
//             ./ekf_zmq --fin-por-silencio   (termina solo si no llegan datos por 3 s; para pruebas)
// ============================================================================

#include <zmq.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <errno.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

#define _float_t double   // tinyekf usa float por defecto; acá usamos double
#define EKF_N 4           // tamaño del estado
#define EKF_M 4           // filas de medición: x, y, v, psi
#include "tinyekf.h"

#include "protocolo.h"    // PI, L, Y_RUEDA, ANTENA_*, SIGMA_*, puertos, wrap_pi, rumbo_a_psi

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

// ============================================================================
//  Mensajes y COLA  (receptor -> EKF)
// ============================================================================
enum Tipo { T_GIRO = 0, T_VEL, T_RUMBO, T_POS, T_GT, T_N };
static const char *NOMBRE_TIPO[T_N] = { "giro", "vel", "rumbo", "pos", "gt" };

struct Mensaje {
    Tipo   tipo;
    double t;              // timestamp del dato (cuándo se midió)
    double a, b, c, d;     // valores (según el tipo, ver protocolo.h)
    double t_rx;           // cuándo lo recibimos (solo para estadísticas de retraso)
};

static std::mutex              g_mx;
static std::condition_variable g_cv;
static std::deque<Mensaje>     g_cola;
static size_t                  g_cola_max = 0;       // máxima ocupación vista
static size_t                  g_cola_perdidos = 0;  // mensajes descartados por cola llena
static std::atomic<long>       g_malformados{0};
static std::atomic<bool>       g_salir{false};

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
static void cola_sacar_todo(std::vector<Mensaje> &out, int timeout_ms)
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

static void hilo_receptor(void *ctx)
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

// ============================================================================
//  El filtro (casi igual a la versión 2)
// ============================================================================
static ekf_t kf;   // kf.x = estado (4),  kf.P = covarianza (4x4).  Es la copia de TRABAJO.

// Instantánea completa del filtro en un instante: lo que se guarda en el historial.
struct Estado {
    ekf_t  kf;
    double delta;   // último giro de ruedas conocido (se mantiene constante entre muestras)
    double t;       // instante al que corresponde el estado
};

static Estado estado_inicial(double t)
{
    // tinyekf pide las VARIANZAS de la diagonal de P
    double pdiag[EKF_N] = { ARR_STD_XY * ARR_STD_XY, ARR_STD_XY * ARR_STD_XY,
                            ARR_STD_PSI * ARR_STD_PSI, ARR_STD_V * ARR_STD_V };
    Estado e;
    ekf_initialize(&e.kf, pdiag);   // x = 0
    e.delta = 0.0;
    e.t = t;
    return e;
}

// ---- PREDICCIÓN con dt variable ----
static void filtro_predecir(double delta, double dt)
{
    double psi = kf.x[2];
    double v   = kf.x[3];
    double w   = tan(delta) / L;          // curvatura: psi' = v * tan(delta) / L

    double fx[EKF_N];
    fx[0] = kf.x[0] + v * cos(psi) * dt;
    fx[1] = kf.x[1] + v * sin(psi) * dt;
    fx[2] = wrap_pi(psi + v * w * dt);
    fx[3] = v;                            // velocidad constante

    double F[EKF_N * EKF_N] = {0};
    F[0*EKF_N + 0] = 1;  F[0*EKF_N + 2] = -v * sin(psi) * dt;  F[0*EKF_N + 3] = cos(psi) * dt;
    F[1*EKF_N + 1] = 1;  F[1*EKF_N + 2] =  v * cos(psi) * dt;  F[1*EKF_N + 3] = sin(psi) * dt;
    F[2*EKF_N + 2] = 1;                                         F[2*EKF_N + 3] = w * dt;
    F[3*EKF_N + 3] = 1;

    // Ruido de proceso. Posición y velocidad: ruido continuo => varianza proporcional a dt.
    double Q[EKF_N * EKF_N] = {0};
    double c = cos(delta);
    double dpsi_ddelta = v * dt / (L * c * c);       // cuánto cambia psi si delta se equivoca
    Q[0*EKF_N + 0] = RUIDO_POS * RUIDO_POS * dt;
    Q[1*EKF_N + 1] = RUIDO_POS * RUIDO_POS * dt;
    Q[2*EKF_N + 2] = dpsi_ddelta * dpsi_ddelta * SIGMA_DELTA * SIGMA_DELTA + RUIDO_PSI * RUIDO_PSI * dt;
    Q[3*EKF_N + 3] = RUIDO_V * RUIDO_V * dt;

    ekf_predict(&kf, fx, F, Q);
}

// ---- Armado de las mediciones de UN evento ----
static double g_z [EKF_M];
static double g_hx[EKF_M];
static double g_H [EKF_M * EKF_N];
static double g_R [EKF_M * EKF_M];

// Deja todas las filas como "sin dato": H=0, z=hx=0, R=1
static void medidas_limpiar(void)
{
    memset(g_z,  0, sizeof g_z);
    memset(g_hx, 0, sizeof g_hx);
    memset(g_H,  0, sizeof g_H);
    memset(g_R,  0, sizeof g_R);
    for (int i = 0; i < EKF_M; i++) g_R[i*EKF_M + i] = 1.0;
}

// Agrega la fila i. Devuelve false si la compuerta la rechazó (la fila queda "sin dato").
static bool medida_agregar(int i, double z_i, double hx_i, const double Hfila[EKF_N], double r, double compuerta)
{
    double innov = z_i - hx_i;

    double S = r;                         // S = H P H' + R
    for (int a = 0; a < EKF_N; a++)
        for (int b = 0; b < EKF_N; b++)
            S += Hfila[a] * kf.P[a*EKF_N + b] * Hfila[b];

    if (compuerta > 0.0 && innov * innov > compuerta * S)
        return false;

    g_z[i]  = z_i;
    g_hx[i] = hx_i;
    for (int j = 0; j < EKF_N; j++) g_H[i*EKF_N + j] = Hfila[j];
    g_R[i*EKF_M + i] = r;
    return true;
}

// Igual, para un ÁNGULO: la innovación se lleva a [-pi, pi].
static bool medida_agregar_angulo(int i, double z_i, double hx_i, const double Hfila[EKF_N], double r, double compuerta)
{
    double hx_ajustado = z_i - wrap_pi(z_i - hx_i);
    return medida_agregar(i, z_i, hx_ajustado, Hfila, r, compuerta);
}

// Corre ekf_update con lo que haya en g_*. Si ninguna fila fue aceptada, no hace nada.
static void medidas_aplicar(bool alguna)
{
    if (!alguna) return;
    if (ekf_update(&kf, g_z, g_hx, g_H, g_R)) {
        kf.x[2] = wrap_pi(kf.x[2]);
        // simetrizar P: al repetir muchos updates/replays evita que se acumule asimetría numérica
        for (int i = 0; i < EKF_N; i++)
            for (int j = i + 1; j < EKF_N; j++) {
                double p = 0.5 * (kf.P[i*EKF_N + j] + kf.P[j*EKF_N + i]);
                kf.P[i*EKF_N + j] = kf.P[j*EKF_N + i] = p;
            }
    }
}

// Los cuatro updates. Devuelven true si TODAS las filas del evento fueron aceptadas.

// Posición: el sensor mide la posición de un punto desplazado (ANTENA_X/Y) del centro del eje.
static bool actualizar_pos(double zx, double zy)
{
    medidas_limpiar();
    double cp = cos(kf.x[2]), sp = sin(kf.x[2]);
    double hx_gx = kf.x[0] + ANTENA_X * cp - ANTENA_Y * sp;
    double hx_gy = kf.x[1] + ANTENA_X * sp + ANTENA_Y * cp;
    double Hgx[EKF_N] = { 1, 0, -ANTENA_X * sp - ANTENA_Y * cp, 0 };
    double Hgy[EKF_N] = { 0, 1,  ANTENA_X * cp - ANTENA_Y * sp, 0 };
    bool ok_x = medida_agregar(Z_X, zx, hx_gx, Hgx, SIGMA_POS * SIGMA_POS, COMPUERTA);
    bool ok_y = medida_agregar(Z_Y, zy, hx_gy, Hgy, SIGMA_POS * SIGMA_POS, COMPUERTA);
    medidas_aplicar(ok_x || ok_y);
    return ok_x && ok_y;
}

// Velocidad: la rueda está desplazada del centro del eje; en curva mide v * (1 - Y_RUEDA * tan(delta) / L)
static bool actualizar_vel(double zv, double delta)
{
    medidas_limpiar();
    double f = 1.0 - Y_RUEDA * tan(delta) / L;
    double Hv[EKF_N] = { 0, 0, 0, f };
    bool ok = medida_agregar(Z_V, zv, f * kf.x[3], Hv, SIGMA_VW * SIGMA_VW, COMPUERTA);
    medidas_aplicar(ok);
    return ok;
}

// Rumbo del INS (ya convertido a psi interno)
static bool actualizar_rumbo(double zpsi)
{
    medidas_limpiar();
    double Hpsi[EKF_N] = { 0, 0, 1, 0 };
    bool ok = medida_agregar_angulo(Z_PSI, zpsi, kf.x[2], Hpsi, SIGMA_INS * SIGMA_INS, COMPUERTA_RUMBO);
    medidas_aplicar(ok);
    return ok;
}

// ============================================================================
//  HISTORIAL de eventos ordenados por timestamp + retroceder-y-rehacer
// ============================================================================
struct Evento {
    Mensaje m;
    Estado  despues;      // estado del filtro justo después de procesar este evento
    bool    rechazado;    // la compuerta lo rechazó (total o parcialmente)
};

static std::deque<Evento> g_hist;      // ordenado por m.t (a igual t, por orden de llegada)
static Estado             g_base;      // estado justo ANTES de g_hist[0]
static bool               g_iniciado = false;

// Estadísticas
static long   g_n_evt[T_N]        = {0};
static double g_retraso_max[T_N]  = {0};
static long   g_fuera_de_orden    = 0;   // eventos que obligaron a retroceder
static long   g_ejecuciones       = 0;   // total de "predecir+actualizar" (incluye replays)
static size_t g_replay_max        = 0;   // mayor cantidad de eventos rehechos de una vez
static long   g_descartados_viejos = 0;  // llegaron más viejos que la ventana
static long   g_rechazos[T_N]     = {0}; // rechazos de la compuerta por tipo (contados al salir del historial)
static double g_retroceso_max_s   = 0;   // cuánto hubo que retroceder en el peor caso [s]
static long   g_tardios_100ms     = 0;   // eventos que llegaron con más de 100 ms de atraso respecto del último

// "Listo" = ya llegó al menos un dato de pos, vel y rumbo
static double g_t_primero[T_N] = { -1, -1, -1, -1, -1 };
static bool   g_listo   = false;
static double g_t_listo = 0;
static double g_t_inicio = 0;

// ---- Salida opcional de estados (para usar el EKF sin ground truth) ----
static const bool MOSTRAR_ESTADOS = true;  // true = imprime estado tras la predicción y tras la actualización
static bool g_imprimir = false;             // lo activa reprocesar_desde SOLO para el evento recién llegado
                                            // (los eventos que se rehacen por un dato tardío no se imprimen)

// Imprime el estado de trabajo (kf) en la etapa indicada: "pred" = tras predecir, "upd" = tras actualizar.
static void mostrar_estado(const char *etapa, const Evento &e)
{
    char dato[48];   // valor que llegó del sensor (giro, vel y rumbo: 1 valor; pos: x, y)
    if (e.m.tipo == T_POS) snprintf(dato, sizeof dato, "%.2f, %.2f", e.m.a, e.m.b);
    else                   snprintf(dato, sizeof dato, "%.3f", e.m.a);

    printf("t=%8.3f %-5s %-4s dato=[%s] x=%8.2f y=%8.2f psi=%7.3f v=%6.2f | std: x=%.2f y=%.2f psi=%.3f v=%.2f%s\n",
           e.m.t - g_t_inicio, NOMBRE_TIPO[e.m.tipo], etapa, dato,
           kf.x[0], kf.x[1], kf.x[2], kf.x[3],
           sqrt(kf.P[0*EKF_N + 0]), sqrt(kf.P[1*EKF_N + 1]), sqrt(kf.P[2*EKF_N + 2]), sqrt(kf.P[3*EKF_N + 3]),
           (!strcmp(etapa, "upd") && e.rechazado) ? "  [RECHAZADO]" : "");
}

// Procesa UN evento a partir del estado previo: predice hasta su timestamp (dt dinámico) y actualiza.
static void ejecutar_evento(Evento &e, const Estado &previo)
{
    kf = previo.kf;
    double delta = previo.delta;
    double dt = e.m.t - previo.t;
    if (dt > 0.0) filtro_predecir(delta, dt);
    if (MOSTRAR_ESTADOS && g_imprimir) mostrar_estado("pred", e);

    bool ok = true;
    switch (e.m.tipo) {
    case T_GIRO:  delta = e.m.a;                                            break;  // entrada de control
    case T_VEL:   ok = actualizar_vel(e.m.a, delta);                        break;
    case T_RUMBO: ok = actualizar_rumbo(rumbo_a_psi(e.m.a));               break;
    case T_POS:   ok = actualizar_pos(e.m.a, e.m.b);                        break;
    default: break;
    }

    e.rechazado = !ok;
    if (MOSTRAR_ESTADOS && g_imprimir && e.m.tipo != T_GIRO) mostrar_estado("upd", e);
    e.despues.kf    = kf;
    e.despues.delta = delta;
    e.despues.t     = (dt > 0.0) ? e.m.t : previo.t;   // el tiempo del estado nunca retrocede
    g_ejecuciones++;
}

// Rehace los eventos g_hist[i0 .. fin], cada uno partiendo del snapshot del anterior.
static void reprocesar_desde(size_t i0)
{
    for (size_t i = i0; i < g_hist.size(); i++) {
        const Estado &previo = (i == 0) ? g_base : g_hist[i - 1].despues;
        g_imprimir = (i == i0);          // solo el evento nuevo; los rehechos van en silencio
        ejecutar_evento(g_hist[i], previo);
    }
    g_imprimir = false;
    size_t n = g_hist.size() - i0;
    if (n > g_replay_max) g_replay_max = n;
}

// Los eventos más viejos que la ventana ya no se pueden corregir: el último pasa a ser la nueva base.
static void retirar_viejos(void)
{
    double t_max = g_hist.back().m.t;
    while (!g_hist.empty() && g_hist.front().m.t < t_max - HISTORIA_S) {
        g_base = g_hist.front().despues;
        if (g_hist.front().rechazado) g_rechazos[g_hist.front().m.tipo]++;
        g_hist.pop_front();
    }
}

// Estimación en un instante cualquiera tq: último snapshot con t <= tq, predicho hasta tq.
// (solo para evaluar; no modifica el historial)
static void estimado_en(double tq, double out[EKF_N])
{
    const Estado *s = &g_base;
    for (size_t i = 0; i < g_hist.size() && g_hist[i].m.t <= tq; i++) s = &g_hist[i].despues;
    kf = s->kf;
    double dt = tq - s->t;
    if (dt > 0.0) filtro_predecir(s->delta, dt);
    memcpy(out, kf.x, EKF_N * sizeof(double));
}

// ============================================================================
//  Evaluación contra el ground truth (solo pruebas)
// ============================================================================
static double g_e2_pos = 0, g_e2_psi = 0, g_e2_v = 0;
static long   g_n_eval = 0;
static double g_t_print = -1e18;

static void evaluar_gt(const Mensaje &m)
{
    if (!g_iniciado || !g_listo) return;

    double est[EKF_N];
    estimado_en(m.t, est);                          // lo que el EKF "sabe" ahora, llevado a t_gt
    double ex = est[0] - m.a, ey = est[1] - m.b;
    double epsi = wrap_pi(est[2] - rumbo_a_psi(m.c));
    double ev = est[3] - m.d;

    if (m.t - g_t_listo >= CONVERGENCIA_S) {
        g_e2_pos += ex * ex + ey * ey;
        g_e2_psi += epsi * epsi;
        g_e2_v   += ev * ev;
        g_n_eval++;

        if (m.t - g_t_print >= IMPRIMIR_CADA_S) {
            if (g_t_print < -1e17)
                printf("\n   t[s]   x_real   y_real  psi_real  v_real |   x_est    y_est  psi_est   v_est | err_pos[m]\n");
            g_t_print = m.t;
            printf("%7.1f %8.2f %8.2f %9.3f %7.2f | %8.2f %8.2f %8.3f %7.2f | %7.2f\n",
                   m.t - g_t_inicio, m.a, m.b, rumbo_a_psi(m.c), m.d,
                   est[0], est[1], est[2], est[3], sqrt(ex * ex + ey * ey));
            fflush(stdout);
        }
    }
}

// ============================================================================
//  Un mensaje nuevo entra al EKF
// ============================================================================
static void procesar_mensaje(const Mensaje &m)
{
    if (m.tipo == T_GT) { evaluar_gt(m); return; }

    if (!g_iniciado) {
        g_base = estado_inicial(m.t - MARGEN_INICIO_S);
        g_t_inicio = m.t;
        g_iniciado = true;
    }

    if (m.t < g_base.t) {              // más viejo que todo lo que podemos rehacer
        g_descartados_viejos++;
        return;
    }

    // Estadísticas
    g_n_evt[m.tipo]++;
    double retraso = m.t_rx - m.t;
    if (retraso > g_retraso_max[m.tipo]) g_retraso_max[m.tipo] = retraso;
    if (g_t_primero[m.tipo] < 0) g_t_primero[m.tipo] = m.t;
    if (!g_listo && g_t_primero[T_POS] >= 0 && g_t_primero[T_VEL] >= 0 && g_t_primero[T_RUMBO] >= 0) {
        g_listo = true;
        g_t_listo = std::max(g_t_primero[T_POS], std::max(g_t_primero[T_VEL], g_t_primero[T_RUMBO]));
    }

    // Se inserta en su lugar según el timestamp (después de los que tienen igual t).
    Evento e;
    e.m = m;
    e.rechazado = false;
    auto it = std::upper_bound(g_hist.begin(), g_hist.end(), m.t,
                               [](double t, const Evento &ev) { return t < ev.m.t; });
    size_t idx = it - g_hist.begin();
    g_hist.insert(it, e);

    if (idx != g_hist.size() - 1) {                     // llegó tarde: retroceder y rehacer
        g_fuera_de_orden++;
        double atraso = g_hist.back().m.t - m.t;
        if (atraso > g_retroceso_max_s) g_retroceso_max_s = atraso;
        if (atraso > 0.1) g_tardios_100ms++;
    }
    reprocesar_desde(idx);                              // camino normal: idx = último -> 1 sola ejecución
    retirar_viejos();
}

// ============================================================================
//  Main: hilo principal = EKF
// ============================================================================
static void manejador(int) { g_salir = true; }

static void imprimir_resumen(void)
{
    long rechazos_tipo[T_N];
    long rechazos = 0;
    for (int i = 0; i < T_N; i++) rechazos_tipo[i] = g_rechazos[i];
    for (const Evento &e : g_hist) if (e.rechazado) rechazos_tipo[e.m.tipo]++;
    for (int i = 0; i < T_N; i++) rechazos += rechazos_tipo[i];

    printf("\n================ RESUMEN ================\n");
    printf("Eventos procesados:");
    for (int i = 0; i < T_GT; i++) printf(" %s=%ld", NOMBRE_TIPO[i], g_n_evt[i]);
    printf("\nEventos fuera de orden (obligaron a retroceder): %ld, de los cuales %ld con más de 100 ms de atraso\n",
           g_fuera_de_orden, g_tardios_100ms);
    printf("Mayor retroceso: %.3f s | replay máx: %zu eventos | ejecuciones totales (con replays): %ld\n",
           g_retroceso_max_s, g_replay_max, g_ejecuciones);
    printf("Retraso máx. observado (recepción - timestamp) [s]:");
    for (int i = 0; i < T_GT; i++) printf(" %s=%.3f", NOMBRE_TIPO[i], g_retraso_max[i]);
    printf("\nDescartados por demasiado viejos: %ld | malformados: %ld\n", g_descartados_viejos, g_malformados.load());
    printf("Rechazados por compuerta: %ld  (", rechazos);
    for (int i = 0; i < T_GT; i++) printf("%s%s=%ld", i ? " " : "", NOMBRE_TIPO[i], rechazos_tipo[i]);
    printf(")\n");
    printf("Cola: máx. ocupación = %zu / %zu | perdidos por cola llena = %zu\n",
           g_cola_max, CAP_COLA, g_cola_perdidos);
    if (g_n_eval > 0)
        printf("Error RMS vs ground truth (%ld muestras):  posición = %.2f m | rumbo = %.3f rad | velocidad = %.3f m/s\n",
               g_n_eval, sqrt(g_e2_pos / g_n_eval), sqrt(g_e2_psi / g_n_eval), sqrt(g_e2_v / g_n_eval));
    else
        printf("(sin ground truth suficiente para calcular errores)\n");
}

int main(int argc, char **argv)
{
    bool fin_por_silencio = (argc > 1 && !strcmp(argv[1], "--fin-por-silencio"));

    signal(SIGINT,  manejador);
    signal(SIGTERM, manejador);

    void *ctx = zmq_ctx_new();
    std::thread receptor(hilo_receptor, ctx);

    printf("EKF esperando eventos por zmq (HISTORIA_S = %.1f s, compuerta = %.1f)...\n", HISTORIA_S, COMPUERTA);
    fflush(stdout);

    using reloj = std::chrono::steady_clock;
    auto t_ultimo_dato = reloj::now();
    bool hubo_datos = false;
    std::vector<Mensaje> lote;

    while (!g_salir) {
        lote.clear();
        cola_sacar_todo(lote, 100);         // duerme hasta que llegue algo

        if (lote.empty()) {
            double callado = std::chrono::duration<double>(reloj::now() - t_ultimo_dato).count();
            if (fin_por_silencio && hubo_datos && callado > SILENCIO_FIN_S) break;
            continue;
        }
        hubo_datos = true;
        t_ultimo_dato = reloj::now();

        // Si varios eventos se acumularon mientras el EKF estaba ocupado, ordenarlos por timestamp
        // antes de procesarlos ahorra retrocesos (el sort es estable: respeta el orden de llegada).
        std::stable_sort(lote.begin(), lote.end(),
                         [](const Mensaje &a, const Mensaje &b) { return a.t < b.t; });

        for (const Mensaje &m : lote) procesar_mensaje(m);
    }

    g_salir = true;
    receptor.join();
    zmq_ctx_term(ctx);
    imprimir_resumen();
    return 0;
}
