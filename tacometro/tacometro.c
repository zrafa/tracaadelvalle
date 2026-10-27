/* tacometro.c
 *
 * Aprende dos mesetas (UP y DOWN) de un sensor y detecta
 * transiciones entre ellas. Las medias se adaptan continuamente
 * a la deriva del sensor.
 *
 * Sin dependencia de <stdbool.h>.
 */

#include <stdint.h>

/* ===================== Configuración ===================== */

/* Rango válido. NUNCA poner X_MIN_VALIDO en 0:
 * en un tractor los ecos perdidos dan 0 y hunden media_down. */
#define X_MIN_VALIDO     20
#define X_MAX_VALIDO     3800

/* Separación mínima entre mesetas para considerar calibración válida. */
#define SEP_MINIMA       20

/* Peso de adaptación. Más chico = más resistente a vibración. */
#define ALPHA            0.02

/* Tolerancia alrededor de la media para permitir deriva lenta. */
#define TOL_FRAC         0.10

/* Desvío máximo para aceptar una muestra como "sobre la meseta".
 * Muestras más lejanas (transiciones, outliers) NO entrenan. */
#define DESVIO_MAX_FRAC  0.30

/* Histeresis de detección (fracción de sep). */
#define UMBRAL_UP_FRAC   0.65
#define UMBRAL_DN_FRAC   0.35

/* ===================== Estado ===================== */

static double   media_up   = 0.0;
static double   media_down = 0.0;
static uint8_t  inicializado = 0;
static uint8_t  calibrada    = 0;

uint8_t         estado = 0;              /* 0 = DOWN, 1 = UP */
static uint16_t confirmaciones = 0;

/* ===================== Helpers ===================== */

static uint8_t x_valido(uint16_t x)
{
    return (uint8_t)((x >= X_MIN_VALIDO) && (x <= X_MAX_VALIDO));
}

/* Filtro de mediana de 3. Llamar UNA vez por muestra cruda,
 * antes de calibrar() y detectar_taco(). */
uint16_t tacometro_filtrar(uint16_t x_raw)
{
    static uint16_t buf[3] = {0, 0, 0};
    static uint8_t  idx = 0;
    static uint8_t  n   = 0;

    buf[idx] = x_raw;
    idx = (uint8_t)((idx + 1) % 3);
    if (n < 3) n++;
    if (n < 3) return x_raw;

    {
        uint16_t a = buf[0], b = buf[1], c = buf[2];
        uint16_t t;
        if (a > b) { t = a; a = b; b = t; }
        if (b > c) { t = b; b = c; c = t; }
        if (a > b) { t = a; a = b; b = t; }
        return b;
    }
}

/* ===================== API ===================== */

void calibrar(uint16_t x)
{
    if (!x_valido(x)) return;

    if (!inicializado) {
        media_up = media_down = (double)x;
        inicializado = 1;
        return;
    }

    if (!calibrada) {
        if ((double)x > media_up)
            media_up += ALPHA * ((double)x - media_up);
        else if ((double)x < media_down)
            media_down += ALPHA * ((double)x - media_down);

        if ((media_up - media_down) >= (double)SEP_MINIMA)
            calibrada = 1;
        return;
    }

    {
        double sep = media_up - media_down;
        double tol, desvio_max;

        /* Guardia anti-colapso: si las medias se juntaron, reiniciar. */
        if (sep < (double)SEP_MINIMA) {
            media_up = media_down = (double)x;
            calibrada = 0;
            confirmaciones = 0;
            return;
        }

        tol        = sep * TOL_FRAC;
        desvio_max = sep * DESVIO_MAX_FRAC;

        /* Solo entrena si la muestra está claramente sobre una meseta.
         * Las muestras de transición (entre las mesetas) se ignoran. */
        if ((double)x >= media_up - tol && (double)x <= media_up + desvio_max) {
            media_up += ALPHA * ((double)x - media_up);
        }
        else if ((double)x <= media_down + tol && (double)x >= media_down - desvio_max) {
            media_down += ALPHA * ((double)x - media_down);
        }
    }
}

uint8_t detectar_taco(uint16_t x, uint16_t muestras_necesarias)
{
    double sep, umbral_up, umbral_dn;
    uint8_t candidato;

    if (!calibrada || !x_valido(x)) return estado;

    sep = media_up - media_down;
    if (sep < (double)SEP_MINIMA) return estado;

    umbral_up = media_down + sep * UMBRAL_UP_FRAC;
    umbral_dn = media_down + sep * UMBRAL_DN_FRAC;

    candidato = estado;

    if (estado == 0 && (double)x >= umbral_up)
        candidato = 1;
    else if (estado == 1 && (double)x <= umbral_dn)
        candidato = 0;

    if (candidato != estado) {
        if (++confirmaciones >= muestras_necesarias) {
            estado = candidato;
            confirmaciones = 0;
        }
    } else {
        confirmaciones = 0;
    }

    return estado;
}

/* Accesores para debug */
double   tacometro_media_up(void)       { return media_up;   }
double   tacometro_media_down(void)     { return media_down; }
uint8_t  tacometro_calibrada(void)      { return calibrada;  }
uint16_t tacometro_confirmaciones(void) { return confirmaciones; }