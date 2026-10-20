
trac..caa ... trac..caaa ... tracaa ... tracaa taca taca tacatacatatata....

(Sonido mecánico irregular entrecortado, producido, al arrancar en frío
por un motor diesel de un tractor antiguo del alto valle.)

Tractores Antiguos Autónomos (tracaa) del Alto Valle
====================================================

Equipamiento y sistemas de navegación autónoma para tractores antiguos del Alto Valle.

En este repositorio se encuentra el código fuente de todas las capas
de software que componen la herramienta experimental desarrollada
durante el doctorado, para una solución de navegación autónoma futura
en tractores antiguos del Alto Valle de Río Negro y Neuquen.

Varios sub proyectos
--------------------

caja\_adquisidora: drivers y software de adquisición de datos desde los sensores.

lidar\_nuevo: driver para el LIDAR de bajo costo.

tacometro: sistema embebido que reporta velocidad (m/s) en base a un ultrasonido sobre rueda trasera.

navegacion\_inercial: INS que utiliza los datos de la IMU (acelerometros, girocopios, magnetómetro) y reporta la pose relativa.


Para Adquirir a la maxima frecuencia utilizando todos los sensores:

./adquirir.sh  /DIRECTORIO/DESTINO/

El código fuente para el control de los distintos sensores (drivers,
software de adquisición, filtrado, calibración, documentación) de la
herramienta experimental en construcción están en 
caja\_adquisidora/
