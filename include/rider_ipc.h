#ifndef RIDER_IPC_H
#define RIDER_IPC_H

#include <mqueue.h>
#include <sys/types.h>
#include "rider_protocol.h"

typedef struct {
    mqd_t ordenes, respuestas;
    pid_t pid;
    int estado;
    bool recogido;
    char nombre_ordenes[96], nombre_respuestas[96];
} Puente;

/* 1 enviado/recibido, 0 no disponible, -1 error, -2 mensaje mal formado. */
int rider_enviar(mqd_t cola, const Mensaje *m, unsigned prioridad, long limite);
int rider_recibir(mqd_t cola, Mensaje *m, unsigned *prioridad);
int puente_iniciar(Puente *p, const char *ruta, Mensaje *listo);
/* 1 recogido, 0 sigue vivo, -1 error. Unico llamante durante la ejecucion. */
int puente_comprobar(Puente *p);
/* Siempre cierra y desenlaza sus colas; termina y recoge el hijo si hace falta.
   Devuelve -1 si hubo terminacion forzada, salida anormal o error. */
int puente_finalizar(Puente *p);

#endif
