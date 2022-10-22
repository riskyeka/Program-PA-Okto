#ifndef __RTDB_COMM_H
#define __RTDB_COMM_H

#include "rtdb_api.h"

int DB_comm_put (int _agent, int _id, int _size, void *_value, int life);
int DB_comm_init(RTDBconf_var *rec);
#endif