#ifndef __RTDBDEFS_H
#define __RTDBDEFS_H

// #define DEBUG

#define SELF 0

#define CONFIG_FILE	"/home/hendro/COMM_workspace/src/ersow_comm/src/rtdb/config/rtdb.ini"
#define SHMEM_KEY 0x2000
#define SHMEM_SECOND_TEAM_KEY 0x3000

#define MAX_AGENTS 4	// max number of agents (basestation included)
#define MAX_RECS 100	// max number of item (shared + local)


typedef struct
{
	unsigned char id;				
	unsigned char size;			
	unsigned char period;	
} RTDBconf_var;

#endif