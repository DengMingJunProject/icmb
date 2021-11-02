#ifndef __ICMB_SAMPLE_H__
#define __ICMB_SAMPLE_H__

#include "stdint.h"
#include "stdbool.h"
#include "misc.h"
#include "icmb.h"

//pecu 资源表
typedef struct _pecu_device_info{
	uint8_t		sn[10];
	uint8_t		product[20];
	uint8_t		date_of_manufacture[20];
	uint8_t		data[100];
}pecu_device_info_t;

typedef struct _pecu_work_info{
	uint8_t		state;
	uint8_t		error;
	uint8_t		time;
}pecu_work_info_t;

typedef struct _pecu_work_set{
	uint8_t		mode;
}pecu_work_set_t;


DEF_ICMB_SRC_LIST( pecu_src_list_t )
	pecu_device_info_t	dev_info;
	pecu_work_info_t	work_info;
	pecu_work_set_t		work_set;
END_DEF_ICMB_SRC_LIST( pecu_src_list_t )



//mvb 资源表
typedef struct _mvb_device_info{
	uint8_t		sn[10];
	uint8_t		product[20];
	uint8_t		date_of_manufacture[20];
}mvb_device_info_t;

DEF_ICMB_SRC_LIST( mvb_src_list_t )
	mvb_device_info_t	mvb_device_info;
END_DEF_ICMB_SRC_LIST( mvb_src_list_t )

#endif
