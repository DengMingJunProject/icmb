
#ifndef __ICMB_H__
#define __ICMB_H__

#include "stdint.h"
#include "stdbool.h"
#include "ican_comm.h"

typedef enum{
	ICMB_READ=1,
	ICMB_WRITE,
	ICMB_READ_BACK,
	ICMB_ACK,
	ICMB_NACK,
	ICMB_SEARCH,
	ICMB_SEARCH_ACK,
}icmb_cmd_t;

typedef enum _icmb_result{
	ICMB_RESULT_OK=0,
	ICMB_RESULT_UDP=0x01,
	ICMB_RESULT_485=0x02,
	ICMB_RESULT_ICAN=0x04,
	ICMB_RESULT_TO=0x08,
}icmb_result_t;

typedef struct _icmb_comm_quality{
	uint32_t 	send_cnt;					//发送计数=写计数+读计数
	uint32_t	write_cnt;					//写计数
	uint32_t	read_cnt;					//读计数
	uint32_t	reply_cnt;					//应答计数，不管是ACK还是NACK
	uint32_t	recv_ack_cnt;				//接收的ACK计数
	uint32_t	send_ack_cnt;				//发送的ACK计数
}icmb_comm_quality_t;

#define DEF_ICMB_SRC_LIST(__NAME,...)   \
	typedef struct __NAME __NAME;\
		__VA_ARGS__\
		struct __NAME{\
			
#define END_DEF_ICMB_SRC_LIST(__NAME)   \
		}; ALIGN(1)

typedef struct _icmb_user_cb_para{
	icmb_cmd_t cmd;						//用户回调的命令
	uint8_t addr;						//用户回调的地址
	uint16_t reg_addr;					//用户回调的寄存器地址
	uint16_t len; 						//用户回调的读写长度
	uint8_t *ptr;						//用户回调的数据指针
}icmb_user_cb_para_t;
		
typedef	rt_err_t (*ICMB_USER_CALLBACK_T)(icmb_user_cb_para_t *cb_para);
		
#define DEF_ICMB_USER_CB(__NAME)\
		rt_err_t __NAME(icmb_user_cb_para_t *cb_para){\

#define END_DEF_ICMB_USER_CB(__NAME)\
		}
		
#define DEF_ICMB_ICAN_FILTER()\
		const struct rt_can_filter_item filter_items[] = \
		{
			
#define END_DEF_ICMB_ICAN_FILTER()\
		};\
const uint8_t filter_cnt = (sizeof(filter_items)/sizeof(struct rt_can_filter_item));

#define		icmb_data()		(sizeof(icmb_protocol_t)-5)
#define 	icmb_len(l)		(sizeof(icmb_protocol_t)+l-1)
#define		icmb_crc(l)		(sizeof(icmb_protocol_t)+l-5)
#define		icmb_headlen()	(sizeof(icmb_protocol_t)-5)

typedef enum{
	ICMB_UDP = 0x01,
	ICMB_485 = 0x02,
	ICMB_ICAN = 0x04,
}icmb_comm_t;

#define		ICMB_SELF_ID		1
#define		ICMB_COMM_TYPE		ICMB_ICAN
#define		ICMB_ADD_SRC_ID		1				//例如485，没有底层协议，需添加源地址

#define		ICMB_ICAN_DEV		"can1"
#define		ICMB_ICAN_BR_A		CAN100kBaud
#define 	ICMB_ICAN_BR_D		CAN1MBaud
#define		ICMB_ICAN_ACK_TIME	10				//ican等待接收方返回ACK时间
#define		ICMB_ICAN_RECV_LEN	256

#define		ICMB_UDP_IP_PREFIX	"192.168.1."
#define 	ICMB_UDP_PORT		9527
#define		ICMB_UDP_RECV_LEN	256

#define 	ICMB_UART_NAME      "uart0"   			 /* 串口设备名称 */
#define 	ICMB_UART_BAUD_RATE	 		BAUD_RATE_9600        //修改波特率为 9600
#define 	ICMB_UART_DATA_BITS			DATA_BITS_8           //数据位 8
#define 	ICMB_UART_STOP_BITS			STOP_BITS_1           //停止位 1
#define		ICMB_UART_BUFSZ				128                   //修改缓冲区 buff size 为 128
#define 	ICMB_UART_PARITY			PARITY_NONE           //无奇偶校验位
#define		ICMB_UART_RECV_LEN	256

extern void icmb_rosrc_register(void *src_list, uint32_t len);								//注册只读资源表
extern void icmb_rwsrc_register(void *src_list, uint32_t len);								//注册只写资源表
extern void icmb_usercb_register(ICMB_USER_CALLBACK_T user_cb);								//注册回调函数
extern icmb_result_t icmb_write(uint8_t addr, uint16_t reg_addr, void *buf, uint16_t len);	//写目的地址资源表
extern icmb_result_t icmb_read(uint8_t addr, uint16_t reg_addr, uint16_t len);				//读目的地址资源表
extern icmb_result_t icmb_search(uint8_t addr);												//寻找目标址
extern void icmb_485_txrx_cb(void (*tx)(void), void (*rx)(void));							//注册485的发送接收使能函数
extern icmb_comm_quality_t * icmb_comm_quality(void);										//返回icmb通讯质量参数
#endif
