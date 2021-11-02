/** 
* @file         icmb.c 
* @brief        基于ican、udp及485的智能机箱通信总线. 
* @details  	第一步先完成基于ican的can总线通讯. 
* @author       ken deng 
* @date     	2020-10-13 
* @version  	A001 
* @par Copyright (c):  
*       GlobalLink 广州国联通信有限公司 
* @par History:          
*   version: ken deng, 2020-10-13, 建立\n 
*			 key deng, 2020-11-13, 增加寻找地址功能
*/ 
#include "icmb.h"
#include "rtthread.h"
#include "lwip/sockets.h"
#include "lwip/ip4_addr.h"
#include "string.h"
#include "ican_comm.h"

#define LOG_TAG             "icmb"
#include <ulog.h>

typedef struct _icmb_udp_port{
	int		socket;
	char	*ip_prefix;
	
	uint8_t			*recv_buf;				//udp接收数据指针
	uint16_t		recv_len;				//udp接收数据长度
}icmb_udp_port_t;

typedef struct _icmb_485_port{
	rt_device_t 	serial;
	
	void			(*icmb_485_enrx)(void);
	void			(*icmb_485_entx)(void);
	
	uint8_t			*recv_buf;				//串行口接收数据指针
	uint16_t		recv_len;				//串行口接收数据长度
}icmb_485_port_t;

typedef struct _icmb_ican_port{
	ican_comm_t		*ican_comm;				//ican数据结构
	uint32_t		BaudrateA;				//canfd同步速率
	uint32_t		BaudrateD;				//canfd数s据速率
	char			can_dev[5];				//can设备名称
	uint16_t		timeout;				//ican_recvfrom函数等待对方ACK的时间，设为0不等待
	uint8_t			*recv_buf;				//ican接收数据指针
	uint16_t		recv_len;				//ican接收数据长度
}icmb_ican_port_t;

typedef struct _icmb_src_list{
	void			*src_ptr;				//资源表指针
	uint32_t		src_len;				//资源表长度
}icmb_src_list_t;

typedef struct _icmb_recv_msgid{
	uint8_t			addr;					//当前消息ID地址
	uint16_t		msgid;					//保存消息IDs
}icmb_recv_msgid_t;
#define ICMB_MSGID_MAX				10		//最大保存的地址数

typedef struct{
	
	rt_sem_t		 	search_sem;			//寻找地址信号量
	
	icmb_comm_t			comm_type;			//配置需要的通讯类型
	icmb_comm_quality_t	comm_quality;		//通讯质量统计
	
	icmb_udp_port_t		udp_port;			//ican接口
	icmb_485_port_t		rs485_port;			//485接口
	icmb_ican_port_t	ican_port;			//udp接口

	uint16_t			msg_id;				//当前设备发出的消息ID
//	uint16_t			recv_msg_id;
	uint8_t				self_id;			//本地ID
	uint8_t				target_id;			//目标ID
	
	icmb_recv_msgid_t	icmb_recv_msgid[ICMB_MSGID_MAX];
	
	icmb_src_list_t		src_ro_list;		//只读资源表
	icmb_src_list_t		src_wo_list;		//只写资源表
	
	ICMB_USER_CALLBACK_T	icmb_user_cb;	//用户回调函数指针
}icmb_state_t;

typedef struct _icmb_recv_mesg{
	uint8_t		*recv_buf;					//提供给内部回调函数的接收指针
	uint16_t	recv_len;					//提供给内部回调函数的接收长度
	uint8_t		recv_flag;
}icmb_recv_msg_t;

typedef __packed struct _icmb_protocol{
	uint8_t		addr;						//目标设备地址
	#if ICMB_ADD_SRC_ID == 1
	uint8_t		src_addr;					//在485通讯下使用的源地址
	#endif
	uint8_t		cmd;						//控制命令
	uint16_t	msg_id;						//消息ID，用于过滤重复消息
	uint16_t	reg;						//寄存器地址
	uint16_t	len;						//数据长度
	uint8_t 	data;						//数据区(0-n)
	uint32_t	crc32;						//校验
}icmb_protocol_t;

icmb_state_t	icmb_state;

/** 
* 保存指定地址的消息ID. 
* 无.
* @param[in]   addr：查找的地址，msgid：消息ID. 
* @param[out]  无.  
* @retval  无. 
* @par 标识符 
*      保留 
* @par 其它 
*      无 
* @par 修改日志 
*      ken deng于2020-10-27创建 
*/
static void icmb_save_recv_msgid(uint8_t addr, uint16_t msgid)
{
	for( uint8_t i=0; i<ICMB_MSGID_MAX; i++ ){
		if( icmb_state.icmb_recv_msgid[i].addr ==0 || icmb_state.icmb_recv_msgid[i].addr == addr ){
			icmb_state.icmb_recv_msgid[i].addr = addr;
			icmb_state.icmb_recv_msgid[i].msgid = msgid;
			break;
		}
	}
}

/** 
* 获取指定地址的消息ID. 
* 无.
* @param[in]   addr：查找的地址. 
* @param[out]  无.  
* @retval  返回消息ID. 
* @par 标识符 
*      保留 
* @par 其它 
*      无 
* @par 修改日志 
*      ken deng于2020-10-27创建 
*/
static uint16_t icmb_get_recv_msgid(uint8_t addr)
{
	for( uint8_t i=0; i<ICMB_MSGID_MAX; i++ ){
		if( icmb_state.icmb_recv_msgid[i].addr == addr ){
			return icmb_state.icmb_recv_msgid[i].msgid;
		}
	}
	return 0;
}

/** 
* 设置ICMB自身ID. 
* 无.
* @param[in]   id：设置的ID. 
* @param[out]  无.  
* @retval  无. 
* @par 标识符 
*      保留 
* @par 其它 
*      无 
* @par 修改日志 
*      ken deng于2020-10-27创建 
*/
void icmb_set_self_id(uint8_t id)
{
	icmb_state.self_id = id;
}

/** 
* udp初始化. 
* socket申请，udp初始化，申请接收内存缓冲.
* @param[in]   无. 
* @param[out]  无.  
* @retval  无. 
* @par 标识符 
*      保留 
* @par 其它 
*      无 
* @par 修改日志 
*      ken deng于2020-10-27创建 
*/
static rt_err_t icmb_udp_init(void)
{
	
	icmb_udp_port_t		*udp_ptr = &icmb_state.udp_port;
	rt_memset(udp_ptr,0,sizeof(icmb_udp_port_t));
	
	icmb_state.udp_port.ip_prefix = ICMB_UDP_IP_PREFIX;
	
	rt_err_t			result = RT_EOK;
	
	/* 创建一个socket，类型是SOCK_DGRAM，UDP类型 */
    if ((udp_ptr->socket = lwip_socket(AF_INET, SOCK_DGRAM, 0)) == -1)
    {        
        return RT_ERROR;
    }
	
	struct sockaddr_in server_addr;

    /* 初始化服务端地址 */
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(ICMB_UDP_PORT);
    server_addr.sin_addr.s_addr = INADDR_ANY;
	//server_addr.sin_addr.s_addr = inet_addr("192.168.1.216"); 
    rt_memset(&(server_addr.sin_zero), 0, sizeof(server_addr.sin_zero));
     
    /* 绑定socket到服务端地址 */
    if (lwip_bind(udp_ptr->socket, (struct sockaddr *)&server_addr,
             sizeof(struct sockaddr)) == -1)
    {  
        result = RT_ERROR;
    }
	
	if( result == RT_EOK ){
		udp_ptr->recv_buf = rt_malloc(ICMB_ICAN_RECV_LEN);
		
		if( udp_ptr->recv_buf == RT_NULL )
			result = RT_ERROR;
	}
	
	return result;
}

/** 
* udp的icmb读写接口. 
* .
* @param[in]   addr：目标地址， msgid:消息ID,cmd：命令，reg：寄存器地址，buf：缓冲指针，len：读写长度. 
* @param[out]  无.  
* @retval  RT_EOK/RT_ERROR. 
* @par 标识符 
*      保留 
* @par 其它 
*      无 
* @par 修改日志 
*      ken deng于2020-10-27创建 
*/
static rt_err_t icmb_udp_operate(uint8_t addr, uint16_t msgid, icmb_cmd_t cmd, uint16_t reg, void *buf, uint16_t len)
{
	if( icmb_state.udp_port.socket < 0 )
		return RT_ERROR;
	
	struct sockaddr_in remote_addr;
	remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(ICMB_UDP_PORT);
	
	char ip[16]={0};
	snprintf( ip, 16, "%s%d",icmb_state.udp_port.ip_prefix,addr );
	remote_addr.sin_addr.s_addr = inet_addr(ip);
	
	rt_memset(&(remote_addr.sin_zero), 0, sizeof(remote_addr.sin_zero));
	
	void *buf_ptr;
	uint16_t	data_len;
	
	if( cmd == ICMB_WRITE || cmd == ICMB_READ_BACK ){
		buf_ptr = rt_malloc(icmb_len(len));
		data_len = len;
	}
	else{
		buf_ptr = rt_malloc(icmb_len(0));
		data_len = 0;
	}
	
	if( buf_ptr ){
		
		icmb_protocol_t *icmb_ptr = (icmb_protocol_t *)buf_ptr;
		
		icmb_ptr->addr = addr;
		#if ICMB_ADD_SRC_ID == 1
		icmb_ptr->src_addr = ICMB_SELF_ID;
		#endif
		icmb_ptr->cmd = cmd;
		icmb_ptr->msg_id = icmb_state.msg_id;
		icmb_ptr->reg = reg;
		icmb_ptr->len = len;
		
		if( cmd == ICMB_WRITE || cmd == ICMB_READ_BACK )
			rt_memcpy(&icmb_ptr->data,buf,data_len);
		
		uint32_t crc;
		crc = crc32_cal(buf_ptr, icmb_crc(data_len));
		rt_memcpy((uint8_t*)buf_ptr+icmb_crc(data_len), &crc, 4);
		
		rt_err_t result;
		result = lwip_sendto(icmb_state.udp_port.socket, buf_ptr, icmb_len(data_len), 0,(struct sockaddr *)&remote_addr,  sizeof(struct sockaddr));
		
		rt_free(buf_ptr);
		return result;
	}
	else{
		return RT_ERROR;
	}
}

/** 
* 485串口发磅完成回调. 
* .
* @param[in]   dev:设备，buffer:缓冲指针. 
* @param[out]  无.  
* @retval  无. 
* @par 标识符 
*      保留 
* @par 其它 
*      无 
* @par 修改日志 
*      ken deng于2020-10-28创建 
*/
static rt_err_t icmb_485_tx_done(rt_device_t dev,void *buffer)
{
	if( icmb_state.rs485_port.icmb_485_enrx ){
		icmb_state.rs485_port.icmb_485_enrx();
	}
}

/** 
* 485串口初始化. 
* 打开串口，初始化波特率.
* @param[in]   无. 
* @param[out]  无.  
* @retval  无. 
* @par 标识符 
*      保留 
* @par 其它 
*      无 
* @par 修改日志 
*      ken deng于2020-10-28创建 
*/
static rt_err_t icmb_485_init(void)
{
	rt_err_t result = RT_EOK;
	struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */
	/* step1：查找串口设备 */
	char name[10]={0};
	snprintf(name,10,"%s",ICMB_UART_NAME);
	icmb_state.rs485_port.serial = rt_device_find(name);
	
	if( icmb_state.rs485_port.serial == RT_NULL )
		result = RT_ERROR;

	if( result == RT_EOK ){
		/* step2：修改串口配置参数 */
		config.baud_rate = ICMB_UART_BAUD_RATE;        		//修改波特率为 9600
		config.data_bits = ICMB_UART_DATA_BITS;           //数据位 8
		config.stop_bits = ICMB_UART_STOP_BITS;           //停止位 1
		config.bufsz     = ICMB_UART_BUFSZ;                   //修改缓冲区 buff size 为 128
		config.parity    = ICMB_UART_PARITY;           //无奇偶校验位

		/* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
		result = rt_device_control(icmb_state.rs485_port.serial, RT_DEVICE_CTRL_CONFIG, &config);
	}

	if( result == RT_EOK ){
		/* step4：打开串口设备。以中断接收及轮询发送模式打开串口设备 */
		result = rt_device_open(icmb_state.rs485_port.serial, RT_DEVICE_FLAG_INT_RX|RT_DEVICE_FLAG_INT_TX);
	}
	
	if( result == RT_EOK ){
		result = rt_device_set_tx_complete(icmb_state.rs485_port.serial, icmb_485_tx_done);
	}
	
	if( result == RT_EOK ){
		icmb_state.rs485_port.recv_buf = rt_malloc(ICMB_UART_RECV_LEN);
		if( icmb_state.rs485_port.recv_buf == RT_NULL ){
			result = RT_ERROR;
		}
	}
	
	return result;
}

/** 
* 485的icmb读写接口. 
* .
* @param[in]   addr：目标地址， msgid:消息ID,cmd：命令，reg：寄存器地址，buf：缓冲指针，len：读写长度. 
* @param[out]  无.  
* @retval  RT_EOK/RT_ERROR. 
* @par 标识符 
*      保留 
* @par 其它 
*      无 
* @par 修改日志 
*      ken deng于2020-10-28创建 
*/
static rt_err_t icmb_485_operate(uint8_t addr, uint16_t msgid, icmb_cmd_t cmd, uint16_t reg, void *buf, uint16_t len)
{
	if( icmb_state.rs485_port.serial == RT_NULL )
		return RT_ERROR;
	
		void *buf_ptr;
	uint16_t	data_len;
	
	if( cmd == ICMB_WRITE || cmd == ICMB_READ_BACK ){
		buf_ptr = rt_malloc(icmb_len(len));
		data_len = len;
	}
	else{
		buf_ptr = rt_malloc(icmb_len(0));
		data_len = 0;
	}
	
	if( buf_ptr ){
		
		icmb_protocol_t *icmb_ptr = (icmb_protocol_t *)buf_ptr;
		
		icmb_ptr->addr = addr;
		#if ICMB_ADD_SRC_ID == 1
		icmb_ptr->src_addr = ICMB_SELF_ID;
		#endif
		icmb_ptr->cmd = cmd;
		icmb_ptr->msg_id = msgid;
		icmb_ptr->reg = reg;
		icmb_ptr->len = len;
		
		if( cmd == ICMB_WRITE || cmd == ICMB_READ_BACK ){
			rt_memcpy(&icmb_ptr->data,buf,data_len);
		}
		
		uint32_t crc;
		crc = crc32_cal(buf_ptr, icmb_crc(data_len));
		rt_memcpy((uint8_t*)buf_ptr+icmb_crc(data_len), &crc, 4);
		
		
		if( icmb_state.rs485_port.icmb_485_entx )
			icmb_state.rs485_port.icmb_485_entx();
		
		rt_err_t result;
		result = rt_device_write(icmb_state.rs485_port.serial, 0, buf_ptr, icmb_len(data_len));
		
		rt_free(buf_ptr);
		return result;
	}
	else{
		return RT_ERROR;
	}
}

/** 
* 设置485接口的发送使能及接收使能函数. 
* .
* @param[in]   tx:发送使能函数地址，rx:接收使能函数地址. 
* @param[out]  无.  
* @retval  无. 
* @par 标识符 
*      保留 
* @par 其它 
*      无 
* @par 修改日志 
*      ken deng于2020-10-28创建 
*/
void icmb_485_txrx_cb(void (*tx)(void), void (*rx)(void))
{
	icmb_state.rs485_port.icmb_485_entx = tx;
	icmb_state.rs485_port.icmb_485_enrx = rx;
}

/** 
* ican初始化. 
* can驱动，ican协议栈，can硬件过滤表，ican接收缓冲初始化.
* @param[in]   无. 
* @param[out]  无.  
* @retval  无. 
* @par 标识符 
*      保留 
* @par 其它 
*      无 
* @par 修改日志 
*      ken deng于2020-10-21创建 
*/
extern struct rt_can_filter_item filter_items[];
extern uint8_t filter_cnt;		
static rt_err_t icmb_ican_init(void)
{
	rt_err_t result = RT_EOK;
	
    struct rt_can_filter_config cfg = {filter_cnt, 1, filter_items};
	
	icmb_ican_port_t	*ican_ptr;
	ican_ptr = &icmb_state.ican_port;
	
	ican_ptr->ican_comm = rt_malloc(sizeof(ican_comm_t));
	rt_memset(ican_ptr->ican_comm,0,sizeof(ican_comm_t));
	strncpy(ican_ptr->can_dev,ICMB_ICAN_DEV,sizeof(((icmb_ican_port_t*)0)->can_dev));
	ican_ptr->BaudrateA = ICMB_ICAN_BR_A;
	ican_ptr->BaudrateD = ICMB_ICAN_BR_D;
	ican_ptr->timeout = ICMB_ICAN_ACK_TIME;
	
	if( result == RT_EOK ){
		result = can_init(&ican_ptr->ican_comm->can_port,ican_ptr->can_dev,ican_ptr->BaudrateA, ican_ptr->BaudrateD, &cfg);
	}
	
	if( result == RT_EOK ){
		result = ican_comm_init(&(*ican_ptr->ican_comm));
	}
	
	if( result == RT_EOK ){
		ican_ptr->recv_buf = rt_malloc(ICMB_ICAN_RECV_LEN);
		
		if( ican_ptr->recv_buf == RT_NULL )
			result = RT_ERROR;
	}
	
	return result;
}

/** 
* ican的icmb读写接口. 
* .
* @param[in]   addr：目标地址， msgid:消息ID,cmd：命令，reg：寄存器地址，buf：缓冲指针，len：读写长度. 
* @param[out]  无.  
* @retval  RT_EOK/RT_ERROR. 
* @par 标识符 
*      保留 
* @par 其它 
*      无 
* @par 修改日志 
*      ken deng于2020-10-21创建 
*/
static rt_err_t icmb_ican_operate(uint8_t addr, uint16_t msgid, icmb_cmd_t cmd, uint16_t reg, void *buf, uint16_t len)
{	
	void *buf_ptr;
	uint16_t	data_len;
	
	if( cmd == ICMB_WRITE || cmd == ICMB_READ_BACK ){
		buf_ptr = rt_malloc(icmb_len(len));
		data_len = len;
	}
	else{
		buf_ptr = rt_malloc(icmb_len(0));
		data_len = 0;
	}
	
	if( buf_ptr ){
		
		icmb_protocol_t *icmb_ptr = (icmb_protocol_t *)buf_ptr;
		
		icmb_ptr->addr = addr;
		#if ICMB_ADD_SRC_ID == 1
		icmb_ptr->src_addr = ICMB_SELF_ID;
		#endif
		icmb_ptr->cmd = cmd;
		icmb_ptr->msg_id = msgid;
		icmb_ptr->reg = reg;
		icmb_ptr->len = len;
		
		if( cmd == ICMB_WRITE || cmd == ICMB_READ_BACK ){
			rt_memcpy(&icmb_ptr->data,buf,data_len);
		}
		
		uint32_t crc;
		crc = crc32_cal(buf_ptr, icmb_crc(data_len));
		rt_memcpy((uint8_t*)buf_ptr+icmb_crc(data_len), &crc, 4);
				
		rt_err_t result;
		result = ican_sendto(addr, icmb_state.self_id, icmb_len(data_len), buf_ptr, icmb_state.ican_port.timeout);
		
		rt_free(buf_ptr);
		return result;
	}
	else{
		return RT_ERROR;
	}
}

/** 
* icmb写总接口. 
* 向外提供调用,统一以太网，485，ICAN.
* @param[in]   addr：目标地址，reg_addr：寄存器地址，buf：缓冲指针，len：读写长度. 
* @param[out]  无.  
* @retval  RT_EOK/RT_ERROR. 
* @par 标识符 
*      保留 
* @par 其它 
*      无 
* @par 修改日志 
*      ken deng于2020-10-21创建 
*/
icmb_result_t icmb_write(uint8_t addr, uint16_t reg_addr, void *buf, uint16_t len)
{
	icmb_state.msg_id ++;
	if( icmb_state.msg_id == 0 )		//发出的消息ID不能为0
		icmb_state.msg_id++;
	
	icmb_state.comm_quality.send_cnt ++;
	icmb_state.comm_quality.write_cnt ++;
	
	LOG_I("write msgid %d\r\n",icmb_state.msg_id);
	
	icmb_result_t result = ICMB_RESULT_OK;
	
	if( icmb_state.comm_type & ICMB_UDP ){
		if( icmb_udp_operate(addr, icmb_state.msg_id, ICMB_WRITE, reg_addr, buf, len) != RT_EOK )
			result |= ICMB_RESULT_UDP;
	}
	
	if( icmb_state.comm_type & ICMB_485 ){
		if( icmb_485_operate(addr, icmb_state.msg_id, ICMB_WRITE, reg_addr, buf, len) != RT_EOK )
			result |= ICMB_RESULT_485;
	}
		
	if( icmb_state.comm_type & ICMB_ICAN ){
		if( icmb_ican_operate(addr, icmb_state.msg_id, ICMB_WRITE, reg_addr, buf, len) != RT_EOK )
			result |= ICMB_RESULT_ICAN;
	}
	return result;
//	icmb_state.recv_msg_id = icmb_state.msg_id;
}

/** 
* icmb读返回，向对方发送数据. 
* .
* @param[in]   addr：目标地址，reg_addr：寄存器地址，buf：缓冲指针，len：读写长度. 
* @param[out]  无.  
* @retval  RT_EOK/RT_ERROR. 
* @par 标识符 
*      保留 
* @par 其它 
*      无 
* @par 修改日志 
*      ken deng于2020-10-22创建 
*/
static icmb_result_t icmb_read_back(uint8_t addr, uint16_t reg_addr, void *buf, uint16_t len)
{
	icmb_result_t	result = ICMB_RESULT_OK;
	
	if( icmb_state.comm_type & ICMB_UDP ){
		if( icmb_udp_operate(addr, icmb_get_recv_msgid(addr), ICMB_READ_BACK, reg_addr, buf, len) != RT_EOK )
			result |= ICMB_RESULT_UDP;
	}
	
	if( icmb_state.comm_type & ICMB_485 ){
		if( icmb_485_operate(addr, icmb_get_recv_msgid(addr), ICMB_READ_BACK, reg_addr, buf, len) != RT_EOK )
			result |= ICMB_RESULT_485;
	}
	
	if( icmb_state.comm_type & ICMB_ICAN ){
		if( icmb_ican_operate(addr, icmb_get_recv_msgid(addr), ICMB_READ_BACK, reg_addr, buf, len) != RT_EOK )
			result |= ICMB_RESULT_ICAN;
	}
	return result;
}

/** 
* icmb应答返回. 
* .
* @param[in]   addr：目标地址，reply：ACK/NACK. 
* @param[out]  无.  
* @retval  RT_EOK/RT_ERROR. 
* @par 标识符 
*      保留 
* @par 其它 
*      无 
* @par 修改日志 
*      ken deng于2020-10-22创建 
*/
static rt_err_t icmb_reply(uint8_t addr, uint8_t reply)
{
	if( icmb_state.comm_type & ICMB_UDP )
		icmb_udp_operate(addr, icmb_get_recv_msgid(addr), reply, 0, RT_NULL, 0);
	
	if( icmb_state.comm_type & ICMB_485 )
		icmb_485_operate(addr, icmb_get_recv_msgid(addr), reply, 0, RT_NULL, 0);
	
	if( icmb_state.comm_type & ICMB_ICAN )
		icmb_ican_operate(addr,icmb_get_recv_msgid(addr), reply, 0, RT_NULL, 0);
}

/** 
* 获取协议中的CRC32校验码值. 
* .
* @param[in]   data：CRC32校验码开始地址. 
* @param[out]  无.  
* @retval  CRC32. 
* @par 标识符 
*      保留 
* @par 其它 
*      无 
* @par 修改日志 
*      ken deng于2020-10-22创建 
*/
static uint32_t icmb_get_crc32(uint8_t *data)
{
	uint32_t crc32 = 0;
	crc32 = *data;
	crc32 |= (uint32_t)*(data+1)<<8;
	crc32 |= (uint32_t)*(data+2)<<16;
	crc32 |= (uint32_t)*(data+3)<<24;
	
	return crc32;
}

/** 
* icmb读总接口. 
* 向外提供调用,统一以太网，485，ICAN.
* @param[in]   addr：目标地址，reg_addr：寄存器地址，len：读写长度. 
* @param[out]  无.  
* @retval  RT_EOK/RT_ERROR. 
* @par 标识符 
*      保留 
* @par 其它 
*      无 
* @par 修改日志 
*      ken deng于2020-10-21创建 
*/
icmb_result_t icmb_read(uint8_t addr, uint16_t reg_addr, uint16_t len)
{
	icmb_state.msg_id ++;
	if( icmb_state.msg_id == 0 )		//发出的消息ID不能为0
		icmb_state.msg_id++;

	icmb_state.comm_quality.send_cnt ++;
	icmb_state.comm_quality.read_cnt ++;
	
	LOG_I("read msgid %d\r\n",icmb_state.msg_id);
	
	icmb_result_t result = ICMB_RESULT_OK;
	
	if( icmb_state.comm_type & ICMB_UDP ){
		if( icmb_udp_operate(addr, icmb_state.msg_id, ICMB_READ, reg_addr, RT_NULL, len) != RT_EOK )
			result |= ICMB_RESULT_UDP;
	}
	
	if( icmb_state.comm_type & ICMB_485 ){
		if( icmb_485_operate(addr, icmb_state.msg_id, ICMB_READ, reg_addr, RT_NULL, len) != RT_EOK )
			result |= ICMB_RESULT_485;
	}
	
	if( icmb_state.comm_type & ICMB_ICAN ){
		if( icmb_ican_operate(addr, icmb_state.msg_id, ICMB_READ, reg_addr, RT_NULL, len) != RT_EOK )
			result |= ICMB_RESULT_ICAN;
	}
	
	return result;
	
//	icmb_state.det_msg_id = icmb_state.msg_id;
}

/** 
* icmb寻找指定地址用于判断是否地址重复. 
* 没有回应最多重复寻找5次,最长等待时间5秒.
* @param[in]   addr：目标地址. 
* @param[out]  无.  
* @retval  ICMB_RESULT_OK/ICMB_RESULT_TO. 
* @par 标识符 
*      保留 
* @par 其它 
*      无 
* @par 修改日志 
*      ken deng于2020-11-13创建 
*/
icmb_result_t icmb_search(uint8_t addr)
{
	uint8_t i=0;
	do{
		
		icmb_result_t result = ICMB_RESULT_OK;
		
		if( icmb_state.comm_type & ICMB_UDP ){
			if( icmb_udp_operate(addr, icmb_state.msg_id, ICMB_READ, 0, RT_NULL, 0) != RT_EOK )
				result |= ICMB_RESULT_UDP;
		}
		
		if( icmb_state.comm_type & ICMB_485 ){
			if( icmb_485_operate(addr, icmb_state.msg_id, ICMB_READ, 0, RT_NULL, 0) != RT_EOK )
				result |= ICMB_RESULT_485;
		}
		
		if( icmb_state.comm_type & ICMB_ICAN ){
			if( icmb_ican_operate(addr, icmb_state.msg_id, ICMB_READ, 0, RT_NULL, 0) != RT_EOK )
				result |= ICMB_RESULT_ICAN;
		}
		
		if( rt_sem_take(icmb_state.search_sem, 1000) == RT_EOK ){
			return ICMB_RESULT_OK;
		}
	}while( i++<5 );
	
	return ICMB_RESULT_TO;
}

/** 
* icmb通讯质量参数返回. 
* .
* @param[in]   无. 
* @param[out]  无.  
* @retval  icmb_comm_quality_t 通讯质量指针. 
* @par 标识符 
*      保留 
* @par 其它 
*      无 
* @par 修改日志 
*      ken deng于2020-10-28创建 
*/
icmb_comm_quality_t * icmb_comm_quality(void)
{
	return &icmb_state.comm_quality;
}

/** 
* 注册icmb的只读资源. 
* 向外提供调用.
* @param[in]   src_list：资源表地址，len：资源表长度. 
* @param[out]  无.  
* @retval  RT_EOK/RT_ERROR. 
* @par 标识符 
*      保留 
* @par 其它 
*      无 
* @par 修改日志 
*      ken deng于2020-10-21创建 
*/
void icmb_rosrc_register(void *src_list, uint32_t len)
{
	icmb_state.src_ro_list.src_ptr = src_list;
	icmb_state.src_ro_list.src_len = len;
}

/** 
* 注册icmb的只写资源. 
* 向外提供调用.
* @param[in]   src_list：资源表地址，len：资源表长度. 
* @param[out]  无.  
* @retval  RT_EOK/RT_ERROR. 
* @par 标识符 
*      保留 
* @par 其它 
*      无 
* @par 修改日志 
*      ken deng于2020-10-21创建 
*/
void icmb_rwsrc_register(void *src_list, uint32_t len)
{
	icmb_state.src_wo_list.src_ptr = src_list;
	icmb_state.src_wo_list.src_len = len;
}

/** 
* 注册icmb的用户回调函数. 
* 向外提供调用.
* @param[in]   user_cb：回调函数. 
* @param[out]  无.  
* @retval  无. 
* @par 标识符 
*      保留 
* @par 其它 
*      无 
* @par 修改日志 
*      ken deng于2020-10-21创建 
*/
void icmb_usercb_register(ICMB_USER_CALLBACK_T user_cb)
{
	icmb_state.icmb_user_cb = user_cb;
}

/** 
* icmb内部回调函数. 
* .
* @param[in]   cmd：命令，addr：目标地址，reg_addr：寄存器地址，len：读写长度，buf：缓冲指针. 
* @param[out]  无.  
* @retval  无. 
* @par 标识符 
*      保留 
* @par 其它 
*      无 
* @par 修改日志 
*      ken deng于2020-10-21创建 
*/
static void icmb_callback(icmb_cmd_t cmd, uint8_t addr, uint16_t reg_addr, uint16_t len, void *buf)
{	
	//对方发起写操作
	if( cmd == ICMB_WRITE ){
		
		if( icmb_state.src_wo_list.src_ptr!=RT_NULL ){
			
			if( reg_addr + len <= icmb_state.src_wo_list.src_len ){
			
				LOG_I("%d device write reg %d len %d\r\n",addr,reg_addr,len);
				
				rt_memcpy((uint8_t*)icmb_state.src_wo_list.src_ptr+reg_addr, buf, len );
				
				rt_err_t result;
				if( icmb_state.icmb_user_cb != RT_NULL){
					
					icmb_user_cb_para_t cb_para;
					cb_para.cmd = cmd;
					cb_para.addr = addr;
					cb_para.reg_addr = reg_addr;
					cb_para.len = len;
					cb_para.ptr = (uint8_t*)icmb_state.src_wo_list.src_ptr+reg_addr;
					
					result = icmb_state.icmb_user_cb(&cb_para);
				}
				
				if( result == RT_EOK )
					icmb_reply(addr,ICMB_ACK);
				else
					icmb_reply(addr,ICMB_NACK);
			}
			//超出表写范围
			else{
				icmb_reply(addr,ICMB_NACK);
			}
		}
		else{
			LOG_I("icmb no have write only soruce\r\n");
			icmb_reply(addr,ICMB_NACK);
		}
	}
	//对方发起读操作
	else if( cmd == ICMB_READ ){
		
		if( icmb_state.src_ro_list.src_ptr!=RT_NULL ){
		
			if( reg_addr + len <= icmb_state.src_ro_list.src_len ){
				rt_err_t result;
				if( icmb_state.icmb_user_cb != RT_NULL){
					
					icmb_user_cb_para_t cb_para;
					cb_para.cmd = cmd;
					cb_para.addr = addr;
					cb_para.reg_addr = reg_addr;
					cb_para.len = len;
					cb_para.ptr = (uint8_t*)icmb_state.src_ro_list.src_ptr+reg_addr;
					
					result = icmb_state.icmb_user_cb(&cb_para);
				}
				
				if( result == RT_EOK ){
					LOG_I("%d device read reg %d len %d\r\n",addr,reg_addr,len);
					icmb_read_back(addr, reg_addr, (uint8_t*)icmb_state.src_ro_list.src_ptr+reg_addr, len);
				}
				else{
					LOG_I("read operation user callback reply nack\r\n");
					icmb_reply(addr,ICMB_NACK);
				}
			}
			//超出表读范围
			else{
				icmb_reply(addr,ICMB_NACK);
			}
		}
		else{
			LOG_I("icmb no have read only soruce\r\n");
			icmb_reply(addr,ICMB_NACK);
		}
	}
	//本地发起的读操作返回
	else if( cmd == ICMB_READ_BACK ){
		if( icmb_state.icmb_user_cb != RT_NULL){
			
			icmb_user_cb_para_t cb_para;
			cb_para.cmd = cmd;
			cb_para.addr = addr;
			cb_para.reg_addr = reg_addr;
			cb_para.len = len;
			cb_para.ptr = buf;

			rt_err_t result;
			result = icmb_state.icmb_user_cb(&cb_para);
			
			if( result == RT_EOK ){
				icmb_reply(addr,ICMB_ACK);
				icmb_state.comm_quality.send_ack_cnt ++;
			}
			else
				icmb_reply(addr,ICMB_NACK);
			
			icmb_state.comm_quality.reply_cnt ++;
		}
	}
	//回复寻找地址命令
	else if( cmd == ICMB_SEARCH ){
		icmb_reply(addr,ICMB_SEARCH_ACK);
	}
	//对方响应了寻找地址命令
	else if( cmd == ICMB_SEARCH_ACK ){
		rt_sem_release(icmb_state.search_sem);
	}
	else{		//给用户回调ACK及NACK处理等待其它命令处理
		if( icmb_state.icmb_user_cb != RT_NULL){
			
			icmb_user_cb_para_t cb_para;
			cb_para.cmd = cmd;
			cb_para.addr = addr;
			cb_para.reg_addr = reg_addr;
			cb_para.len = len;
			cb_para.ptr = buf;
			
			icmb_state.comm_quality.reply_cnt ++;
			if( cmd == ICMB_ACK ){
				icmb_state.comm_quality.recv_ack_cnt ++;
			}
			
			icmb_state.icmb_user_cb(&cb_para);
		}
	}
}

/** 
* icmb内部回调函数. 
* .
* @param[in]   recv：数据指针，len：数据长度，msg：返回数据指针、长度. 
* @param[out]  无.  
* @retval  RT_EOK：接收数据成功，RT_ERROR：接收数据未完成. 
* @par 标识符 
*      保留 
* @par 其它 
*      无 
* @par 修改日志 
*      ken deng于2020-10-27创建 
*/
static rt_err_t icmb_check_finish(uint8_t *recv, uint16_t *len, icmb_recv_msg_t *msg)
{
	if( *len > icmb_headlen() ){
	
		icmb_protocol_t *icmb_pro = (icmb_protocol_t *)recv;
		uint16_t data_len;
		if(icmb_pro->cmd == ICMB_WRITE || icmb_pro->cmd == ICMB_READ_BACK){
			data_len = icmb_pro->len;
		}
		else{
			data_len = 0;
		}
		
		if( *len >= icmb_len(data_len) ){
			uint32_t crc1, crc2;
			crc1 = crc32_cal(recv, icmb_crc(data_len));
			
			crc2 = icmb_get_crc32(recv+icmb_crc(data_len));
			
			if( crc1 == crc2 ){
				LOG_I("icmb message id %d\r\n",icmb_pro->msg_id);
			
				icmb_save_recv_msgid(icmb_pro->src_addr, icmb_pro->msg_id);	//清空检测的消息ID
			
				msg->recv_buf = recv;
				msg->recv_len = *len;
				msg->recv_flag = 1;
				*len = 0;
				return RT_EOK;
			
			}
			else
				return RT_ERROR;
		}
		else
			return RT_ERROR;
	}
	else
		return RT_ERROR;
}

/** 
* icmb接收数据处理线程. 
* 接收成功调用内部调用及用户回调.
* @param[in]   paramter：线程参数. 
* @param[out]  无.  
* @retval  无. 
* @par 标识符 
*      保留 
* @par 其它 
*      无 
* @par 修改日志 
*      ken deng于2020-10-21创建 
*/
static void icmb_task(void *paramter)
{
	icmb_recv_msg_t recv_msg;
//	uint8_t src_id;
	
	while(1){
		
		if( icmb_state.comm_type & ICMB_UDP ){
			
			icmb_udp_port_t *udp_ptr = &icmb_state.udp_port;
			socklen_t addr_len = sizeof(struct sockaddr);
			struct sockaddr_in client_addr;
			/* 从sock中收取最大BUFSZ - 1字节数据 */
			int len;
			len = lwip_recvfrom(udp_ptr->socket, udp_ptr->recv_buf, ICMB_UDP_RECV_LEN, MSG_DONTWAIT,
                              (struct sockaddr *)&client_addr, &addr_len);
			
			if( len>0 ){
				udp_ptr->recv_len = len;
				if( icmb_check_finish( udp_ptr->recv_buf, &udp_ptr->recv_len, &recv_msg) == RT_EOK ){
//					src_id = client_addr.sin_addr.s_addr>>24;
					rt_kprintf("recv udp %d msg\r\n",client_addr.sin_addr.s_addr>>24);
				}
			}
		}
		
		if( icmb_state.comm_type & ICMB_485 ){
			
			icmb_485_port_t *rs485_ptr = &icmb_state.rs485_port;
			
			rs485_ptr->recv_len += rt_device_read(rs485_ptr->serial, 0, rs485_ptr->recv_buf + rs485_ptr->recv_len, ICMB_UART_RECV_LEN);
			
			if( icmb_check_finish(rs485_ptr->recv_buf, &rs485_ptr->recv_len, &recv_msg) == RT_EOK ){
				
				#if ICMB_ADD_SRC_ID == 1
				icmb_protocol_t *icmb_pro = (icmb_protocol_t *)rs485_ptr->recv_buf;
//				src_id = icmb_pro->src_addr;
				#endif
//				rt_kprintf("recv 485 %d msg\r\n",src_id);
			}
		}
		
		if( icmb_state.comm_type & ICMB_ICAN ){
			ican_reply_t	reply;
			
			icmb_ican_port_t *ican_ptr = &icmb_state.ican_port;
			ican_ptr->recv_len += ican_recvfrom(icmb_state.self_id, 0, ICMB_ICAN_RECV_LEN, ican_ptr->recv_buf + ican_ptr->recv_len, 0, &reply);
			
			if( icmb_check_finish(ican_ptr->recv_buf, &ican_ptr->recv_len, &recv_msg) == RT_EOK ){
//				src_id = reply.src_mac_id;
//				rt_kprintf("src_id:%d dst_id:%d source_id:%d func_id:%d\r\n",reply.src_mac_id,reply.dest_mac_id,reply.source_id,reply.func_id);
			}
		}
		
		if( recv_msg.recv_flag ){
			
			recv_msg.recv_flag = 0;
			
			static uint8_t **icmb_pro ;
			icmb_pro = &recv_msg.recv_buf;
			
			//if( src_id == 2 ){
				rt_kprintf("addr %d len %d\r\n",((icmb_protocol_t*)*icmb_pro)->src_addr,((icmb_protocol_t*)*icmb_pro)->len);
			//}
			
			icmb_callback(((icmb_protocol_t*)*icmb_pro)->cmd,((icmb_protocol_t*)*icmb_pro)->src_addr,((icmb_protocol_t*)*icmb_pro)->reg,\
			((icmb_protocol_t*)*icmb_pro)->len,&((icmb_protocol_t*)*icmb_pro)->data);
		}
		
		rt_thread_delay(1);
	}
}

/** 
* icmb初始化. 
* 初始化ican,485,udp的接口.
* @param[in]   无. 
* @param[out]  无.  
* @retval  无. 
* @par 标识符 
*      保留 
* @par 其它 
*      无 
* @par 修改日志 
*      ken deng于2020-10-21创建 
*/
rt_err_t icmb_init(void)
{
	rt_err_t result=RT_EOK;
	
	rt_memset(&icmb_state, 0, sizeof(icmb_state_t));
	
	icmb_state.comm_type = ICMB_COMM_TYPE;
	icmb_set_self_id(ICMB_SELF_ID);
	
	icmb_state.search_sem = rt_sem_create("search", 0, RT_IPC_FLAG_FIFO);
	if( icmb_state.search_sem == RT_NULL ){
		
		if( result == RT_EOK ){
			result = RT_ERROR;
		}
	}
	
	if( icmb_state.comm_type & ICMB_UDP ){
		
		result = icmb_udp_init();
	}

	if( icmb_state.comm_type & ICMB_485 ){
		
		if( result == RT_EOK ){
			result = icmb_485_init();
		}
	}

	if( icmb_state.comm_type & ICMB_ICAN ){
		
		if( result == RT_EOK ){
			result = icmb_ican_init();
		}
	}
	
	if( result == RT_EOK ){
		rt_thread_t thread=RT_NULL;
		thread = rt_thread_create("icmb_task", icmb_task, NULL, 1024, 15, 10);
		
		if (thread != RT_NULL){
			
			rt_thread_startup(thread);
		}
		else{
			result = RT_ERROR;
		}
	}
	
	return result;
}
//INIT_ENV_EXPORT(icmb_init);
//INIT_APP_EXPORT(icmb_init);