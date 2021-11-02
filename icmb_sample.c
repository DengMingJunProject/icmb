#include "rtthread.h"
#include "icmb_sample.h"
#include "icmb.h"
#include "string.h"

static pecu_src_list_t pecu_src_ro_list;
static pecu_src_list_t pecu_src_wo_list;

DEF_ICMB_ICAN_FILTER()
	//接收本地ID
	RT_CAN_FILTER_ITEM_INIT(ICAN_ID_ITEM_INIT(ICMB_SELF_ID,0,0,0,0),1,0,0,ICAN_ID_ITEM_INIT(0xff,0,0,0,0), RT_NULL, RT_NULL),
	//接收ICAN广播数据
	RT_CAN_FILTER_ITEM_INIT(ICAN_ID_ITEM_INIT(0xff,0,0,0,0),1,0,0,ICAN_ID_ITEM_INIT(0xff,0,0,0,0), RT_NULL, RT_NULL),
END_DEF_ICMB_ICAN_FILTER()

//ICMB用户回调
DEF_ICMB_USER_CB(pecu_icmb_cb)/*(icmb_user_cb_para_t *cb_para)*/

	//回调参数
	/*typedef struct _icmb_user_cb_para{
		icmb_cmd_t cmd;
		uint8_t addr;
		uint16_t reg_addr;
		uint16_t len; 
		uint8_t *ptr;
	}icmb_user_cb_para_t;
	*/
	//用户回调函数返回RT_EOK，则向发送方返回ACK
	//用户回调函数返回RT_ERROR，则向发送方返回NACK，
	rt_err_t result = RT_EOK;
	//写入操作，写入数据后调用回调
	if( cb_para->cmd==ICMB_WRITE ){
		
		switch( cb_para->reg_addr ){
			
			case offsetof(pecu_src_list_t,dev_info):
			{
				pecu_device_info_t *dev_info_ptr = (pecu_device_info_t *)cb_para->ptr;;
				
				char buf[30];
				rt_memset(buf,0,30);
				rt_memcpy(buf,dev_info_ptr->sn,sizeof(((pecu_device_info_t*)0)->sn));
				rt_kprintf("write dev sn %s\r\n",buf);
				
				rt_memset(buf,0,30);
				rt_memcpy(buf,dev_info_ptr->product,sizeof(((pecu_device_info_t*)0)->product));
				rt_kprintf("write dev product %s\r\n",buf);
				
				rt_memset(buf,0,30);
				rt_memcpy(buf,dev_info_ptr->date_of_manufacture,sizeof(((pecu_device_info_t*)0)->date_of_manufacture));
				rt_kprintf("write dev manufacture %s\r\n",buf);
				break;
			}
			case offsetof(pecu_src_list_t,work_info):
			{
				pecu_work_info_t *work_info_ptr = (pecu_work_info_t *)cb_para->ptr;
				rt_kprintf("write error=%d state=%d time=%d\r\n",work_info_ptr->error,work_info_ptr->state,work_info_ptr->time);
				break;
			}
			case offsetof(pecu_src_list_t,work_set):
			{
				pecu_work_set_t *work_set_ptr = (pecu_work_set_t *)cb_para->ptr;
				rt_kprintf("write mode=%d\r\n",work_set_ptr->mode);
				break;
			}
			default:
				result = RT_ERROR;
				break;
		}
	}
	//读取操作，返回数据前调用回调
	else if( cb_para->cmd==ICMB_READ ){
		
		//数据返回前对pecu_src_ro_list进行修改
		switch( cb_para->reg_addr ){
			case offsetof(pecu_src_list_t,dev_info):
			{
				pecu_device_info_t *dev_info_ptr = (pecu_device_info_t *)cb_para->ptr;;
				
				rt_memcpy(dev_info_ptr->sn,"1234567890",sizeof(((pecu_device_info_t*)0)->sn));
				rt_memcpy(dev_info_ptr->product,"global link pecu 100",sizeof(((pecu_device_info_t*)0)->product));
				rt_memcpy(dev_info_ptr->date_of_manufacture,"2020-10-21 14:41:35",sizeof(((pecu_device_info_t*)0)->date_of_manufacture));
				break;
			}
			case offsetof(pecu_src_list_t,work_info):
			{
				pecu_work_info_t *work_info_ptr = (pecu_work_info_t *)cb_para->ptr;
				work_info_ptr->error = 0;
				work_info_ptr->state = 1;
				work_info_ptr->time = 124;
				break;
			}
			case offsetof(pecu_src_list_t,work_set):
			{
				pecu_work_set_t *work_set_ptr = (pecu_work_set_t *)cb_para->ptr;
				work_set_ptr->mode = 8;
				break;
			}
			default:
				result = RT_ERROR;
				break;
		}
	}
	else if( cb_para->cmd ==  ICMB_READ_BACK ){
		char buf[11]={0};
		rt_memcpy(buf,cb_para->ptr,10);
		rt_kprintf("RB msg %s\r\n",buf);
//		for( uint16_t i=0; i<cb_para->len; i++ ){
//			rt_kprintf("%02x ",*cb_para->ptr++);
//		}
//		rt_kprintf("\r\n");
	}
	else if( cb_para->cmd ==  ICMB_ACK ){
		rt_kprintf("recv No.%d ack\r\n",cb_para->addr);
	}
	else if( cb_para->cmd ==  ICMB_NACK ){
		rt_kprintf("recv No.%d nack\r\n",cb_para->addr);
	}
	else{
		result = RT_ERROR;
	}
	return result;
END_DEF_ICMB_USER_CB(pecu_icmb_cb)

void icmb_sample(int argc, char *argv[])
{
	if( strcmp(argv[1],"start")==0 && argc==2 ){
		icmb_rosrc_register(&pecu_src_ro_list,sizeof(pecu_src_list_t));		//注册只读数据区域
		icmb_rwsrc_register(&pecu_src_wo_list,sizeof(pecu_src_list_t));		//注册只写数据区域
		icmb_usercb_register(pecu_icmb_cb);									//注册命令响应回调函数
		rt_memcpy(&pecu_src_ro_list.dev_info,"pecu20201019",sizeof(((pecu_src_list_t*)0)->dev_info));
	}
	else if( strcmp(argv[1],"read")==0 && argc==5 ){
		uint8_t id = atoi(argv[2]);
		uint16_t reg = atoi(argv[3]);
		uint16_t len = atoi(argv[4]);
		//icmb_read(1, offsetof(pecu_src_list_t,dev_info), sizeof(((pecu_src_list_t*)0)->dev_info));
		if( icmb_read(id, reg, len) == ICMB_RESULT_OK )
			rt_kprintf("icmb read cmd send OK\r\n");
	}
	else if( strcmp(argv[1],"write")==0 && argc==6 ){
		uint8_t id = atoi(argv[2]);
		uint16_t reg = atoi(argv[3]);
		uint16_t len = atoi(argv[4]);
		//icmb_write(1, offsetof(pecu_src_list_t,dev_info), "abcdefghij",sizeof(((pecu_src_list_t*)0)->dev_info));
		if( icmb_write(id, reg, argv[5],len) == ICMB_RESULT_OK )
			rt_kprintf("icmb write cmd send OK\r\n");
	}
	else if( strcmp(argv[1],"list")==0 && argc==3 ){
		uint8_t *ptr;
		uint16_t len;
		if( strcmp(argv[2],"ro")==0 ){
			ptr = (uint8_t*)&pecu_src_ro_list;
			len = sizeof(pecu_src_list_t);
		}
		else if( strcmp(argv[2],"rw")==0 ){
			ptr = (uint8_t*)&pecu_src_wo_list;
			len = sizeof(pecu_src_list_t);
		}
		
		rt_kprintf("print %s data list:\n",argv[2]);
		uint16_t i=0;
		uint8_t j = 0;
		while( i<len ){
			if( j<16 )
				rt_kprintf("%02x ",*ptr++);
			j++;
			if( j==16 ){
				j = 0;
				rt_kprintf("\n");
			}
			
			i++;
		}
		rt_kprintf("\r\n");
	}
	else{
		rt_kprintf("error icmb_sample parameter\r\n");
	}
}
#ifdef RT_USING_FINSH
#include <finsh.h>
FINSH_FUNCTION_EXPORT(icmb_sample, [start|read|write|list id reg len data] running the icmb test sample );
#ifdef FINSH_USING_MSH
MSH_CMD_EXPORT(icmb_sample, [start|read|write|list id reg len data] running the icmb test sample );
#endif /* FINSH_USING_MSH */
#endif /* RT_USING_FINSH */

void icmb_sample_auto_task(void *parameter)
{	
	static uint32_t	send_cnt=0;
	uint8_t flag = 1;
	while(1)
	{
		if( flag==1 ){
			flag = 2;
			icmb_read(3, 0, 154);
		}
		else if( flag==2 ){
			flag = 1;
			icmb_read(2, 0, 124);
		}
		send_cnt++;
		rt_kprintf("send %u\r\n",send_cnt);
		rt_thread_delay(100);
	}
}

void icmb_sample_auto_test(void)
{
	icmb_rosrc_register(&pecu_src_ro_list,sizeof(pecu_src_list_t));		//注册只读数据区域
	icmb_rwsrc_register(&pecu_src_wo_list,sizeof(pecu_src_list_t));		//注册只写数据区域
	icmb_usercb_register(pecu_icmb_cb);									//注册命令响应回调函数
	rt_memcpy(&pecu_src_ro_list.dev_info,"pecu20201019",sizeof(((pecu_src_list_t*)0)->dev_info));
	
	rt_thread_t thread=RT_NULL;
	thread = rt_thread_create("icmb_sample_auto", icmb_sample_auto_task, NULL, 1024, 15, 10);
	
	if (thread != RT_NULL){
		
		rt_thread_startup(thread);
	}
}
//INIT_APP_EXPORT(icmb_sample_auto_test);
