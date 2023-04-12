/*
 * board_prod.c
 *
 *  Created on: 11 de abr de 2023
 *      Author: igors
 */

/* Includes ------------------------------------------------------------------*/
#include "board_prod.h"
#include "stdio.h"

/* Defines -------------------------------------------------------------------*/
#define PROD_MODE_TIMEOUT       10
#define AT_BUFFER_SIZE 			64
#define AT_END_OF_CHAR        	'\r'

/* Functions prototypes -----------------------------------------------------*/
static void at_exe_cmd_null(at_cmd_table_t* prod_table);
static void at_exe_quit(at_cmd_table_t* prod_table);
static void at_query_inf(at_cmd_table_t* prod_table);

/* Variables -----------------------------------------------------------------*/
extern uint32_t timestamp;
extern fw_version_t fw_version;
extern const uart_handler_t *p_huart1;
extern const uart_handler_t *p_huart3;

/* cmd_name, cmd_len, (*test_cmd), (*query_cmd), (*setup_cmd), (*exe_cmd)
 *         ,        , AT+<CMD>=?\r, AT+<CMD>?\r, AT+<CMD>=..\r, AT+<CMD>\r */
const at_cmd_table_t prod_table[] = {
  {""       , 0, NULL, NULL         , NULL        , at_exe_cmd_null},
  {"+QUIT"  , 5, NULL, NULL 		, NULL        , at_exe_quit},
  {"+INF"   , 4, NULL, at_query_inf , NULL        , NULL},
//  {"+HWV"   , 4, NULL, at_query_hwv , NULL        , NULL},
//  {"+SN"    , 3, NULL, at_query_sn  , NULL        , NULL},
//  {"+JIG"   , 4, NULL, at_query_jig , at_setup_jig, NULL}
};
const size_t AT_CMD_HANDLE_NUM = (sizeof(prod_table) / sizeof(prod_table[0]));

static char at_buffer[AT_BUFFER_SIZE] = {0};
at_prod_t at_prod;

const char at_head[]          = "AT";
const char AT_HEAD_LENGTH     =  sizeof(at_head) - 1;
const char at_cmd_ok[]        = "\r\nOK\r\n";
const char at_cmd_error[]     = "\r\nERROR\r\n";

/* Functions definitions -----------------------------------------------------*/
/**
 * \brief           Initialize AT command Handler
 * \param[in]       uart: Uart Handler
 */
void at_prod_init(void){
	uint32_t start = timestamp;

	/* Setup production mode Handler */
	at_prod.p_input = (lwrb_t *) &p_huart1->rx_data->usart_rx_rb;
	at_prod.prod_table = (at_cmd_table_t *) &prod_table;
	at_prod.table_size = AT_CMD_HANDLE_NUM;
	at_prod.p_buffer = (char *) &at_buffer;
	at_prod.buffer_size = ARRAY_LEN(at_buffer);
	at_prod.at_id = 0;
	at_prod.buff_id = 0;
	at_prod.at_capture = 0;
	at_prod.prod_mode_en = 0;

	printf("!{\r\n\"Label\": \"Pre-Production\"\r\n");
	printf("\"Info\": \"Type AT + Enter to continue\"\r\n");
	printf("}!\r\n\r\n");

	/* Wait for the instruction to enter Production Mode */
	while(!at_prod.prod_mode_en && (timestamp - start) < PROD_MODE_TIMEOUT){
		at_prod_handle();
	}
	/* Process Production Mode */
	while(at_prod.prod_mode_en) at_prod_handle();
}

void at_prod_handle(void){
	uint8_t buff[1]; /* Purpose the address is 4-byte aligned here */
	size_t cnt_limit = at_prod.buffer_size - AT_HEAD_LENGTH;
	while (lwrb_read(at_prod.p_input, &buff, 1) > 0){
		char c = buff[0];
		if (!at_prod.at_capture){
			if (check_strncmp((char*) &at_head, c, &at_prod.at_id, AT_HEAD_LENGTH)){
				at_prod.at_capture = 1;
				at_prod.buff_id = 0;
				cnt_limit = at_prod.buffer_size - AT_HEAD_LENGTH;
			}
		}else{
			at_prod.p_buffer[at_prod.buff_id++] = c;
			if (at_prod.buff_id == at_prod.buffer_size){
				at_prod.at_capture = 0;
			}
			if (c == AT_END_OF_CHAR){
				at_prod.p_buffer[at_prod.buff_id++] = '\0'; /* Add null terminal character */
				at_prod.at_capture = 0;
				at_prod_cmd_parse();
				break;
			}
		}
		// Avoid while loop forever
		--cnt_limit;
		if(0 == cnt_limit) break;
	}
}

void at_prod_cmd_parse(void){
	int cmd_length;
	int cmd_index = -1;

	cmd_length = at_cmd_get_length();
	if(cmd_length != -1){
		for(int i = 0; i < at_prod.table_size; ++i){
			if(cmd_length == at_prod.prod_table[i].cmd_len){
				if(memcmp(at_prod.p_buffer,
						at_prod.prod_table[i].cmd_name,
						cmd_length) == 0){
					cmd_index = i;
					break;
				}
			}
		}
	}
	if(cmd_index != -1){
		at_cmd_table_t* p_cmd_handle = (at_cmd_table_t *) &at_prod.prod_table[cmd_index];
		char* s;
		s = at_prod.p_buffer + cmd_length;
		if(s[0] == '\r'){
			if(p_cmd_handle->exe_cmd){
				p_cmd_handle->exe_cmd(p_cmd_handle);
			}else{
				printf(at_cmd_error);
			}
		}else if(s[0] == '?' && (s[1] == '\r')){
			if(p_cmd_handle->query_cmd){
				p_cmd_handle->query_cmd(p_cmd_handle);
			}else{
				printf(at_cmd_error);
			}
		}else if((s[0] == '=') && (s[1] == '?') && (s[2] == '\r')){
			if(p_cmd_handle->test_cmd){
				p_cmd_handle->test_cmd(p_cmd_handle);
			}else{
				printf(at_cmd_error);
			}
		}else if(((s[0] >= '0') && (s[0] <= '9')) || (s[0] == '=')){
			if(p_cmd_handle->setup_cmd){
				char* cr = strrchr(s, '\r');
				if (cr){
					*cr = '\0'; /* remove CR character */
				}
				++s;
				p_cmd_handle->setup_cmd(p_cmd_handle, s);
			}else{
				printf(at_cmd_error);
			}
		}else{
			printf(at_cmd_error);
		}

	}else{
		printf(at_cmd_error);
	}
	printf("\r\n");
}

bool check_strncmp(char *str, char dat, uint8_t *index, uint8_t len){
	if (str[*index] == dat){
		(*index)++;
		if ((*index) == len){
			*index = 0;
			return true;
		}
	}else{
		*index = 0;
		if (str[*index] == dat) (*index)++;
	}
	return false;
}

int at_cmd_get_length(void){
	char* s = at_prod.p_buffer;
	size_t size = at_prod.buffer_size;
	int index = -1;
	while(*s && size){
		char c = *s;
		++s;
		if((c == '\r') || (c == '=') || (c == '?')
				|| ((c >= '0') && (c <= '9'))){
			index = at_prod.buffer_size - size;
			break;
		}else{
			--size;
		}
	}
	return index;
}

static void at_exe_cmd_null(at_cmd_table_t* prod_table){
	at_prod.prod_mode_en = 1;
	printf(at_cmd_ok);
}

static void at_exe_quit(at_cmd_table_t* prod_table){
	at_prod.prod_mode_en = 0;
	printf("\r\n%s: OK\r\n", prod_table->cmd_name);
}

static void at_query_inf(at_cmd_table_t* prod_table){
	printf("\r\n%s: \"%u.%u.%u\"\r\n%s",
			prod_table->cmd_name,
			fw_version.arq,
			fw_version.feature,
			fw_version.hotfix,
			at_cmd_ok);
}
