/*
 * board_prod.h
 *
 *  Created on: 11 de abr de 2023
 *      Author: igors
 */

#ifndef INC_BOARD_PROD_H_
#define INC_BOARD_PROD_H_

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* Defines -------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/
typedef struct at_cmd_table {
	char *cmd_name;                            				/*!< at command name */
	int8_t cmd_len;                            				/*!< at command length */
	void (*test_cmd)(struct at_cmd_table*);    				/*!< Test Command function pointer AT+<CMD>=?\r */
	void (*query_cmd)(struct at_cmd_table*);   				/*!< Query Command function pointer AT+<CMD>?\r */
	void (*setup_cmd)(struct at_cmd_table*, char *p_para); 	/*!< Setup Command function pointer AT+<CMD>=<data>\r */
	void (*exe_cmd)(struct at_cmd_table*);     				/*!< Execute Command function pointer AT+<CMD>\r */
} at_cmd_table_t;

typedef struct{
	lwrb_t* p_input;
	at_cmd_table_t* prod_table;
	size_t table_size;
	char* p_buffer;
	size_t buffer_size;
	uint8_t at_id;
	uint8_t at_capture;
	uint8_t buff_id;
	uint8_t prod_mode_en;
} at_prod_t;

/* Functions definitions -----------------------------------------------------*/
void at_prod_init(void);
void at_prod_handle(void);
void at_prod_cmd_parse(void);
bool check_strncmp(char *str, char dat, uint8_t *index, uint8_t len);
int at_cmd_get_length(void);

#endif /* INC_BOARD_PROD_H_ */
