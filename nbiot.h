/*
 * nbiot.h
 *
 *  Created on: 24 de out de 2023
 *      Author: igors
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"

/* Defines -------------------------------------------------------------------*/
#define MAX_REV_SIZE			20
#define MAX_IMEI_SIZE			15
#define MIN_ICCID_SIZE			19
#define MAX_ICCID_SIZE			20
#define MAX_APN_SIZE			30
#define MAX_OPER_SIZE			10
#define MAX_DATETIME_SIZE		20
#define DEF_TIMEZONE			-3

#define AT "AT"
#define CRLF "\r\n"
#define CTRL_Z "\x1A"
#define DMODE ">"
#define ESC "\x1B"

#define ATE1 "ATE1"
#define QCSEARFCN "AT+NCSEARFCN"
#define CEREG "AT+CEREG"
#define CMEE "AT+CMEE=0"
#define QBAND "AT+NBAND"
#define CFUN0 "AT+CFUN=0"
#define CFUN1 "AT+CFUN=1"
#define ATI "ATI"
#define CGSN "AT+CGSN=1"
#define QCCID "AT+NCCID"
#define APN "AT+CGDCONT"
#define COPS "AT+COPS"
#define Q_COPS "AT+COPS?"
#define CGATT "AT+CGATT"
#define CGPADDR "AT+CGPADDR"
#define DNS_CFG "AT+QIDNSCFG=1.1.1.1,8.8.8.8"
#define CBC "AT+CBC"
#define QENG "AT+NUESTATS"
#define CCLK "AT+CCLK?"

#define _OK "OK"
#define _ERROR "ERROR"
#define _RDY "RDY"
#define _CEREG "+CEREG:0,"
#define _CGATT "+CGATT:"
#define _CGPADDR "+CGPADDR:"
#define _APN "+CGDCONT:"
#define _ATI "Revision:"
#define _CGSN "+CGSN:"
#define _QCCID "+NCCID:"
#define _COPS "+COPS:"
#define _RSRP "Signal power:"
#define _RSRQ "RSRQ:"
#define _SNR "SNR:"
#define _BAND "CURRENT BAND:"
#define _CCLK "+CCLK:"

/* Private function prototypes -----------------------------------------------*/

/* Variables -----------------------------------------------------------------*/
typedef enum {
    AT_RESP_OK = 0,
    AT_RESP_ERROR,
    AT_RESP_TIMEOUT,
    AT_RESP_GARBAGE
} cmd_rsp_t;

typedef void (*cmd_cb)(char*, size_t);

typedef enum {
    AT_CMD = 0,
	ATI_CMD,
	IMEI_CMD,
	CCID_CMD,
	ATE_CMD,
	CFUN0_CMD,
	CFUN1_CMD,
	BAND_CMD,
	CEREG_CMD,
	Q_CGATT_CMD,
	Q_CGPADDR_CMD,
	DNS_CFG_CMD,
	CMEE_CMD,
	APN_CMD,
	COPS_CMD,
	Q_COPS_CMD,
	QENG_CMD,
	CCLK_CMD,
	CLEARNB_CMD,
	CGATT_CMD
} nbiot_cmd_list_t;

typedef enum {
	/* Estados de verificacao */
	BC660K_INIT,
	BC660K_IDLE,
	BC660K_WAIT_COMM,
	BC660K_DELAY,
	BC660K_FAIL_HANDLER,
	BC660K_TURN_OFF,
	BC660K_EXIT,
	/* Estados de comandos */
	BC660K_SELF_TEST,
	BC660K_GET_REV,
	BC660K_GET_IMEI,
	BC660K_GET_CCID,
	BC660K_GET_CCID_1,
	BC660K_SET_ECHO,
	BC660K_CFUN_OFF,
	BC660K_SET_BAND,
	BC660K_CEREG,
	BC660K_CMEE,
	BC660K_CFUN_ON,
	BC660K_SET_APN,
	BC660K_COPS,
	BC660K_Q_CGATT,
	BC660K_Q_CGPADDR,
	BC660K_DNS_CFG,
	BC660K_GET_VBATT,
	BC660K_RF_INFO,
	BC660K_RF_INFO_2,
	BC660K_SYNC,
	BC660K_DISC,
	BC660K_RT_RESET,
	BC660K_RT_CFUN_0,
	BC660K_RT_CLEAR_NB,
	BC660K_RT_CFUN_1,
	BC660K_RT_SET_ATTCH
} nbiot_st_mch_t;

typedef struct {
	nbiot_cmd_list_t cmd_index;			/* Index do comando AT */
	char *at_cmd;						/* String com o corpo do comando AT */
	char *exp_rsp;						/* String com a resposta esperada pelo comando AT */
	cmd_cb run_cb;						/* Callback executado quando a string esperada é recebida */
	uint32_t timeout;					/* Timeout do comando */
} nbiot_cmd_t;

typedef struct {
	/* Index do comando */
	size_t index;
	/* Buffer de resposta */
	char* p_rsp;
	size_t rsp_len;
	/* Callback de resposta */
	cmd_cb run_cb;
	/* Tipo de resposta do comando */
	cmd_rsp_t rsp;
	/* Buffer receptor */
	char* p_rx;
	size_t rx_index;
	size_t rx_size;
	/* Temporizador do comando */
	struct {
		bool run;
		uint32_t t0;
		uint32_t timeout;
	} timer;
} at_cmd_t;

typedef enum{
	LQI_STRONG = 0,
	LQI_MEDIUM,
	LQI_WEAK,
	LQI_INVALID
} lqi_t;

typedef struct {
	/* State Control */
	nbiot_st_mch_t prev_state;
	nbiot_st_mch_t state;
	nbiot_st_mch_t next_state;
	uint8_t n_fails;
	uint8_t clk_fails;
	/* Timer */
	struct {
		uint32_t t0;
		uint32_t timeout;
	} timer;
	struct {
		uint32_t t0;
		uint32_t timeout;
	} delay;
	/* Flags */
	union {
		uint8_t all_flags;
		struct {
			uint8_t reserved : 3,
			get_rev : 1,
			get_imei : 1,
			get_ccid : 1,
			get_rfinfo : 1,
			sync : 1;
		};
	} tasks;
	/* Modem task status */
	union {
		uint8_t all_flags;
		struct {
			uint8_t reserved : 4,
			self_test : 1,
			setup : 1,
			nb_conn : 1,
			clk_fail : 1;
		};
	} status;
	/* Modem Information */
	uint8_t rev[MAX_REV_SIZE + 1];
	uint8_t imei[MAX_IMEI_SIZE + 1];
	uint8_t iccid[MAX_ICCID_SIZE + 1];
	uint8_t apn[MAX_APN_SIZE + 1];
	uint8_t oper[MAX_OPER_SIZE + 1];
	uint16_t act_cnt;
	int rsrp;
	int rsrq;
	int sirn;
	uint8_t banda;
	lqi_t lqi;
	uint32_t timestamp;
	int8_t timezone;
} modem_t;

/* Functions definitions -----------------------------------------------------*/
void nbiot_hw_reset(void);
void nbiot_cmd_send(nbiot_cmd_list_t index, char *arg, nbiot_st_mch_t n_state);
void nbiot_rx_handler(void);
bool nbiot_rx_process(at_cmd_t *p_at_cmd, char *buffer, size_t size);
nbiot_st_mch_t nbiot_st_mch(void);
void nbiot_set_delay(uint32_t timeout, nbiot_st_mch_t next_state);
void nbiot_timeout_process(void);
size_t is_valid_luhn(char* number);
lqi_t get_signal_quality(int rsrp, int rsrq, int snr);

/* Callbacks -----------------------------------------------------------------*/
void rev_process(char* data, size_t len);
void imei_process(char* data, size_t len);
void ccid_process(char* data, size_t len);
void self_test_ok(char* data, size_t len);
void cfun0_ok(char* data, size_t len);
void cgatt_process(char* data, size_t len);
void cgpaddr_process(char* data, size_t len);
void cops_process(char* data, size_t len);
void qeng_process(char* data, size_t len);
void cclk_process(char* data, size_t len);
