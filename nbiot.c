/*
 * nbiot.c
 *
 *  Created on: 24 de out de 2023
 *      Author: igors
 */

/* Includes ------------------------------------------------------------------*/
#include "nbiot.h"
#include "util.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "ctype.h"
#include "fw_version.h"
#include "sensors.h"
#include "lwjson.h"
#include "math.h"

/* Defines -------------------------------------------------------------------*/
#define SMALL_TIMEOUT			300
#define LARGE_TIMEOUT			CONV_2MS(6)
#define EXTENDED_TIMEOUT		CONV_2MS(15)
#define MODEM_ST_MCH_TIMEOUT	CONV_2MS(2*MINUTES_LEN)
#define RX_BUFFER_SIZE			2048
#define NBIOT_CMD_SIZE   		256

/* Private function prototypes -----------------------------------------------*/

/* Variables -----------------------------------------------------------------*/
extern huart_port_t huart_p3;
extern uint32_t timestamp;

uint8_t rx_buffer[RX_BUFFER_SIZE];
char cmd[NBIOT_CMD_SIZE] = {0};

/**
 * @brief Struct do modem.
 *
 * Estrutura com as variáveis necessárias para o funcionamendo do modem.
 */
modem_t modem = {
	.prev_state = BC660K_EXIT,
	.state = BC660K_INIT,
	.next_state = BC660K_INIT,
	.n_fails = 0,
	.clk_fails = 0,
	.delay = {0},
	.timer = {0},
	.tasks = {0},
	.status = {0},
	.rev = {0},
	.imei = {0},
	.iccid = {0},
	.apn = {0},
	.oper = {0},
	.act_cnt = 0,
	.rsrp = 0,
	.rsrq = 0,
	.sirn = 0,
	.banda = 0,
	.lqi = LQI_INVALID,
	.timestamp = 0,
	.timezone = 0
};

/**
 * @brief Struct da interface de comandos AT do modem nbiot.
 *
 * Estrutura com as variáveis necessárias para o funcionamendo da interface at.
 */
at_cmd_t at_cmd = {
	.index = 0,
	.p_rx = (char *) &rx_buffer,
	.rx_index = 0,
	.rx_size = RX_BUFFER_SIZE,
	.timer = {0}
};

/**
 * @brief Lista de comandos AT do modem nbiot.
 *
 * O array contem as informações necessárias para a execução do comando.
 * Formato: {index, corpo do comando, resposta esperada, callback, timeout}
 */
static const nbiot_cmd_t nbiot_cmd[] = {
	{AT_CMD, AT, NULL, self_test_ok, LARGE_TIMEOUT},
	{ATI_CMD, ATI, _ATI, rev_process, LARGE_TIMEOUT},
	{IMEI_CMD, CGSN, _CGSN, imei_process, LARGE_TIMEOUT},
	{CCID_CMD, QCCID, _QCCID, ccid_process, LARGE_TIMEOUT},
	{ATE_CMD, ATE1, NULL, NULL, LARGE_TIMEOUT},
	{CFUN0_CMD, CFUN0, NULL, cfun0_ok, EXTENDED_TIMEOUT},
	{CFUN1_CMD, CFUN1, NULL, NULL, LARGE_TIMEOUT},
	{BAND_CMD, QBAND, NULL, NULL, LARGE_TIMEOUT},
	{CEREG_CMD, CEREG, NULL, NULL, LARGE_TIMEOUT},
	{Q_CGATT_CMD, CGATT, _CGATT, cgatt_process, LARGE_TIMEOUT},
	{Q_CGPADDR_CMD, CGPADDR, _CGPADDR, cgpaddr_process, LARGE_TIMEOUT},
	{DNS_CFG_CMD, DNS_CFG, NULL, NULL, SMALL_TIMEOUT},
	{CMEE_CMD, CMEE, NULL, NULL, LARGE_TIMEOUT},
	{APN_CMD, APN, NULL, NULL, LARGE_TIMEOUT},
	{COPS_CMD, COPS, NULL, NULL, LARGE_TIMEOUT},
	{Q_COPS_CMD, Q_COPS, _COPS, cops_process, LARGE_TIMEOUT},
	{QENG_CMD, QENG, NULL, qeng_process, LARGE_TIMEOUT},
	{CCLK_CMD, CCLK, NULL, cclk_process, LARGE_TIMEOUT},
	{CLEARNB_CMD, QCSEARFCN, NULL, NULL, LARGE_TIMEOUT},
	{CGATT_CMD, CGATT, NULL, NULL, LARGE_TIMEOUT},
};

/* Functions definitions ----------------------------------------------------*/
void nbiot_hw_reset(void){
	/* Desabilita flags */
	modem.status.nb_conn = false;
	modem.status.self_test = false;
	/* Reinicia o modem */
	HAL_GPIO_WritePin(BC_RESET_GPIO_Port, BC_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(BC_RESET_GPIO_Port, BC_RESET_Pin, GPIO_PIN_SET);
}

void nbiot_cmd_send(nbiot_cmd_list_t index, char *arg, nbiot_st_mch_t n_state){
	/* Limpa o buffer e a resposta */
	memset(at_cmd.p_rx, 0, at_cmd.rx_index);
	at_cmd.rx_index = 0;
	at_cmd.rsp = AT_RESP_GARBAGE;
	/* Encontra comando pelo index e salva argumentos */
	nbiot_cmd_t *p_cmd = (nbiot_cmd_t *) &nbiot_cmd[index];
	at_cmd.index = index;
	at_cmd.p_rsp = p_cmd->exp_rsp;
	at_cmd.rsp_len = strlen(p_cmd->exp_rsp);
	/* Liga o temporizador do comando */
	at_cmd.timer.run = true;
	at_cmd.timer.t0 = HAL_GetTick();
	at_cmd.timer.timeout = p_cmd->timeout;
	/* Envia o comando para o modem */
	if(strlen(p_cmd->at_cmd) > 0) uart_write_string(&huart_p3, p_cmd->at_cmd, strlen(p_cmd->at_cmd));
	if(strlen(arg) > 0) uart_write_string(&huart_p3, arg, strlen(arg));
	uart_write_string(&huart_p3, CRLF, strlen(CRLF));
	/* Debug da comunicacao */
#if (defined DEBUG_NBIOT) && (DEBUG_NBIOT == 1)
	char c;
	printf("Modem Tx: {");
	for(int i = 0; i < strlen(p_cmd->at_cmd); ++i){
		c = p_cmd->at_cmd[i];
		if(!iscntrl(c)) printf("%c", c);
	}
	for(int i = 0; i < strlen(arg); ++i){
		c = arg[i];
		if(!iscntrl(c)) printf("%c", c);
	}
	printf("}%s", CRLF);
#endif
	/* Controle da Maquina de Estados */
	modem.prev_state = modem.state;
	modem.state = BC660K_WAIT_COMM;
	modem.next_state = n_state;
}

void nbiot_rx_handler(void){
	char buff = 0;
	/* Verifica se o comando foi executado */
	if(!at_cmd.timer.run) return;
	/* Verifica o temporizador e o tamanho da resposta do comando */
	if((HAL_GetTick()-at_cmd.timer.t0) >= at_cmd.timer.timeout){
		/* Define a resposta como encontrada como estouro do timer */
		at_cmd.rsp = AT_RESP_TIMEOUT;
		at_cmd.timer.run = false;
#if (defined DEBUG_NBIOT) && (DEBUG_NBIOT == 1)
		printf("Modem Rx: {TIMEOUT}%s", CRLF);
#endif
	}else{
		/* Leitura do buffer */
		while(uart_read_byte(&huart_p3, &buff)){
			memcpy(&at_cmd.p_rx[at_cmd.rx_index++], (char *) &buff, 1);
			/* Verifica tamanho da resposta */
			if(at_cmd.rx_index >= at_cmd.rx_size){
				/* Define a resposta como encontrada como lixo */
				at_cmd.rsp = AT_RESP_GARBAGE;
				at_cmd.timer.run = false;
#if (defined DEBUG_NBIOT) && (DEBUG_NBIOT == 1)
				printf("Modem Rx: {GARBAGE}%s", CRLF);
#endif
				break;
				/* Busca terminador de string ou data mode */
			}else if(buff == *CRLF || buff == *DMODE){
				/* Processa o buffer */
				if(nbiot_rx_process(&at_cmd, at_cmd.p_rx, at_cmd.rx_index)) break;
			}
		}
	}
}

bool nbiot_rx_process(at_cmd_t *p_at_cmd, char *buffer, size_t size){
	bool erro = false, ok = false;
	/* Verifica tamanho e conteudo do buffer */
	if(size >= p_at_cmd->rsp_len){
		/* Procura por erros */
		if(strnstr(buffer, _ERROR, size) != NULL){
			erro = true;
		}else{
			/* Caso o ponteiro de resposta for nulo, busca por resposta generica OK */
			if(strlen(p_at_cmd->p_rsp) == 0){
				if(strnstr(buffer, _OK, size) != NULL){
					ok = true;
				}
			}else{
				/* Busca resposta esperada do comando */
				if(strnstr(buffer, p_at_cmd->p_rsp, size) != NULL){
					/* Alem da resposta esperada, buscar pelo resposta OK, > ou +QMTRECV */
					if(strnstr(buffer, _OK, size) != NULL){
						ok = true;
					}
				}
			}
		}
	}
	if(erro || ok){
		/* Para o timer */
		at_cmd.timer.run = false;
		/* Processa a resposta encontrada */
		if(erro){
			p_at_cmd->rsp = AT_RESP_ERROR;
#if (defined DEBUG_NBIOT) && (DEBUG_NBIOT == 1)
			printf("Modem Rx: {ERROR}%s", CRLF);
#endif
		}else if(ok){
			p_at_cmd->rsp = AT_RESP_OK;
			/* Acessa a tabela de comandos e busca o callback */
			nbiot_cmd_t *p_cmd = (nbiot_cmd_t *) &nbiot_cmd[p_at_cmd->index];
			/* Executa o callback caso exista algum cadastrado na tabela */
			if(p_cmd->run_cb != NULL) p_cmd->run_cb(buffer, size);
			/* Envia o comando para o modem */
#if (defined DEBUG_NBIOT) && (DEBUG_NBIOT == 1)
			char c;
			printf("Modem Rx: {");
			for(int i = 0; i < size; ++i){
				c = buffer[i];
				if(!iscntrl(c)) printf("%c", c);
			}
			printf("}%s", CRLF);
#endif
		}
		return true;
	}
	return false;
}

nbiot_st_mch_t nbiot_st_mch(void){
	/* Processa timeout da maquina de estados */
	nbiot_timeout_process();
	/* Salva estado anterior caso seja estado de comando */
	if(modem.state >= BC660K_EXIT) modem.prev_state = modem.state;
	/* Run State Machine */
	switch (modem.state){
	case BC660K_INIT:{
		modem.timer.t0 = HAL_GetTick();
		modem.timer.timeout = MODEM_ST_MCH_TIMEOUT;
		modem.n_fails = 0;
		modem.clk_fails = 0;
		modem.state = BC660K_IDLE;
	}
	break;

	case BC660K_IDLE:{
		/* No tasks pending: disconnect and turn off modem */
		if(modem.tasks.all_flags == 0){
			if(modem.status.nb_conn){
				modem.state = BC660K_DISC;
			}else{
				modem.state = BC660K_TURN_OFF;
			}
		}else{
			/* Perform self-test */
			if(!modem.status.self_test){
				modem.state = BC660K_SELF_TEST;
			/* Get modem info */
			}else if(modem.tasks.get_rev){
				modem.state = BC660K_GET_REV;
			/* Get modem IMEI */
			}else if(modem.tasks.get_imei){
				modem.state = BC660K_GET_IMEI;
			/* Get modem ICCID */
			}else if(modem.tasks.get_ccid){
				modem.state = BC660K_GET_CCID;
			/* Modem setup and turn on radio */
			}else if(!modem.status.nb_conn){
				if(modem.status.setup){
					modem.state = BC660K_CFUN_ON;
				}else{
					modem.state = BC660K_SET_ECHO;
				}
			/* Get link quality info */
			}else if(modem.tasks.get_rfinfo){
				modem.state = BC660K_RF_INFO;
			/* Get date time from network */
			}else if(modem.tasks.sync && !modem.status.clk_fail){
				modem.state = BC660K_SYNC;
			}else{
				modem.tasks.all_flags = 0;
			}
		}
	}
	break;

	case BC660K_SELF_TEST:{
		nbiot_cmd_send(AT_CMD, NULL, BC660K_IDLE);
	}
	break;

	case BC660K_GET_REV:{
		nbiot_cmd_send(ATI_CMD, NULL, BC660K_IDLE);
	}
	break;

	case BC660K_GET_IMEI:{
		nbiot_cmd_send(IMEI_CMD, NULL, BC660K_IDLE);
	}
	break;

	case BC660K_GET_CCID:{
		nbiot_cmd_send(CFUN1_CMD, NULL, BC660K_GET_CCID_1);
	}
	break;

	case BC660K_GET_CCID_1:{
		nbiot_cmd_send(CCID_CMD, NULL, BC660K_IDLE);
	}
	break;

	case BC660K_SET_ECHO:{
		nbiot_cmd_send(ATE_CMD, NULL, BC660K_CFUN_OFF);
	}
	break;

	case BC660K_CFUN_OFF:{
		nbiot_cmd_send(CFUN0_CMD, NULL, BC660K_SET_BAND);
	}
	break;

	case BC660K_SET_BAND:{
		nbiot_cmd_send(BAND_CMD, "=28", BC660K_CEREG);
	}
	break;

	case BC660K_CEREG:{
		nbiot_cmd_send(CEREG_CMD, "=0", BC660K_CMEE);
	}
	break;

	case BC660K_CMEE:{
		nbiot_cmd_send(CMEE_CMD, NULL, BC660K_CFUN_ON);
	}
	break;

	case BC660K_CFUN_ON:{
		nbiot_cmd_send(CFUN1_CMD, NULL, BC660K_SET_APN);
	}
	break;

	case BC660K_SET_APN:{
		modem.status.setup = true;
		nbiot_cmd_send(APN_CMD, "=1,\"IP\",\"ex_apn\"", BC660K_COPS);
	}
	break;

	case BC660K_COPS:{
		nbiot_cmd_send(COPS_CMD, "=0", BC660K_Q_CGATT);
	}
	break;

	case BC660K_Q_CGATT:{
		nbiot_cmd_send(Q_CGATT_CMD, "?", BC660K_Q_CGPADDR);
	}
	break;

	case BC660K_Q_CGPADDR:{
		nbiot_cmd_send(Q_CGPADDR_CMD, NULL, BC660K_DNS_CFG);
	}
	break;

	case BC660K_DNS_CFG:{
		nbiot_cmd_send(DNS_CFG_CMD, NULL, BC660K_IDLE);
	}
	break;

	case BC660K_RF_INFO:{
		nbiot_cmd_send(Q_COPS_CMD, NULL, BC660K_RF_INFO_2);
	}
	break;

	case BC660K_RF_INFO_2:{
		nbiot_cmd_send(QENG_CMD, NULL, BC660K_IDLE);
	}
	break;

	case BC660K_SYNC:{
		nbiot_cmd_send(CCLK_CMD, NULL, BC660K_IDLE);
	}
	break;

	case BC660K_DISC:{
		modem.status.nb_conn = false;
		nbiot_cmd_send(COPS_CMD, "=2", BC660K_IDLE);
	}
	break;

	case BC660K_WAIT_COMM:{
		if(!at_cmd.timer.run){
			if(at_cmd.rsp == AT_RESP_OK){
				modem.n_fails = 0;
				modem.state = modem.next_state;
			}else{
				modem.n_fails++;
				modem.next_state = modem.prev_state;
				modem.state = BC660K_FAIL_HANDLER;
			}
		}
	}
	break;

	case BC660K_DELAY:{
		if((HAL_GetTick()-modem.delay.t0) >= modem.delay.timeout){
			modem.state = modem.next_state;
		}
	}
	break;

	case BC660K_FAIL_HANDLER:{
		/* Processamento de falhas */
		if(modem.n_fails >= 3){
			/* Reinicia contador */
			modem.n_fails = 0;
			/* Reinicia a comunicação (uart) com o modem */
			HAL_UART_DeInit(huart_p3.p_huart);
			HAL_Delay(10);
			MX_USART3_UART_Init();
			/* Reinicia o modem */
			nbiot_hw_reset();
			/* Retorna estado IDLE */
			modem.state = BC660K_IDLE;
		}else{
			/* Clean rx buffer */
			uart_flush_rx(&huart_p3);
			memset(at_cmd.p_rx, 0, at_cmd.rx_size);
			/* Envia Esc e Terminador para cancelar comandos não completos */
			uart_write_string(&huart_p3, CRLF, strlen(CRLF));
			uart_write_string(&huart_p3, ESC, strlen(ESC));
			/* Volta ao estado anterior */
			nbiot_set_delay(1000, modem.prev_state);
		}
	}
	break;

	case BC660K_RT_RESET:{
		nbiot_cmd_send(AT_CMD, NULL, BC660K_RT_CFUN_0);
	}
	break;

	case BC660K_RT_CFUN_0:{
		nbiot_cmd_send(CFUN0_CMD, NULL, BC660K_RT_CLEAR_NB);
	}
	break;

	case BC660K_RT_CLEAR_NB:{
		nbiot_cmd_send(CLEARNB_CMD, NULL, BC660K_RT_CFUN_1);
	}
	break;

	case BC660K_RT_CFUN_1:{
		nbiot_cmd_send(CFUN1_CMD, NULL, BC660K_RT_SET_ATTCH);
	}
	break;

	case BC660K_RT_SET_ATTCH:{
		nbiot_cmd_send(CGATT_CMD, "=1", BC660K_IDLE);
	}
	break;

	case BC660K_TURN_OFF:{
		modem.status.self_test = false;
		nbiot_cmd_send(CFUN0_CMD, NULL, BC660K_EXIT);
	}
	break;

	case BC660K_EXIT:{
		if(modem.tasks.all_flags != 0) modem.state = BC660K_INIT;
	}
	break;

	default: break;
	}
	/* Process Rx Buffer */
	nbiot_rx_handler();
	/* Return Modem State */
	return modem.state;
}

void nbiot_set_delay(uint32_t timeout, nbiot_st_mch_t next_state){
	modem.state = BC660K_DELAY;
	modem.next_state = next_state;
	modem.delay.t0 = HAL_GetTick();
	modem.delay.timeout = timeout;
}

void nbiot_timeout_process(void){
	if(modem.state == BC660K_INIT ||
			modem.state == BC660K_TURN_OFF ||
			modem.state == BC660K_WAIT_COMM ||
			modem.state == BC660K_EXIT) return;
	/* Check if timer is running */
	if((HAL_GetTick()-modem.timer.t0) >= modem.timer.timeout){
		/* Clear setup */
		modem.status.setup = false;
		/* Desabilita tarefas */
		modem.tasks.all_flags = 0;
		/* desliga o modem */
		modem.state = BC660K_TURN_OFF;
	}
}

size_t is_valid_luhn(char* number){
    int len = 0, sum = 0;
    char c = 0;
    /* Validate input and calculate length */
    do {
    	c = (char) number[len];
    	if(!isdigit(c)) break;
    	len++;
    } while(len <= MAX_ICCID_SIZE);
    /* Check size */
    if (len < MAX_IMEI_SIZE || len > MAX_ICCID_SIZE) return 0;
    /* Luhn Algorithm Validation */
    bool alternate = false;
    for (int i = len - 1; i >= 0; i--) {
        int d = number[i] - '0';
        if (alternate) {
            d *= 2;
            if (d > 9) d -= 9;  // Sum the digits if > 9
        }
        sum += d;
        alternate = !alternate;  // Toggle alternation
    }
    /* Check if the total sum is a multiple of 10 */
    return (sum % 10 == 0 ? len : 0);
}

lqi_t get_signal_quality(int rsrp, int rsrq, int snr){
	if(rsrp < -140 || rsrp > 0 || snr < -20 || snr > 30 || rsrq < -20 || rsrq > 0){
		return LQI_INVALID;
	}else if(rsrp >= -100 && snr >= 3 && rsrq >= -7){
		return LQI_STRONG;
	}else if(rsrp >= -110 && snr > -3 && rsrq > -11){
		return LQI_MEDIUM;
	}else{
        return LQI_WEAK;
    }
}

/* Callbacks -----------------------------------------------------------------*/
void rev_process(char* data, size_t len){
	char *pch = NULL;
	/* Valida se a resposta foi encontrada */
	pch = strnstr(data, _ATI, len);
	if(pch != NULL){
		/* Pula texto da resposta */
		pch += strlen(_ATI);
		*strnstr(pch, "\r\n", len) = 0;
		if(strlen(pch) < MAX_REV_SIZE){
			strncpy((char *) &modem.rev, pch, MAX_REV_SIZE);
			modem.tasks.get_rev = false;
		}
	}
}

void imei_process(char* data, size_t len){
	char *pch = NULL;
	/* Valida se a resposta foi encontrada */
	pch = strnstr(data, _CGSN, len);
	if(pch != NULL){
		/* Pula texto da resposta */
		pch += strlen(_CGSN);
		/* Validação da string com IMEI encontrado */
		uint8_t diff = is_valid_luhn(pch);
		if(diff){
			strncpy((char *) &modem.imei, pch, diff);
			modem.tasks.get_imei = false;
		}
	}
}

void ccid_process(char* data, size_t len){
	char *pch = NULL;
	uint8_t diff = 0;
	/* Valida se a resposta foi encontrada */
	pch = strnstr(data, _QCCID, len);
	if(pch != NULL){
		/* Pula texto da resposta */
		pch += strlen(_QCCID);
		/* Validação da string com NCCID encontrado */
		diff = is_valid_luhn(pch);
		if(diff >= MIN_ICCID_SIZE && diff <= MAX_ICCID_SIZE){
			strncpy((char *) &modem.iccid, pch, diff);
			modem.tasks.get_ccid = false;
		}
	}
}

void self_test_ok(char* data, size_t len){
	modem.status.self_test = true;
}

void cfun0_ok(char* data, size_t len){
	modem.status.nb_conn = false;
}

void cgatt_process(char* data, size_t len){
	char *pch = NULL;
	/* Valida se a resposta foi encontrada */
	pch = strnstr(data, _CGATT, len);
	if(pch != NULL){
		/* Pula texto da resposta */
		pch += strlen(_CGATT);
		/* Validação se o ME esta conectado com a operadora */
		if(atoi(pch) != 1){
			nbiot_set_delay(1000, BC660K_Q_CGATT);
		}
	}
}

void cgpaddr_process(char* data, size_t len){
	char *pch = NULL;
	/* Valida se a resposta foi encontrada */
	pch = strnstr(data, _CGPADDR, len);
	if(pch != NULL){
		/* Pula texto da resposta */
		pch += strlen(_CGPADDR);
		/* Validação do endereco de IP */
		if(strlen(pch) > 3){
			modem.status.nb_conn = true;
		}else{
			nbiot_set_delay(1000, BC660K_Q_CGATT);
		}
	}
}

void cops_process(char* data, size_t len){
	char *start = data, *end = NULL;
	start = strstr(start, "\"");
	end = strstr(start + 1, "\"");
	if(start != NULL && end != NULL && ((end - data) < len)){
		start++;
		memcpy((char *) &modem.oper[0], start, (end - start));
	}
}

void qeng_process(char* data, size_t len){
	uint8_t banda = 0;
	char *p_rsrp = NULL, *p_rsrq = NULL, *p_snr = NULL, *p_band = NULL;
	p_rsrp = strnstr(data, _RSRP, len);
	p_rsrq = strnstr(data, _RSRQ, len);
	p_snr = strnstr(data, _SNR, len);
	p_band = strnstr(data, _BAND, len);
	if(p_rsrp != NULL && p_rsrq != NULL && p_snr != NULL && p_band != NULL){
		/* Validação da banda encontrada */
		banda = atoi(p_band + strlen(_BAND));
		if(banda != 0){
			modem.status.setup = false;
			modem.banda = banda;
		}
		/* Validação dos indicadores encontrados */
		modem.rsrp = roundf(atoi(p_rsrp + strlen(_RSRP)) / 10.0);
		modem.rsrq = roundf(atoi(p_rsrq + strlen(_RSRQ)) / 10.0);
		modem.sirn = roundf(atoi(p_snr + strlen(_SNR)) / 10.0);
		if(modem.rsrp != -32768 && modem.rsrq != -32768 && modem.sirn != -32768){
			modem.lqi = get_signal_quality(modem.rsrp, modem.rsrq, modem.sirn);
			modem.tasks.get_rfinfo = false;
		}
	}
}

void cclk_process(char* data, size_t len){
	char *pch = NULL;
	/* Valida se a resposta foi encontrada */
	pch = strnstr(data, _CCLK, len);
	if(pch != NULL){
		/* Pula texto da resposta */
		pch += strlen(_CCLK);
		*strstr(pch, "\r\n") = 0;
		/* Parse com validação da string date-time encontrada */
		if(strlen(pch) == MAX_DATETIME_SIZE &&
				parse_datetime(pch, &modem.timestamp, &modem.timezone)){
			timestamp = modem.timestamp;
			modem.tasks.sync = false;
			/* Tenta encontrar date-time valido novamente 3 vezes */
		}else if(++modem.clk_fails < 3){
			nbiot_set_delay(1000, BC660K_SYNC);
			/* Caso de falha, segue para sincronizacao com a rede pelo topico no mqtt */
		}else{
			modem.status.clk_fail = true;
			nbiot_set_delay(10, BC660K_IDLE);
		}
		/* Caso o provedor não forneça horario da rede, segue para sincronizacao pelo mqtt */
	}else{
		modem.status.clk_fail = true;
		nbiot_set_delay(10, BC660K_IDLE);
	}
}
