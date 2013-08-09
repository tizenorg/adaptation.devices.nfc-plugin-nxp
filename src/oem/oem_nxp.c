/*
 * Copyright (C) 2010 NXP Semiconductors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <dirent.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <malloc.h>
#include <vconf.h>

#include "net_nfc_oem_controller.h"
#include "net_nfc_typedef.h"
#include "nfc_debug_private.h"
#include "phLibNfc.h"
#include "phDal4Nfc_message_glib.h"
#include "phNfcHalTypes.h"
#include "phNfcIoctlCode.h"
#include "net_nfc_util_private.h"

////////////// DEFINE START /////////////////

#ifndef NET_NFC_EXPORT_API
#define NET_NFC_EXPORT_API __attribute__((visibility("default")))
#endif

//#define EEDATA_SETTINGS_NUMBER 24
#define EEDATA_SETTINGS_NUMBER 29 // from ginger bread 2.3.4
#define NET_NFC_EEPROM_WRITEN "memory/private/nfc-plugin-nxp/eeprom"
#define  NET_NFC_OEM_CONTROLLER_LOCK \
do{\
	pthread_mutex_lock(&g_controller_lock);\
}while(0);

#define  NET_NFC_OEM_CONTROLLER_UNLOCK \
do{\
	pthread_mutex_unlock(&g_controller_lock);\
}while(0);

#define NET_NFC_OEM_CONTROLLER_SIGNAL(cond) \
do {\
	pthread_mutex_lock(&g_controller_lock);\
	pthread_cond_signal(cond);\
	pthread_mutex_unlock(&g_controller_lock);\
}while(0);

#define NET_NFC_OEM_CONTROLLER_WAIT(cond) \
do {\
	struct timeval now;\
	struct timespec ts;\
	gettimeofday(&now, NULL);\
	ts.tv_sec = now.tv_sec + 2;\
	ts.tv_nsec = now.tv_usec * 1000;\
	pthread_cond_timedwait(cond, &g_controller_lock, &ts);\
}while(0);

typedef struct _socket_info_s
{
	net_nfc_llcp_socket_t socket_handle;
	data_s data;
	unsigned int my_index;
	void *user_context;
	bool isValid;
//	net_nfc_llcp_socket_t socket_handle_incomming;
	uint16_t miu;
	uint8_t rw;
	void *context;
} socket_info_s;

typedef struct _ndef_info_s
{
	uint8_t ndef_card_state;
	int max_data_size;
	int real_data_size;
} ndef_info_s;

typedef struct _controller_context_s
{
	pthread_cond_t *controller_cond;
	void *user_context;
	int result;
} controller_context_s;

typedef struct _accept_context_s
{
	net_nfc_target_handle_s *handle;
	net_nfc_llcp_socket_t server_scket;
	net_nfc_llcp_socket_t incomming;
	void *user_param;
} accept_context_s;

typedef struct _transfer_context_s
{
	net_nfc_llcp_socket_t oal_socket;
	data_s data;
	void *user_param;
} transfer_context_s;

////////////// DEFINE END /////////////////

////////////// STATIC FIELD START ///////////////

/* static variable */

#define KEYS_ISO14443A_MAX 6
#define KEYS_ISO14443B_MAX 7
#define KEYS_JEWEL_MAX 3
#define KEYS_FELICA_MAX 3
#define KEYS_ISO15693_MAX 4
#define KEYS_NFCIP_MAX 5

#define keys_ISO14443A "UID:APP_DATA:SAK:ATQA:MAX_DATA_RATE:FWI_SFGT:"
#define keys_ISO14443B "UID:APP_DATA:PROTOCOL_INFO:ATQ_RESPONSE:HI_LAYER_RESPONSE:AFI:MAX_DATA_RATE:"
#define keys_JEWEL "UID:HEADER_ROM0:HEADER_ROM1:"
#define keys_FELICA "IDm:PMm:SYSTEM_CODE:"
#define keys_ISO15693 "UID:DSF_ID:FLAGS:AFI:"
#define keys_NFCIP "UID:ATR_INFORMATION:SAK:ATQA:MAX_DATA_RATE:"

#define BUFFER_LENGTH_MAX 1024

#define READ_BUFFER_LENGTH_MAX BUFFER_LENGTH_MAX
#define WRITE_BUFFER_LENGTH_MAX BUFFER_LENGTH_MAX
#define NET_NFC_MAX_LLCP_SOCKET_BUFFER BUFFER_LENGTH_MAX

#define NET_NFC_MAX_TRANCIEVE_BUFFER 256

static phHal_sHwReference_t psHwRef = { 0 };
//static phLibNfc_sADD_Cfg_t	sADDConfig ;
static bool g_stack_init_successful;
static pthread_mutex_t g_controller_lock = PTHREAD_MUTEX_INITIALIZER;
static bool is_EEPROM_writen = false;
extern const uint8_t nxp_nfc_fw[];

static net_nfc_target_handle_s *current_working_handle = NULL;

/* callback */
static target_detection_listener_cb g_nxp_controller_target_cb;
static se_transaction_listener_cb g_nxp_controller_se_cb;
static llcp_event_listener_cb g_nxp_controller_llcp_cb;

static phLibNfc_SE_List_t g_se_list[PHLIBNFC_MAXNO_OF_SE] = { { 0 } };
static uint8_t g_se_no = 0;

/* llcp */
socket_info_s socket_info_array[PHFRINFC_LLCP_NB_SOCKET_MAX] = { { 0, } };

static phNfc_sData_t gInputParam;
static phNfc_sData_t gOutputParam;
////////////// STATIC FIELD END ///////////////

////////////// STATIC FUNCTION DECLARE START //////////

static bool net_nfc_nxp_controller_init(net_nfc_error_e *result);
static bool net_nfc_nxp_controller_deinit(void);
static bool net_nfc_nxp_controller_register_listener(target_detection_listener_cb target_detection_listener, se_transaction_listener_cb se_transaction_listener, llcp_event_listener_cb llcp_event_listener, net_nfc_error_e *result);
static bool net_nfc_nxp_controller_unregister_listener();
static bool net_nfc_nxp_controller_get_firmware_version(data_s **data, net_nfc_error_e *result);
static bool net_nfc_nxp_controller_check_firmware_version(net_nfc_error_e *result);
static bool net_nfc_nxp_controller_update_firmware(net_nfc_error_e *result);
static bool net_nfc_nxp_controller_get_stack_information(net_nfc_stack_information_s *stack_info, net_nfc_error_e *result);
static bool net_nfc_nxp_controller_configure_discovery(net_nfc_discovery_mode_e mode, net_nfc_event_filter_e config, net_nfc_error_e *result);
static bool net_nfc_nxp_controller_get_secure_element_list(net_nfc_secure_element_info_s *list, int *count, net_nfc_error_e *result);
static bool net_nfc_nxp_controller_set_secure_element_mode(net_nfc_secure_element_type_e element_type, net_nfc_secure_element_mode_e mode, net_nfc_error_e *result);
static bool net_nfc_nxp_controller_connect(net_nfc_target_handle_s *handle, net_nfc_error_e *result);
static bool net_nfc_nxp_controller_disconnect(net_nfc_target_handle_s *handle, net_nfc_error_e *result);
static bool net_nfc_nxp_controller_check_ndef(net_nfc_target_handle_s *handle, uint8_t *ndef_card_state, int *max_data_size, int *real_data_size, net_nfc_error_e *result);
static bool net_nfc_nxp_controller_check_target_presence(net_nfc_target_handle_s *handle, net_nfc_error_e *result);
static bool net_nfc_nxp_controller_read_ndef(net_nfc_target_handle_s *handle, data_s **data, net_nfc_error_e *result);
static bool net_nfc_nxp_controller_write_ndef(net_nfc_target_handle_s *handle, data_s *data, net_nfc_error_e *result);
static bool net_nfc_nxp_controller_make_read_only_ndef(net_nfc_target_handle_s *handle, net_nfc_error_e *result);
static bool net_nfc_nxp_controller_transceive(net_nfc_target_handle_s *handle, net_nfc_transceive_info_s *info, data_s **data, net_nfc_error_e *result);
static bool net_nfc_nxp_controller_format_ndef(net_nfc_target_handle_s *handle, data_s *secure_key, net_nfc_error_e *result);
static bool net_nfc_nxp_controller_exception_handler(void);
static bool net_nfc_nxp_controller_is_ready(net_nfc_error_e *error);

static bool net_nfc_nxp_controller_llcp_config(net_nfc_llcp_config_info_s *config, net_nfc_error_e * result);
static bool net_nfc_nxp_controller_llcp_check_llcp(net_nfc_target_handle_s *handle, net_nfc_error_e *result);
static bool net_nfc_nxp_controller_llcp_activate_llcp(net_nfc_target_handle_s *handle, net_nfc_error_e *result);
static bool net_nfc_nxp_controller_llcp_create_socket(net_nfc_llcp_socket_t *socket, net_nfc_socket_type_e socketType, uint16_t miu, uint8_t rw, net_nfc_error_e *result, void *user_param);
static bool net_nfc_nxp_controller_llcp_bind(net_nfc_llcp_socket_t socket, uint8_t service_access_point, net_nfc_error_e *result);
static bool net_nfc_nxp_controller_llcp_listen(net_nfc_target_handle_s *handle, uint8_t *service_access_name, net_nfc_llcp_socket_t socket, net_nfc_error_e *result, void *user_param);
static bool net_nfc_nxp_controller_llcp_accept(net_nfc_llcp_socket_t socket, net_nfc_error_e *result);
static bool net_nfc_nxp_controller_llcp_connect_by_url(net_nfc_target_handle_s *handle, net_nfc_llcp_socket_t socket, uint8_t *service_access_name, net_nfc_error_e *result, void *user_param);
static bool net_nfc_nxp_controller_llcp_connect(net_nfc_target_handle_s *handle, net_nfc_llcp_socket_t socket, uint8_t service_access_point, net_nfc_error_e *result, void *user_param);
static bool net_nfc_nxp_controller_llcp_reject(net_nfc_target_handle_s *handle, net_nfc_llcp_socket_t socket, net_nfc_error_e *result);
static bool net_nfc_nxp_controller_llcp_disconnect(net_nfc_target_handle_s *handle, net_nfc_llcp_socket_t socket, net_nfc_error_e *result, void *user_param);
static bool net_nfc_nxp_controller_llcp_socket_close(net_nfc_llcp_socket_t socket, net_nfc_error_e *result);
static bool net_nfc_nxp_controller_llcp_recv(net_nfc_target_handle_s *handle, net_nfc_llcp_socket_t socket, data_s *data, net_nfc_error_e *result, void *user_param);
static bool net_nfc_nxp_controller_llcp_send(net_nfc_target_handle_s *handle, net_nfc_llcp_socket_t socket, data_s *data, net_nfc_error_e *result, void *user_param);
static bool net_nfc_nxp_controller_llcp_recv_from(net_nfc_target_handle_s *handle, net_nfc_llcp_socket_t socket, data_s *data, net_nfc_error_e *result, void *user_param);
static bool net_nfc_nxp_controller_llcp_send_to(net_nfc_target_handle_s *handle, net_nfc_llcp_socket_t socket, data_s *data, uint8_t service_access_point, net_nfc_error_e *result, void *user_param);
static bool net_nfc_nxp_controller_llcp_get_remote_config(net_nfc_target_handle_s *handle, net_nfc_llcp_config_info_s *config, net_nfc_error_e *result);
static bool net_nfc_nxp_controller_llcp_get_remote_socket_info(net_nfc_target_handle_s *handle, net_nfc_llcp_socket_t socket, net_nfc_llcp_socket_option_s *option, net_nfc_error_e *result);

//static void net_nfc_nxp_controller_print_tag_info(phNfc_uRemoteDevInfo_t *devInfo, phNfc_eRemDevType_t RemDevType);
static int net_nfc_nxp_controller_tag_device_info(phNfc_uRemoteDevInfo_t *devInfo, phNfc_eRemDevType_t RemDevType, uint8_t **buffer, uint32_t *buffer_length);
static void net_nfc_nxp_controller_device_info_ISO14443A(phNfc_uRemoteDevInfo_t *devInfo, uint8_t **buffer, uint32_t *buffer_length);
static void net_nfc_nxp_controller_device_info_ISO14443B(phNfc_uRemoteDevInfo_t *devInfo, uint8_t **buffer, uint32_t *buffer_length);
static void net_nfc_nxp_controller_device_info_Jewel(phNfc_uRemoteDevInfo_t *devInfo, uint8_t **buffer, uint32_t *buffer_length);
static void net_nfc_nxp_controller_device_info_Felica(phNfc_uRemoteDevInfo_t *devInfo, uint8_t **buffer, uint32_t *buffer_length);
static void net_nfc_nxp_controller_device_info_ISO15693(phNfc_uRemoteDevInfo_t *devInfo, uint8_t **buffer, uint32_t *buffer_length);
static void net_nfc_nxp_controller_device_info_NFCIP(phNfc_uRemoteDevInfo_t *devInfo, uint8_t **buffer, uint32_t *buffer_length);

static bool net_nfc_nxp_controller_sim_test(net_nfc_error_e *result);
static bool net_nfc_nxp_controller_prbs_test(net_nfc_error_e *result , uint32_t tech , uint32_t rate);

static bool net_nfc_nxp_controller_test_mode_on(net_nfc_error_e *result);
static bool net_nfc_nxp_controller_test_mode_off(net_nfc_error_e *result);

static bool net_nfc_nxp_controller_support_nfc(net_nfc_error_e *result);

/* callback function */
static void _net_nfc_init_cb(void *pContext, NFCSTATUS status);
static void _net_nfc_deinit_cb(void *pContext, NFCSTATUS status);

static void _net_nfc_se_notification_cb(void *pContext, phLibNfc_eSE_EvtType_t EventType, phLibNfc_Handle hSecureElement, phLibNfc_uSeEvtInfo_t *pSeEvtInfo, NFCSTATUS status);
static void _net_nfc_remotedev_notification_cb(void *pContext, phLibNfc_RemoteDevList_t *psRemoteDevList, uint8_t uNofRemoteDev, NFCSTATUS status);
static void _net_nfc_configure_discovery_cb(void *pContext, NFCSTATUS status);
static void _net_nfc_connect_cb(void *pContext, phLibNfc_Handle hRemoteDev, phLibNfc_sRemoteDevInformation_t *psRemoteDevInfo, NFCSTATUS status);
static void _net_nfc_disconnect_cb(void *pContext, phLibNfc_Handle hRemoteDev, NFCSTATUS status);
static void _net_nfc_transceive_cb(void *pContext, phLibNfc_Handle hRemoteDev, phNfc_sData_t *pResBuffer, NFCSTATUS status);
static void _net_nfc_ndef_read_cb(void *pContext, NFCSTATUS tatus);
static void _net_nfc_ndef_write_cb(void *pContext, NFCSTATUS status);
static void _net_nfc_make_read_only_cb(void *pContext, NFCSTATUS status);
static void _net_nfc_format_remote_dev_ndef_cb(void *pContext, NFCSTATUS status);
static void _net_nfc_target_check_presence_cb(void *pContext, NFCSTATUS status);
static void _net_nfc_target_check_ndef_cb(void *pContext, phLibNfc_ChkNdef_Info_t Ndef_Info, NFCSTATUS status);
static void _net_nfc_firmware_cb(void *pContext, phNfc_sData_t *Outparam_Cb, NFCSTATUS status);

static void _net_nfc_llcp_recv_from_cb(void *pContext, uint8_t service_access_point, NFCSTATUS status);
static void _net_nfc_llcp_recv_cb(void *pContext, NFCSTATUS status);
static void _net_nfc_llcp_sendTo_cb(void *pContext, NFCSTATUS status);
static void _net_nfc_llcp_send_cb(void *pContext, NFCSTATUS status);
static void _net_nfc_llcp_connect_cb(void *pContext, uint8_t nErrCode, NFCSTATUS status);
static void _net_nfc_llcp_connect_sap_cb(void *pContext, uint8_t nErrCode, NFCSTATUS status);
static void _net_nfc_llcp_accept_cb(void *pContext, NFCSTATUS status);
static void _net_nfc_llcp_accepted_socket_err_cb(void *pContext, uint8_t nErrCode);
static void _net_nfc_llcp_listen_cb(void *pContext, net_nfc_llcp_socket_t IncomingSocket);
static void _net_nfc_llcp_socket_err_cb(void *pContext, uint8_t nErrCode);
static void _net_nfc_llcp_link_status_cb(void *pContext, phLibNfc_Llcp_eLinkStatus_t eLinkStatus);
static void _net_nfc_llcp_check_llcp_cb(void *pContext, NFCSTATUS status);
static void _net_nfc_llcp_disconnect_cb(void *pContext, NFCSTATUS status);

static void _net_nfc_test_mode_off_cb(void *pContext, NFCSTATUS status);
static void _net_nfc_test_mode_on_cb(void *pContext, NFCSTATUS status);
static void _net_nfc_swp_test_cb(void *pContext, phNfc_sData_t *Outparam_Cb, NFCSTATUS status);

/* etc static function */
static net_nfc_error_e _net_nfc_nxp_error_converter(uint16_t result);
static int net_nfc_nxp_controller_convert_target_type(int type, phLibNfc_RemoteDevList_t *TagInfo);

/*
 static void _net_nfc_nxp_set_mode_default();
 static void _net_nfc_nxp_set_mode_off();
 static void _net_nfc_nxp_set_mode_reader();
 static void _net_nfc_nxp_set_mode_p2p();
 static void _net_nfc_nxp_set_mode_paymentonly();
 */
static bool _net_nfc_nxp_check_pprom_is_completed();
static void _net_nfc_nxp_set_pprom_is_completed();
static void _net_nfc_nxp_disable_irq_timeout();
static void _net_nfc_nxp_controller_lock_init();

static socket_info_s *_net_nfc_get_available_socket_slot();
static void _net_nfc_remove_socket_slot(net_nfc_llcp_socket_t socket);
static void _net_nfc_reset_socket_array();

static bool __net_nfc_is_valid_target_handle(net_nfc_target_handle_s *handle);
static void __net_nfc_make_valid_target_handle(net_nfc_target_handle_s **handle);
static void __net_nfc_make_invalid_target_handle();

static bool net_nfc_nxp_controller_configure_discovery_stop();


////////////// STATIC FUNCTION DECLARE END //////////

static net_nfc_error_e _net_nfc_nxp_error_converter(uint16_t result)
{
	net_nfc_error_e retVal = NET_NFC_OK;
	result = result & 0xFF;
	switch (result)
	{
	case NFCSTATUS_SUCCESS :
		retVal = NET_NFC_OK;
		break;
	case NFCSTATUS_INVALID_PARAMETER :
		DEBUG_MSG("error : NFCSTATUS_INVALID_PARAMETER ");
		retVal = NET_NFC_NULL_PARAMETER;
		break;
	case NFCSTATUS_BUFFER_TOO_SMALL :
		DEBUG_MSG("error : NFCSTATUS_BUFFER_TOO_SMALL ");
		retVal = NET_NFC_BUFFER_TOO_SMALL;
		break;
	case NFCSTATUS_INVALID_DEVICE :
		DEBUG_MSG("error : NFCSTATUS_INVALID_DEVICE ");
		retVal = NET_NFC_COMMUNICATE_WITH_CONTROLLER_FAILED;
		break;
	case NFCSTATUS_RF_TIMEOUT :
		DEBUG_MSG("error : NFCSTATUS_RF_TIMEOUT ");
		retVal = NET_NFC_RF_TIMEOUT;
		break;
	case NFCSTATUS_RF_ERROR :
		DEBUG_MSG("error : NFCSTATUS_RF_ERROR ");
		retVal = NET_NFC_RF_ERROR;
		break;
	case NFCSTATUS_INSUFFICIENT_RESOURCES :
		DEBUG_MSG("error : NFCSTATUS_INSUFFICIENT_RESOURCES ");
		retVal = NET_NFC_ALLOC_FAIL;
		break;
	case NFCSTATUS_BOARD_COMMUNICATION_ERROR :
		DEBUG_MSG("error : NFCSTATUS_BOARD_COMMUNICATION_ERROR ");
		retVal = NET_NFC_COMMUNICATE_WITH_CONTROLLER_FAILED;
		break;
	case NFCSTATUS_INVALID_STATE :
		DEBUG_MSG("error : NFCSTATUS_INVALID_STATE ");
		retVal = NET_NFC_INVALID_STATE;
		break;
	case NFCSTATUS_NOT_INITIALISED :
		DEBUG_MSG("error : NFCSTATUS_NOT_INITIALISED ");
		retVal = NET_NFC_NOT_INITIALIZED;
		break;
	case NFCSTATUS_ALREADY_INITIALISED :
		DEBUG_MSG("error : NFCSTATUS_ALREADY_INITIALISED ");
		retVal = NET_NFC_ALREADY_INITIALIZED;
		break;
	case NFCSTATUS_FEATURE_NOT_SUPPORTED :
		DEBUG_MSG("error : NFCSTATUS_FEATURE_NOT_SUPPORTED ");
		retVal = NET_NFC_NOT_SUPPORTED;
		break;
	case NFCSTATUS_NOT_REGISTERED :
		DEBUG_MSG("error : NFCSTATUS_NOT_REGISTERED ");
		retVal = NET_NFC_NOT_REGISTERED;
		break;
	case NFCSTATUS_ALREADY_REGISTERED :
		DEBUG_MSG("error : NFCSTATUS_ALREADY_REGISTERED ");
		retVal = NET_NFC_ALREADY_REGISTERED;
		break;
	case NFCSTATUS_NOT_ALLOWED :
		DEBUG_MSG("error : NFCSTATUS_NOT_ALLOWED ");
		retVal = NET_NFC_NOT_ALLOWED_OPERATION;
		break;
	case NFCSTATUS_BUSY :
		DEBUG_MSG("error : NFCSTATUS_BUSY ");
		retVal = NET_NFC_BUSY;
		break;
	case NFCSTATUS_INVALID_REMOTE_DEVICE :
		DEBUG_MSG("error : NFCSTATUS_INVALID_REMOTE_DEVICE ");
		retVal = NET_NFC_INVALID_HANDLE;
		break;
	case NFCSTATUS_SMART_TAG_FUNC_NOT_SUPPORTED :
		DEBUG_MSG("error : NFCSTATUS_SMART_TAG_FUNC_NOT_SUPPORTED ");
		retVal = NET_NFC_NOT_SUPPORTED;
		break;
	case NFCSTATUS_READ_FAILED :
		DEBUG_MSG("error : NFCSTATUS_READ_FAILED ");
		retVal = NET_NFC_TAG_READ_FAILED;
		break;
	case NFCSTATUS_WRITE_FAILED :
		DEBUG_MSG("error : NFCSTATUS_WRITE_FAILED ");
		retVal = NET_NFC_TAG_WRITE_FAILED;
		break;
	case NFCSTATUS_NO_NDEF_SUPPORT :
		DEBUG_MSG("error : NFCSTATUS_NO_NDEF_SUPPORT ");
		retVal = NET_NFC_NO_NDEF_SUPPORT;
		break;
	case NFCSTATUS_EOF_NDEF_CONTAINER_REACHED :
		DEBUG_MSG("error : NFCSTATUS_EOF_NDEF_CONTAINER_REACHED ");
		retVal = NET_NFC_INSUFFICIENT_STORAGE;
		break;
	case NFCSTATUS_INVALID_RECEIVE_LENGTH :
		DEBUG_MSG("error : NFCSTATUS_INVALID_RECEIVE_LENGTH ");
		retVal = NET_NFC_OPERATION_FAIL;
		break;
	case NFCSTATUS_INVALID_FORMAT :
		DEBUG_MSG("error : NFCSTATUS_INVALID_FORMAT ");
		retVal = NET_NFC_INVALID_FORMAT;
		break;
	case NFCSTATUS_INSUFFICIENT_STORAGE :
		DEBUG_MSG("error : NFCSTATUS_INSUFFICIENT_STORAGE ");
		retVal = NET_NFC_INSUFFICIENT_STORAGE;
		break;
	case NFCSTATUS_FORMAT_ERROR :
		DEBUG_MSG("error : NFCSTATUS_FORMAT_ERROR ");
		retVal = NET_NFC_INVALID_FORMAT;
		break;
	case NFCSTATUS_TARGET_LOST :
		DEBUG_MSG("error : TARGET IS MOVED AWAY");
		retVal = NET_NFC_TARGET_IS_MOVED_AWAY;
		break;
	case NFCSTATUS_FAILED :
		DEBUG_MSG("error : FATAL ERROR");
		retVal = NET_NFC_OPERATION_FAIL;
		break;
	case NFCSTATUS_TARGET_NOT_CONNECTED :
		DEBUG_MSG("error : NFCSTATUS_TARGET_NOT_CONNECTED");
		retVal = NET_NFC_NOT_CONNECTED;
		break;
	case NFCSTATUS_PENDING :
		retVal = NET_NFC_BUSY;
		break;
		// Not understanded Error codes
	case NFCSTATUS_MORE_INFORMATION :
	case NFCSTATUS_MULTIPLE_PROTOCOLS : // this is not error
	case NFCSTATUS_MULTIPLE_TAGS :
	case NFCSTATUS_DESELECTED : // This is event
	case NFCSTATUS_RELEASED :

		// Below error codes should not be returned to Client APP
	default :
		DEBUG_MSG("Unkown Error codes is found, %d", result);
		retVal = NET_NFC_UNKNOWN_ERROR;
		break;
	}

	return retVal;
}

static int net_nfc_nxp_controller_convert_llcp_error_codes(uint16_t type)
{
	net_nfc_error_e retVal;
	switch (type)
	{
	case PHFRINFC_LLCP_ERR_DISCONNECTED :
		DEBUG_MSG("llcp error code converter : PHFRINET_NFC_LLCP_ERR_DISCONNECTED");
		retVal = NET_NFC_LLCP_SOCKET_DISCONNECTED;
		break;
	case PHFRINFC_LLCP_ERR_FRAME_REJECTED :
		DEBUG_MSG("llcp error code converter : PHFRINET_NFC_LLCP_ERR_FRAME_REJECTED");
		retVal = NET_NFC_LLCP_SOCKET_FRAME_REJECTED;
		break;
	case PHFRINFC_LLCP_ERR_BUSY_CONDITION :
	case PHFRINFC_LLCP_ERR_NOT_BUSY_CONDITION :
		DEBUG_MSG("llcp error code converter : BUSY CONDITION");
	default :
		DEBUG_MSG("Unkown Error codes is found, %d", type);
		retVal = NET_NFC_UNKNOWN_ERROR;
		break;
	}
	return retVal;
}

static int net_nfc_nxp_controller_convert_target_type(int type, phLibNfc_RemoteDevList_t *TagInfo)
{

	/*

	SAK =>

	case 0x09: // Mini
	case 0x08: // 1K
	case 0x18: // 4K
	case 0x88: // Infineon 1K
	case 0x98: // Pro 4K
	case 0xB8: // Pro 4K
	case 0x28: // 1K emulation
	case 0x38: // 4K emulation

	*/

	int convert = 0;

	DEBUG_MSG("before convert = [%d]", type);

	switch (type)
	{
	case phNfc_ePICC_DevType :
		convert = NET_NFC_GENERIC_PICC;
		break;
	case phNfc_eISO14443_A_PICC :
		{
			if (TagInfo->psRemoteDevInfo->RemoteDevInfo.Iso14443A_Info.Sak == 0x20)
			{
				convert = NET_NFC_MIFARE_DESFIRE_PICC;
			}
			else
			{
				convert = NET_NFC_ISO14443_A_PICC;
			}

		}
		break;
	case phNfc_eISO14443_4A_PICC :
		convert = NET_NFC_ISO14443_4A_PICC;
		break;
	case phNfc_eISO14443_3A_PICC :
		convert = NET_NFC_ISO14443_3A_PICC;
		break;
	case phNfc_eMifare_PICC :
		{
			if (TagInfo->psRemoteDevInfo->RemoteDevInfo.Iso14443A_Info.Sak == 0x09)
			{
				convert = NET_NFC_MIFARE_MINI_PICC;
			}
			else if (TagInfo->psRemoteDevInfo->RemoteDevInfo.Iso14443A_Info.Sak == 0x08)
			{
				convert = NET_NFC_MIFARE_1K_PICC;
			}
			else if (TagInfo->psRemoteDevInfo->RemoteDevInfo.Iso14443A_Info.Sak == 0x18)
			{
				convert = NET_NFC_MIFARE_4K_PICC;
			}
			else if (TagInfo->psRemoteDevInfo->RemoteDevInfo.Iso14443A_Info.Sak == 0x00)
			{
				convert = NET_NFC_MIFARE_ULTRA_PICC;
			}

		}
		break;
	case phNfc_eISO14443_B_PICC :
		convert = NET_NFC_ISO14443_B_PICC;
		break;
	case phNfc_eISO14443_4B_PICC :
		convert = NET_NFC_ISO14443_4B_PICC;
		break;
	case phNfc_eISO14443_BPrime_PICC :
		convert = NET_NFC_ISO14443_BPRIME_PICC;
		break;
	case phNfc_eFelica_PICC :
		convert = NET_NFC_FELICA_PICC;
		break;
	case phNfc_eJewel_PICC :
		convert = NET_NFC_JEWEL_PICC;
		break;
	case phNfc_eISO15693_PICC :
		convert = NET_NFC_ISO15693_PICC;
		break;
	case phNfc_eNfcIP1_Target :
		convert = NET_NFC_NFCIP1_TARGET;
		break;
	case phNfc_eNfcIP1_Initiator :
		convert = NET_NFC_NFCIP1_INITIATOR;
		break;
	case phNfc_eUnknown_DevType :
	case phNfc_eISO14443_A_PCD :
	case phNfc_eISO14443_B_PCD :
	case phNfc_eISO14443_BPrime_PCD :
	case phNfc_eFelica_PCD :
	case phNfc_eJewel_PCD :
	case phNfc_eISO15693_PCD :
	case phNfc_ePCD_DevType :
	case phNfc_eInvalid_DevType :
	default :
		convert = NET_NFC_UNKNOWN_TARGET;
		break;
	}

	DEBUG_MSG("after convert = [%d]", convert);

	return convert;
}

static socket_info_s *_net_nfc_get_available_socket_slot()
{
	int idx = 0;
	for (; idx < PHFRINFC_LLCP_NB_SOCKET_MAX; idx++)
	{
		if (socket_info_array[idx].isValid == false)
		{
			memset(&(socket_info_array[idx]), 0x00, sizeof(socket_info_s));
			socket_info_array[idx].my_index = idx;
			socket_info_array[idx].isValid = true;
			_net_nfc_util_alloc_mem(socket_info_array[idx].data.buffer, NET_NFC_MAX_LLCP_SOCKET_BUFFER);
			socket_info_array[idx].data.length = NET_NFC_MAX_LLCP_SOCKET_BUFFER;
			return &(socket_info_array[idx]);
		}
	}

	return NULL;
}

static void _net_nfc_remove_socket_slot(net_nfc_llcp_socket_t socket)
{
	int idx = 0;

	for (; idx < PHFRINFC_LLCP_NB_SOCKET_MAX; idx++)
	{
		if (socket_info_array[idx].isValid == true &&
			socket_info_array[idx].socket_handle == socket)
		{
			if (socket_info_array[idx].data.buffer != NULL)
			{
				free(socket_info_array[idx].data.buffer);
				socket_info_array[idx].data.buffer = NULL;
			}
			if (socket_info_array[idx].context != NULL)
			{
				free(socket_info_array[idx].context);
				socket_info_array[idx].context = NULL;
			}

			socket_info_array[idx].isValid = false;
			socket_info_array[idx].socket_handle = 0;
		}
	}
}

static socket_info_s *_net_nfc_find_server_socket(net_nfc_llcp_socket_t socket)
{
	int idx = 0;
	for (; idx < PHFRINFC_LLCP_NB_SOCKET_MAX; idx++)
	{
		if (socket_info_array[idx].socket_handle == socket && socket_info_array[idx].isValid == true)
		{
			return &(socket_info_array[idx]);
		}
	}

	return NULL;
}

static void _net_nfc_reset_socket_array()
{
	int idx = 0;

	memset(socket_info_array, 0x00, sizeof(socket_info_array));

	for (; idx < PHFRINFC_LLCP_NB_SOCKET_MAX; idx++)
	{
		socket_info_array[idx].isValid = false;
	}
}

void _net_nfc_phLibNfc_Mgt_IoCtl_cb(void *pContext, phNfc_sData_t *pOutParam, NFCSTATUS Status)
{
	controller_context_s *context = NULL;

	if (pContext != NULL)
	{
		context = (controller_context_s *)pContext;
		context->result = (int)Status;
	}

	if (Status != NFCSTATUS_SUCCESS)
	{
		DEBUG_MSG("IOCTL Error: [0x%x] \n", Status);
	}

	if (context != NULL)
	{
		NET_NFC_OEM_CONTROLLER_SIGNAL(context->controller_cond);
	}
}

///////////////////////////////////////////////// callback function ////////////////////////////////////////////////////////

static void _net_nfc_format_remote_dev_ndef_cb(void *pContext, NFCSTATUS status)
{
	controller_context_s *context = NULL;

	if (pContext != NULL)
	{
		context = (controller_context_s *)pContext;
		context->result = (int)status;
	}

	if (status == NFCSTATUS_SUCCESS)
	{
		DEBUG_MSG("format is successful");
	}
	else
	{
		DEBUG_MSG("format is failed = [0x%x]", status);
	}

	if (context != NULL)
	{
		NET_NFC_OEM_CONTROLLER_SIGNAL(context->controller_cond);
	}
}

static void _net_nfc_init_cb(void *pContext, NFCSTATUS status)
{
	controller_context_s *context = NULL;

	if (pContext != NULL)
	{
		context = (controller_context_s *)pContext;
		context->result = (int)status;
	}

	if (status == NFCSTATUS_SUCCESS)
	{
		DEBUG_MSG("stack initialization is successful");
	}
	else
	{
		DEBUG_MSG("stack initialization is failed = [0x%x]", status);
	}

	if (context != NULL)
	{
		NET_NFC_OEM_CONTROLLER_SIGNAL(context->controller_cond);
	}
}

static void _net_nfc_deinit_cb(void *pContext, NFCSTATUS status)
{
	controller_context_s *context = NULL;

	if (pContext != NULL)
	{
		context = (controller_context_s *)pContext;
		context->result = (int)status;
	}

	if (status == NFCSTATUS_SUCCESS)
	{
		DEBUG_MSG("stack deinitialization is successful");
	}
	else
	{
		DEBUG_MSG("stack deinitialization is failed = [0x%x]", status);
	}

	if (context != NULL)
	{
		NET_NFC_OEM_CONTROLLER_SIGNAL(context->controller_cond);
	}
}


static void _net_nfc_firmware_cb(void *pContext, phNfc_sData_t *Outparam_Cb, NFCSTATUS status)
{
	controller_context_s *context = NULL;

	DEBUG_MSG("_net_nfc_firmware_cb call");

	if (pContext != NULL)
	{
		context = (controller_context_s *)pContext;
		*((int*)context->user_context) = status;
	}

	if (status == NFCSTATUS_SUCCESS)
	{
		DEBUG_MSG("FIRMWARE DOWNLOAD SUCCESS");

	}
	else
	{
		DEBUG_MSG("FIRMWARE DOWNLOAD FAIL [0x%x]\n", status);
	}

	if (context != NULL)
	{
		NET_NFC_OEM_CONTROLLER_SIGNAL(context->controller_cond);
	}
}

static void _net_nfc_configure_discovery_cb(void *pContext, NFCSTATUS status)
{
	controller_context_s *context = NULL;

	if (pContext != NULL)
	{
		context = (controller_context_s *)pContext;
		context->result = (int)status;
	}

	if (status == NFCSTATUS_SUCCESS)
	{
		DEBUG_MSG("configure discovery is successful");
	}
	else
	{
		DEBUG_ERR_MSG("configure discovery is fail");
	}

	if (context != NULL)
	{
		NET_NFC_OEM_CONTROLLER_SIGNAL(context->controller_cond);
	}
}

static void _net_nfc_remotedev_notification_cb(void *pContext, phLibNfc_RemoteDevList_t *psRemoteDevList, uint8_t uNofRemoteDev, NFCSTATUS status)
{
	if (status != NFCSTATUS_SUCCESS && status != NFCSTATUS_MULTIPLE_PROTOCOLS)
	{
		if (status == NFCSTATUS_DESELECTED)
		{
			DEBUG_MSG("Deselected by Remote Device = [0x%x] ", status);

			return;
		}
		else
		{
			DEBUG_MSG(" Tag dectection error =  [0x%x] ", status);

			if (g_nxp_controller_target_cb != NULL)
			{
				// Do we need to call check presency ?
				net_nfc_request_msg_t *req_msg = NULL;

				_net_nfc_util_alloc_mem(req_msg, sizeof(net_nfc_request_msg_t));
				if (req_msg == NULL)
				{
					return;
				}
				req_msg->length = sizeof(net_nfc_request_msg_t);
				req_msg->request_type = NET_NFC_MESSAGE_SERVICE_RESTART_POLLING_LOOP;

				g_nxp_controller_target_cb(req_msg, NULL);
			}

			return;
		}
	}

	switch (psRemoteDevList->psRemoteDevInfo->RemDevType)
	{
	case phNfc_eInvalid_DevType :
		{
			DEBUG_MSG(" remote read or invalid type or unknown type ");
			return;
		}

	default :
		break;
	}

	if (g_nxp_controller_target_cb != NULL)
	{
		net_nfc_target_handle_s *handle = NULL;
		uint8_t *buffer = NULL;
		uint32_t buffer_length = 0, num_of_keys, length, devType;
		net_nfc_request_target_detected_t *target_detected = NULL;
		int index = 0;

		DEBUG_MSG("device is detected count = [0x%x]", uNofRemoteDev);

		__net_nfc_make_valid_target_handle(&handle);
		if (handle == NULL)
		{
			return;
		}

		if (status == NFCSTATUS_MULTIPLE_PROTOCOLS)
		{
			index = 1;
		}

		devType = net_nfc_nxp_controller_convert_target_type(psRemoteDevList[index].psRemoteDevInfo->RemDevType, &(psRemoteDevList[index]));

		num_of_keys = net_nfc_nxp_controller_tag_device_info(&(psRemoteDevList[index].psRemoteDevInfo->RemoteDevInfo), psRemoteDevList[index].psRemoteDevInfo->RemDevType, &buffer, &buffer_length);

		length = sizeof(net_nfc_request_target_detected_t) + buffer_length;

		_net_nfc_util_alloc_mem(target_detected, length);
		if (target_detected == NULL)
		{
			return;
		}
		target_detected->length = length;
		target_detected->request_type = NET_NFC_MESSAGE_SERVICE_STANDALONE_TARGET_DETECTED;

		target_detected->handle = handle;
		handle->connection_id = psRemoteDevList[index].hTargetDev;

		target_detected->devType = devType;
		if (target_detected->devType == NET_NFC_NFCIP1_TARGET)
		{
			DEBUG_MSG("set llcp connection  type. remote device is target");
			handle->connection_type = NET_NFC_P2P_CONNECTION_TARGET;
		}
		else if (target_detected->devType == NET_NFC_NFCIP1_INITIATOR)
		{
			DEBUG_MSG("set llcp connection  type. remote device is initiator");
			handle->connection_type = NET_NFC_P2P_CONNECTION_INITIATOR;
		}
		else
		{
			DEBUG_MSG("set tag connection");
			handle->connection_type = NET_NFC_TAG_CONNECTION;
		}

		target_detected->number_of_keys = num_of_keys;
		if (buffer != NULL)
		{
			target_detected->target_info_values.length = buffer_length;
			memcpy(&target_detected->target_info_values.buffer, buffer, target_detected->target_info_values.length);
		}

		DEBUG_MSG("target info = [%d]", psRemoteDevList[index].psRemoteDevInfo->RemDevType);

		DEBUG_MSG("target info values length = [%d]", target_detected->target_info_values.length);

		DEBUG_MSG("connection type is = [%d]", handle->connection_type);

		g_nxp_controller_target_cb(target_detected, NULL);

		DEBUG_MSG("target detected is end. go to process command");
	}
}

static void _net_nfc_se_mode_cb(void *pContext, phLibNfc_Handle hSecureElement, NFCSTATUS status)
{
	controller_context_s *context = NULL;

	if (pContext != NULL)
	{
		context = (controller_context_s *)pContext;
		context->result = (int)status;
	}

	if (status == NFCSTATUS_SUCCESS)
	{
		DEBUG_MSG("set SE mode is successful");
	}
	else
	{
		DEBUG_MSG("set SE mode is fail");
	}

	if (context != NULL)
	{
		NET_NFC_OEM_CONTROLLER_SIGNAL(context->controller_cond);
	}
}

static void _net_nfc_se_notification_cb(void *pContext, phLibNfc_eSE_EvtType_t EventType, phLibNfc_Handle hSecureElement, phLibNfc_uSeEvtInfo_t *pSeEvtInfo, NFCSTATUS status)
{
	DEBUG_MSG(" transaction in SE type = [%d] secure handle element = [%d] status = [0x%x]", EventType, hSecureElement, status);

	if (pSeEvtInfo != NULL)
	{
		int i = 0;

		DEBUG_MSG("AID : ");

		for (; i < pSeEvtInfo->UiccEvtInfo.aid.length; i++)
		{
			DEBUG_MSG("[0x%x]", pSeEvtInfo->UiccEvtInfo.aid.buffer[i]);
		}
	}

	if (g_nxp_controller_se_cb != NULL)
	{
		net_nfc_request_se_event_t *se_event = NULL;

		_net_nfc_util_alloc_mem(se_event, sizeof(net_nfc_request_se_event_t));
		if (se_event == NULL)
		{
			return;
		}

		se_event->length = sizeof(net_nfc_request_se_event_t);

		se_event->aid.length = pSeEvtInfo->UiccEvtInfo.aid.length;
		if(se_event->aid.length > 0)
		{
			_net_nfc_util_alloc_mem(se_event->aid.buffer, se_event->aid.length);
			memcpy(se_event->aid.buffer, pSeEvtInfo->UiccEvtInfo.aid.buffer, se_event->aid.length);
		}

		se_event->param.length = pSeEvtInfo->UiccEvtInfo.param.length;
		if(se_event->param.length > 0)
		{
			_net_nfc_util_alloc_mem(se_event->param.buffer, se_event->param.length);
			memcpy(se_event->param.buffer, pSeEvtInfo->UiccEvtInfo.param.buffer, se_event->param.length);
		}

		switch (EventType)
		{
		case phLibNfc_eSE_EvtStartTransaction :
			se_event->request_type = NET_NFC_MESSAGE_SE_START_TRANSACTION;
			break;

		case phLibNfc_eSE_EvtEndTransaction :
			se_event->request_type = NET_NFC_MESSAGE_SE_END_TRANSACTION;
			break;

		case phLibNfc_eSE_EvtTypeTransaction :
			se_event->request_type = NET_NFC_MESSAGE_SE_TYPE_TRANSACTION;
			break;

		case phLibNfc_eSE_EvtConnectivity :
			se_event->request_type = NET_NFC_MESSAGE_SE_CONNECTIVITY;
			break;

		case phLibNfc_eSE_EvtFieldOn :
			se_event->request_type = NET_NFC_MESSAGE_SE_FIELD_ON;
			break;

		case phLibNfc_eSE_EvtFieldOff :
			se_event->request_type = NET_NFC_MESSAGE_SE_FIELD_OFF;
			break;

		default :
			se_event->request_type = NET_NFC_MESSAGE_SE_TYPE_TRANSACTION;
			break;
		}

		g_nxp_controller_se_cb((void *)se_event, NULL);

	}
}

static void _net_nfc_connect_cb(void *pContext, phLibNfc_Handle hRemoteDev, phLibNfc_sRemoteDevInformation_t *psRemoteDevInfo, NFCSTATUS status)
{
	controller_context_s *context = NULL;

	if (pContext != NULL)
	{
		context = (controller_context_s *)pContext;
		context->result = (int)status;
	}

	if (status == NFCSTATUS_SUCCESS)
	{
		DEBUG_MSG("connection is successful = [0x%x]", hRemoteDev);
	}
	else
	{
		DEBUG_MSG("connection is fail");
	}

	if (context != NULL)
	{
		NET_NFC_OEM_CONTROLLER_SIGNAL(context->controller_cond);
	}
}

static void _net_nfc_disconnect_cb(void *pContext, phLibNfc_Handle hRemoteDev, NFCSTATUS status)
{
	controller_context_s *context = NULL;

	if (pContext != NULL)
	{
		context = (controller_context_s *)pContext;
		context->result = (int)status;
	}

	if (status == NFCSTATUS_SUCCESS)
	{
		DEBUG_MSG("disconnection is successful");
	}
	else
	{
		DEBUG_MSG("disconnection is fail");
	}

	if (context != NULL)
	{
		NET_NFC_OEM_CONTROLLER_SIGNAL(context->controller_cond);
	}
}

static void _net_nfc_transceive_cb(void *pContext, phLibNfc_Handle hRemoteDev, phNfc_sData_t *pResBuffer, NFCSTATUS status)
{
	controller_context_s *context = NULL;

	context = (controller_context_s *)pContext;

	if (pResBuffer != NULL && pResBuffer->length > 0 && pResBuffer->buffer != NULL)
	{
		DEBUG_MSG("the received byte length = [%d]", pResBuffer->length);

		if (pContext != NULL)
		{
			data_s *data = (data_s *)context->user_context;
			if (data == NULL)
				return;

			context->result = (int)status;

			_net_nfc_util_alloc_mem(data->buffer, pResBuffer->length);
			if ((data)->buffer != NULL)
			{
				data->length = pResBuffer->length;
				memcpy((data)->buffer, pResBuffer->buffer, pResBuffer->length);

				DEBUG_MSG("mem copy");
			}
			else
			{
				DEBUG_MSG("_net_nfc_util_alloc_mem fail");
				return;
			}
		}
	}
	else
	{
		if (pContext != NULL)
		{
			context->result = (int)status;
		}
		if (status != NFCSTATUS_SUCCESS)
		{
			DEBUG_MSG("transceive is failed = [0x%x]", status);
		}
		else
		{
			DEBUG_MSG("transceive is success = [0x%x]", status);
		}
	}

	if (context != NULL)
	{
		NET_NFC_OEM_CONTROLLER_SIGNAL(context->controller_cond);
	}
}

static void _net_nfc_ndef_read_cb(void *pContext, NFCSTATUS status)
{
	controller_context_s *context = NULL;

	if (pContext != NULL)
	{
		context = (controller_context_s *)pContext;
		context->result = (int)status;
	}

	if (status == NFCSTATUS_SUCCESS)
	{
		DEBUG_MSG("ndef read is successful");
	}
	else
	{
		DEBUG_MSG("ndef read is failed = [0x%x]", status);
	}

	if (context != NULL)
	{
		NET_NFC_OEM_CONTROLLER_SIGNAL(context->controller_cond);
	}
}

static void _net_nfc_ndef_write_cb(void *pContext, NFCSTATUS status)
{
	controller_context_s *context = NULL;

	if (pContext != NULL)
	{
		context = (controller_context_s *)pContext;
		context->result = (int)status;
	}

	if (status == NFCSTATUS_SUCCESS)
	{
		DEBUG_MSG("write ndef is successful");
	}
	else
	{
		DEBUG_MSG("write ndef is failed = [0x%x]", status);
	}

	if (context != NULL)
	{
		NET_NFC_OEM_CONTROLLER_SIGNAL(context->controller_cond);
	}
}

static void _net_nfc_make_read_only_cb(void *pContext, NFCSTATUS status)
{
	controller_context_s *context = NULL;

	if (pContext != NULL)
	{
		context = (controller_context_s *)pContext;
		context->result = (int)status;
	}

	if (status == NFCSTATUS_SUCCESS)
	{
		DEBUG_MSG("make read only is successful");
	}
	else
	{
		DEBUG_MSG("make read only is failed = [0x%x]", status);
	}

	if (context != NULL)
	{
		NET_NFC_OEM_CONTROLLER_SIGNAL(context->controller_cond);
	}
}

static void _net_nfc_target_check_presence_cb(void *pContext, NFCSTATUS status)
{
	controller_context_s *context = NULL;

	if (pContext != NULL)
	{
		context = (controller_context_s *)pContext;
		context->result = (int)status;
	}

	if (status == NFCSTATUS_SUCCESS)
	{
		//DEBUG_MSG("target is present");
	}
	else
	{
		DEBUG_MSG("target is moved away = [0x%x]", status);
	}

	if (context != NULL)
	{
		NET_NFC_OEM_CONTROLLER_SIGNAL(context->controller_cond);
	}
}

static void _net_nfc_target_check_ndef_cb(void *pContext, phLibNfc_ChkNdef_Info_t Ndef_Info, NFCSTATUS status)
{
	controller_context_s *context = (controller_context_s *)pContext;

	if (context != NULL)
	{
		ndef_info_s *ndef_info = (ndef_info_s *)context->user_context;
		context->result = (int)status;

		if (status == NFCSTATUS_SUCCESS)
		{
			ndef_info->ndef_card_state = Ndef_Info.NdefCardState;
			ndef_info->max_data_size = Ndef_Info.MaxNdefMsgLength;
			ndef_info->real_data_size = Ndef_Info.ActualNdefMsgLength;

			DEBUG_MSG("target has ndef message : Real data [%d]", Ndef_Info.ActualNdefMsgLength);
		}
		else
		{
			ndef_info->ndef_card_state = 0;
			ndef_info->max_data_size = -1;
			ndef_info->real_data_size = -1;

			DEBUG_MSG("target does not has ndef message = [0x%x]", status);
		}
	}

	if (context != NULL)
	{
		NET_NFC_OEM_CONTROLLER_SIGNAL(context->controller_cond);
	}
}

////////////////////////////////////////////////////////////////////

/*
static void _net_nfc_nxp_set_mode_default()
{
	DEBUG_MSG("Set Config Default");

	// NFCIP will be detected by exernal reader
	// we will be able to detect and read TAG

	sADDConfig.PollDevInfo.PollEnabled = TRUE;

	sADDConfig.PollDevInfo.PollCfgInfo.EnableIso14443A = TRUE;
	sADDConfig.PollDevInfo.PollCfgInfo.EnableIso14443B = TRUE;
	sADDConfig.PollDevInfo.PollCfgInfo.EnableFelica212 = TRUE;
	sADDConfig.PollDevInfo.PollCfgInfo.EnableNfcActive = TRUE;
	sADDConfig.PollDevInfo.PollCfgInfo.EnableIso15693 = TRUE;

	sADDConfig.PollDevInfo.PollCfgInfo.DisableCardEmulation = FALSE;

	sADDConfig.NfcIP_Mode = phNfc_eP2P_ALL;
	sADDConfig.NfcIP_Tgt_Disable = FALSE;

	sADDConfig.Duration = 0xF4240 / 2; // Card Emulation 1
}

static void _net_nfc_nxp_set_mode_off()
{
	DEBUG_MSG("Set Config OFF");
	sADDConfig.PollDevInfo.PollEnabled = FALSE;

	sADDConfig.PollDevInfo.PollCfgInfo.EnableIso14443A = FALSE;
	sADDConfig.PollDevInfo.PollCfgInfo.EnableIso14443B = FALSE;
	sADDConfig.PollDevInfo.PollCfgInfo.EnableFelica212 = FALSE;
	sADDConfig.PollDevInfo.PollCfgInfo.EnableNfcActive = FALSE;
	sADDConfig.PollDevInfo.PollCfgInfo.EnableIso15693 = FALSE;
	sADDConfig.PollDevInfo.PollCfgInfo.DisableCardEmulation = TRUE;

	sADDConfig.NfcIP_Mode = phNfc_eDefaultP2PMode;
	sADDConfig.NfcIP_Tgt_Disable = TRUE;
	sADDConfig.Duration = 0xF4240 / 2; // Card Emulation 1

}

static void _net_nfc_nxp_set_mode_reader()
{
	DEBUG_MSG("Set Config Reader");
	sADDConfig.PollDevInfo.PollEnabled = FALSE;

	sADDConfig.PollDevInfo.PollCfgInfo.EnableIso14443A = TRUE;
	sADDConfig.PollDevInfo.PollCfgInfo.EnableIso14443B = TRUE;
	sADDConfig.PollDevInfo.PollCfgInfo.EnableFelica212 = TRUE;
	sADDConfig.PollDevInfo.PollCfgInfo.EnableNfcActive = FALSE;
	sADDConfig.PollDevInfo.PollCfgInfo.EnableIso15693 = TRUE;

	sADDConfig.PollDevInfo.PollCfgInfo.DisableCardEmulation = TRUE;

	sADDConfig.NfcIP_Mode = phNfc_eDefaultP2PMode;
	sADDConfig.NfcIP_Tgt_Disable = FALSE;
	sADDConfig.Duration = 0xF4240 / 2; // Card Emulation 1
}


static void _net_nfc_nxp_set_mode_p2p()
{
	DEBUG_MSG("Set Config P2P");

	sADDConfig.PollDevInfo.PollEnabled = TRUE;

	sADDConfig.PollDevInfo.PollCfgInfo.EnableIso14443A = FALSE;
	sADDConfig.PollDevInfo.PollCfgInfo.EnableIso14443B = FALSE;
	sADDConfig.PollDevInfo.PollCfgInfo.EnableFelica212 = FALSE;
	sADDConfig.PollDevInfo.PollCfgInfo.EnableNfcActive = TRUE;
	sADDConfig.PollDevInfo.PollCfgInfo.EnableIso15693 = FALSE;

	// is it right to change card emulation to TRUE ??? // check it
	sADDConfig.PollDevInfo.PollCfgInfo.DisableCardEmulation = FALSE;

	sADDConfig.NfcIP_Mode = phNfc_eP2P_ALL;
	sADDConfig.NfcIP_Tgt_Disable = FALSE;

	sADDConfig.Duration = 0xF4240 / 2; // Card Emulation 1
}


static void _net_nfc_nxp_set_mode_paymentonly()
{
	DEBUG_MSG("Set Config card emulation");

	sADDConfig.PollDevInfo.PollEnabled = FALSE;

	sADDConfig.PollDevInfo.PollCfgInfo.EnableIso14443A = FALSE;
	sADDConfig.PollDevInfo.PollCfgInfo.EnableIso14443B = FALSE;
	sADDConfig.PollDevInfo.PollCfgInfo.EnableFelica212 = FALSE;
	sADDConfig.PollDevInfo.PollCfgInfo.EnableNfcActive = TRUE;
	sADDConfig.PollDevInfo.PollCfgInfo.EnableIso15693 = FALSE;
	sADDConfig.PollDevInfo.PollCfgInfo.DisableCardEmulation = FALSE;

	sADDConfig.NfcIP_Mode = phNfc_eDefaultP2PMode;
	sADDConfig.NfcIP_Tgt_Disable = TRUE;
	sADDConfig.Duration = 0xF4240 / 2; // Card Emulation 1

}
*/

static bool _net_nfc_nxp_check_pprom_is_completed()
{
	int vconf_val = false;

	if (is_EEPROM_writen)
		return true;

	if (vconf_get_bool(NET_NFC_EEPROM_WRITEN, &vconf_val) != 0)
	{
		vconf_val = false;
	}

	if (vconf_val == true)
		return true;

	return false;
}

static void _net_nfc_nxp_set_pprom_is_completed()
{
	vconf_set_bool(NET_NFC_EEPROM_WRITEN, true);
	is_EEPROM_writen = true;
}

static bool _net_nfc_dal_config()
{
	phLibNfc_sConfig_t Config;
	int nClientID;

	NET_NFC_OEM_CONTROLLER_LOCK;

	if ((nClientID = InitMessageQueue()) == 0)
	{
		NET_NFC_OEM_CONTROLLER_UNLOCK;
		DEBUG_MSG("failed to init message Q");
		return NFCSTATUS_INSUFFICIENT_RESOURCES;
	}

	Config.nClientId = nClientID;

	if (phLibNfc_Mgt_ConfigureDriver(&Config, &psHwRef.p_board_driver) == NFCSTATUS_SUCCESS)
	{
		NET_NFC_OEM_CONTROLLER_UNLOCK;
		DEBUG_MSG("_net_nfc_dal_config success");
		return true;
	}
	else
	{
		NET_NFC_OEM_CONTROLLER_UNLOCK;
		DEBUG_MSG("failed to configure driver");
		return false;
	}
}

static void _net_nfc_nxp_disable_irq_timeout()
{

	int idx = 0;
	phNfc_sData_t outData;
	phNfc_sData_t inData;

	uint8_t outBuffer[16];
	uint8_t inBuffer[4];
	NFCSTATUS ret;

	inData.buffer = inBuffer;
	inData.length = 4;

	outData.buffer = outBuffer;
	outData.length = 16;

	if (_net_nfc_nxp_check_pprom_is_completed())
	{
		DEBUG_MSG("eeprom is already writed");
		return;
	}

/*
	uint8_t EEDATA_Settings[EEDATA_SETTINGS_NUMBER][4] = {
		// RF Settings
		{0x00,0x9B,0xD1,0x0D} // Tx consumption higher than 0x0D (average 50mA)
		,{0x00,0x9B,0xD2,0x24} // GSP setting for this threshold
		,{0x00,0x9B,0xD3,0x0A} // Tx consumption higher than 0x0A (average 40mA)
		,{0x00,0x9B,0xD4,0x22} // GSP setting for this threshold
		,{0x00,0x9B,0xD5,0x08} // Tx consumption higher than 0x08 (average 30mA)
		,{0x00,0x9B,0xD6,0x1E} // GSP setting for this threshold
		,{0x00,0x9B,0xDD,0x1C} // GSP setting for this threshold
		,{0x00,0x9B,0x84,0x13} // ANACM2 setting
		,{0x00,0x99,0x81,0x7F} // ANAVMID setting PCD
		,{0x00,0x99,0x31,0x70} // ANAVMID setting PICC

		// Enable PBTF
		,{0x00,0x98,0x00,0x3F} // SECURE_ELEMENT_CONFIGURATION -Enable SE
		,{0x00,0x9F,0x09,0x00} // SWP_PBTF_RFU
		,{0x00,0x9F,0x0A,0x05} // SWP_PBTF_RFLD  --> RFLEVEL Detector for PBTF
		,{0x00,0x9E,0xD1,0xA1} //

		// Change RF Level Detector ANARFLDWU
		,{0x00,0x99,0x23,0x00} // Default Value is 0x01

		// Polling Loop Optimisation Detection  - 0x86 to enable - 0x00 to disable
		,{0x00,0x9E,0x74,0x00} // Default Value is 0x00, bits 0->2: sensitivity (0==maximal, 6==minimal), bits 3->6: RFU, bit 7: (0 -> disabled, 1 -> enabled)

		// Polling Loop - Card Emulation Timeout
		,{0x00,0x9F,0x35,0x14} // Time for which PN544 stays in Card Emulation mode after leaving RF field
		,{0x00,0x9F,0x36,0x60} // Default value 0x0411 = 50 ms ---> New Value : 0x1460 = 250 ms

		//LLC Timer diable
		,{0x00,0x9C,0x31,0x00} //
		,{0x00,0x9C,0x32,0x00} //
		,{0x00,0x9C,0x0C,0x00} //
		,{0x00,0x9C,0x0D,0x00} //
		,{0x00,0x9C,0x12,0x00} //
		,{0x00,0x9C,0x13,0x00} //
	};
*/

	// from ginger bread 2.3.4
	uint8_t EEDATA_Settings[EEDATA_SETTINGS_NUMBER][4] = {
		// DIFFERENTIAL_ANTENNA

		// RF Settings
		{0x00,0x9B,0xD1,0x0D} // Tx consumption higher than 0x0D (average 50mA)
		,{0x00,0x9B,0xD2,0x24} // GSP setting for this threshold
		,{0x00,0x9B,0xD3,0x0A} // Tx consumption higher than 0x0A (average 40mA)
		,{0x00,0x9B,0xD4,0x22} // GSP setting for this threshold
		,{0x00,0x9B,0xD5,0x08} // Tx consumption higher than 0x08 (average 30mA)
		,{0x00,0x9B,0xD6,0x1E} // GSP setting for this threshold
		,{0x00,0x9B,0xDD,0x1C} // GSP setting for this threshold
		,{0x00,0x9B,0x84,0x13} // ANACM2 setting
		,{0x00,0x99,0x81,0x7F} // ANAVMID setting PCD
		,{0x00,0x99,0x31,0x70} // ANAVMID setting PICC

		// Enable PBTF
		,{0x00,0x98,0x00,0x3F} // SECURE_ELEMENT_CONFIGURATION - No Secure Element
		,{0x00,0x9F,0x09,0x00} // SWP_PBTF_RFU
		,{0x00,0x9F,0x0A,0x05} // SWP_PBTF_RFLD  --> RFLEVEL Detector for PBTF
		,{0x00,0x9E,0xD1,0xA1} //

		// Change RF Level Detector ANARFLDWU
		,{0x00,0x99,0x23,0x00} // Default Value is 0x01

		// Polling Loop Optimisation Detection  - 0x86 to enable - 0x00 to disable

		,{0x00,0x9E,0x74,0x00} // Default Value is 0x00, bits 0->2: sensitivity (0==maximal, 6==minimal), bits 3->6: RFU, bit 7: (0 -> disabled, 1 -> enabled)

		// Polling Loop - Card Emulation Timeout
		,{0x00,0x9F,0x35,0x14} // Time for which PN544 stays in Card Emulation mode after leaving RF field
		,{0x00,0x9F,0x36,0x60} // Default value 0x0411 = 50 ms ---> New Value : 0x1460 = 250 ms

		//LLC Timer
		,{0x00,0x9C,0x31,0x00} //
		,{0x00,0x9C,0x32,0x00} //
		,{0x00,0x9C,0x0C,0x00} //
		,{0x00,0x9C,0x0D,0x00} //
		,{0x00,0x9C,0x12,0x00} //
		,{0x00,0x9C,0x13,0x00} //

		//WTX for LLCP communication
		,{0x00,0x98,0xA2,0x09} // Max value: 14 (default value: 09)

		//Murata Resonator setting
		,{0x00,0x9C,0x5C,0x06} // default 0x0140 = 1ms
		,{0x00,0x9C,0x5D,0x81} // 0x0681(= 5ms) is recommended value by Murata
		,{0x00,0x9F,0x19,0x60} // nxp test 3sec
		,{0x00,0x9F,0x1A,0x00} // nxp test 3sec
		//,{0x00,0x9F,0x19,0x82} // nxp test 1.6sec
		//,{0x00,0x9F,0x1A,0x35} // nxp test 1.6sec
	};

	DEBUG_MSG("writing eeprom");

	pthread_cond_t controller_cond = PTHREAD_COND_INITIALIZER;
	controller_context_s context = { &controller_cond, NULL, 0 };

	DEBUG_MSG("writing eeprom");

	for (idx = 0; idx < EEDATA_SETTINGS_NUMBER; idx++)
	{
		memcpy(inData.buffer, EEDATA_Settings[idx], inData.length);

		NET_NFC_OEM_CONTROLLER_LOCK;
		ret = phLibNfc_Mgt_IoCtl(psHwRef.p_board_driver, NFC_MEM_WRITE, &inData, &outData, _net_nfc_phLibNfc_Mgt_IoCtl_cb, (void *)&context);
		if (ret != NFCSTATUS_PENDING)
		{
			NET_NFC_OEM_CONTROLLER_UNLOCK;
			DEBUG_MSG("phLibNfc_Mgt_IoCtl returned [%d] _net_nfc_disable_irq_timeout is failed\n", ret);
			break;
		}
		NET_NFC_OEM_CONTROLLER_WAIT(&controller_cond);
		NET_NFC_OEM_CONTROLLER_UNLOCK;
	}

	DEBUG_MSG("_net_nfc_disable_irq_timeout is completed\n");
	_net_nfc_nxp_set_pprom_is_completed();
}

static void _net_nfc_nxp_controller_lock_init()
{
	pthread_mutexattr_t attr;

	pthread_mutexattr_init(&attr);
	pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE_NP);
	pthread_mutex_init(&g_controller_lock, &attr);
}

NET_NFC_EXPORT_API bool onload(net_nfc_oem_interface_s *nxp_interfaces)
{
	nxp_interfaces->init = net_nfc_nxp_controller_init;
	nxp_interfaces->deinit = net_nfc_nxp_controller_deinit;
	nxp_interfaces->register_listener = net_nfc_nxp_controller_register_listener;
	nxp_interfaces->unregister_listener = net_nfc_nxp_controller_unregister_listener;
	nxp_interfaces->get_firmware_version = net_nfc_nxp_controller_get_firmware_version;
	nxp_interfaces->check_firmware_version = net_nfc_nxp_controller_check_firmware_version;
	nxp_interfaces->update_firmeware = net_nfc_nxp_controller_update_firmware;
	nxp_interfaces->get_stack_information = net_nfc_nxp_controller_get_stack_information;
	nxp_interfaces->configure_discovery = net_nfc_nxp_controller_configure_discovery;
	nxp_interfaces->get_secure_element_list = net_nfc_nxp_controller_get_secure_element_list;
	nxp_interfaces->set_secure_element_mode = net_nfc_nxp_controller_set_secure_element_mode;
	nxp_interfaces->connect = net_nfc_nxp_controller_connect;
	nxp_interfaces->disconnect = net_nfc_nxp_controller_disconnect;
	nxp_interfaces->check_ndef = net_nfc_nxp_controller_check_ndef;
	nxp_interfaces->check_presence = net_nfc_nxp_controller_check_target_presence;
	nxp_interfaces->read_ndef = net_nfc_nxp_controller_read_ndef;
	nxp_interfaces->write_ndef = net_nfc_nxp_controller_write_ndef;
	nxp_interfaces->make_read_only_ndef = net_nfc_nxp_controller_make_read_only_ndef;
	nxp_interfaces->transceive = net_nfc_nxp_controller_transceive;
	nxp_interfaces->format_ndef = net_nfc_nxp_controller_format_ndef;
	nxp_interfaces->exception_handler = net_nfc_nxp_controller_exception_handler;
	nxp_interfaces->is_ready = net_nfc_nxp_controller_is_ready;

	nxp_interfaces->config_llcp = net_nfc_nxp_controller_llcp_config;
	nxp_interfaces->check_llcp_status = net_nfc_nxp_controller_llcp_check_llcp;
	nxp_interfaces->activate_llcp = net_nfc_nxp_controller_llcp_activate_llcp;
	nxp_interfaces->create_llcp_socket = net_nfc_nxp_controller_llcp_create_socket;
	nxp_interfaces->bind_llcp_socket = net_nfc_nxp_controller_llcp_bind;
	nxp_interfaces->listen_llcp_socket = net_nfc_nxp_controller_llcp_listen;
	nxp_interfaces->accept_llcp_socket = net_nfc_nxp_controller_llcp_accept;
	nxp_interfaces->connect_llcp_by_url = net_nfc_nxp_controller_llcp_connect_by_url;
	nxp_interfaces->connect_llcp = net_nfc_nxp_controller_llcp_connect;
	nxp_interfaces->disconnect_llcp = net_nfc_nxp_controller_llcp_disconnect;
	nxp_interfaces->close_llcp_socket = net_nfc_nxp_controller_llcp_socket_close;
	nxp_interfaces->recv_llcp = net_nfc_nxp_controller_llcp_recv;
	nxp_interfaces->send_llcp = net_nfc_nxp_controller_llcp_send;
	nxp_interfaces->recv_from_llcp = net_nfc_nxp_controller_llcp_recv_from;
	nxp_interfaces->send_to_llcp = net_nfc_nxp_controller_llcp_send_to;
	nxp_interfaces->reject_llcp = net_nfc_nxp_controller_llcp_reject;
	nxp_interfaces->get_remote_config = net_nfc_nxp_controller_llcp_get_remote_config;
	nxp_interfaces->get_remote_socket_info = net_nfc_nxp_controller_llcp_get_remote_socket_info;

	nxp_interfaces->sim_test = net_nfc_nxp_controller_sim_test;
	nxp_interfaces->prbs_test = net_nfc_nxp_controller_prbs_test;

	nxp_interfaces->test_mode_on = net_nfc_nxp_controller_test_mode_on;
	nxp_interfaces->test_mode_off = net_nfc_nxp_controller_test_mode_off;

	nxp_interfaces->support_nfc = net_nfc_nxp_controller_support_nfc;

	return true;
}

static bool net_nfc_nxp_controller_init(net_nfc_error_e *result)
{
	NFCSTATUS status = NFCSTATUS_SUCCESS;
	pthread_cond_t controller_cond = PTHREAD_COND_INITIALIZER;
	controller_context_s context = { &controller_cond, NULL, 0 };

	if (result == NULL)
	{
		return false;
	}

	*result = NET_NFC_OK;

	_net_nfc_nxp_controller_lock_init();

	DEBUG_MSG("Dal config \n");
	if (_net_nfc_dal_config() != true)
	{
		DEBUG_MSG("Port config is failed");
		return false;
	}

	DEBUG_MSG("start stack init \n");

	NET_NFC_OEM_CONTROLLER_LOCK;
	if ((status = phLibNfc_Mgt_Initialize(psHwRef.p_board_driver, _net_nfc_init_cb, (void *)&context)) != NFCSTATUS_PENDING)
	{
		NET_NFC_OEM_CONTROLLER_UNLOCK;
		DEBUG_MSG("stack init is failed = [0x%x] \n", status);

		g_stack_init_successful = false;

		return false;
	}
	else
	{
	        pthread_cond_wait(&controller_cond, &g_controller_lock);
		//NET_NFC_OEM_CONTROLLER_WAIT(&controller_cond);
		NET_NFC_OEM_CONTROLLER_UNLOCK;

		DEBUG_MSG("stack initialization is in progress");

		if (context.result != NFCSTATUS_SUCCESS)
		{
			DEBUG_MSG("stack init is failed = [0x%x] \n", context.result);
#if 0
			if (net_nfc_nxp_controller_update_firmware(result) == false)
			{
				DEBUG_MSG("UPDATE FAIL");
				return false;
			}
			else
			{
				DEBUG_MSG("UPDATE SUCCESS");
				net_nfc_nxp_controller_init(result);
				g_stack_init_successful = true;
				return true;
			}
#endif
			g_stack_init_successful = false;

			return false;
		}
	}
	DEBUG_MSG("Success Stack init  & Check the Firmware Version!!");

	if(net_nfc_nxp_controller_check_firmware_version(result) == TRUE)
	{
	       DEBUG_MSG("Need to update the firmware");

		if(net_nfc_nxp_controller_update_firmware(result) != TRUE)
		{
			DEBUG_MSG("Fail to download firmware!!");
			return false;
		}
		else/*Firmware Update Success & Retry init*/
		{
			DEBUG_MSG("Firmware UPDATE SUCCESS & Retry init!!");

			if (_net_nfc_dal_config() != true)
			{
				DEBUG_MSG("Port config is failed");
				return false;
			}

			NET_NFC_OEM_CONTROLLER_LOCK;
			if ((status = phLibNfc_Mgt_Initialize(psHwRef.p_board_driver, _net_nfc_init_cb, (void *)&context)) != NFCSTATUS_PENDING)
			{
				NET_NFC_OEM_CONTROLLER_UNLOCK;
				DEBUG_MSG("stack init is failed = [0x%x] \n", status);

				g_stack_init_successful = false;

				return false;
			}
			else
			{
				NET_NFC_OEM_CONTROLLER_WAIT(&controller_cond);
				NET_NFC_OEM_CONTROLLER_UNLOCK;

				DEBUG_MSG("New stack initialization is success!!");

				if (context.result != NFCSTATUS_SUCCESS)
				{
					DEBUG_MSG("stack init is failed = [0x%x] \n", context.result);

					g_stack_init_successful = false;

					return false;
				}
			}
		}
	}

	g_stack_init_successful = true;

	DEBUG_MSG("Stack init finished!!");
	return true;
}

static bool net_nfc_nxp_controller_deinit(void)
{
	NFCSTATUS status = NFCSTATUS_SUCCESS;
	pthread_cond_t controller_cond = PTHREAD_COND_INITIALIZER;
	controller_context_s context = { &controller_cond, NULL, 0 };

	_net_nfc_nxp_controller_lock_init();

	NET_NFC_OEM_CONTROLLER_LOCK;
	if ((status = phLibNfc_Mgt_DeInitialize(psHwRef.p_board_driver, _net_nfc_deinit_cb, /*NULL*/(void *)&context)) != NFCSTATUS_PENDING)
	{
		NET_NFC_OEM_CONTROLLER_UNLOCK;
		DEBUG_MSG("deinit is failed");
		return false;
	}
	else
	{
		pthread_cond_wait(&controller_cond, &g_controller_lock);
		NET_NFC_OEM_CONTROLLER_UNLOCK;
		DEBUG_MSG("deinit is success");
	}

	phLibNfc_Mgt_UnConfigureDriver(psHwRef.p_board_driver);
	DEBUG_MSG("unregister driver");

	return true;
}

static bool net_nfc_nxp_controller_register_listener(target_detection_listener_cb target_detection_listener, se_transaction_listener_cb se_transaction_listener, llcp_event_listener_cb llcp_event_listener, net_nfc_error_e *result)
{
	phLibNfc_Registry_Info_t reginfo = { 0, };
	int user_context = 0;
	NFCSTATUS status = NFCSTATUS_SUCCESS;

	if (result == NULL)
	{
		return false;
	}

	*result = NET_NFC_OK;

	// READER option :  the protocol which will be detected by READER.

	reginfo.MifareUL = TRUE;
	reginfo.MifareStd = TRUE;
	reginfo.ISO14443_4A = TRUE;
	reginfo.ISO14443_4B = TRUE;
	reginfo.Jewel = TRUE;
	reginfo.NFC = TRUE;
	reginfo.Felica = TRUE;
	reginfo.ISO15693 = TRUE;

	NET_NFC_OEM_CONTROLLER_LOCK;

	if ((status = phLibNfc_RemoteDev_NtfRegister(&reginfo, _net_nfc_remotedev_notification_cb, &user_context)) == NFCSTATUS_SUCCESS)
	{
		DEBUG_MSG("register Remote device callback is successful");
	}
	else
	{
		DEBUG_MSG("register Remote device callback is failed = [0x%x]", status);
	}

	if ((status = phLibNfc_SE_NtfRegister(_net_nfc_se_notification_cb, &user_context)) == NFCSTATUS_SUCCESS)
	{
		DEBUG_MSG("register SE callback is successful");
	}
	else
	{
		DEBUG_MSG("register SE callback is failed = [0x%x]", status);
	}

	g_nxp_controller_target_cb = target_detection_listener;
	g_nxp_controller_se_cb = se_transaction_listener;
	g_nxp_controller_llcp_cb = llcp_event_listener;

	NET_NFC_OEM_CONTROLLER_UNLOCK;

	return true;
}

static bool net_nfc_nxp_controller_unregister_listener()
{
	NET_NFC_OEM_CONTROLLER_LOCK;

	g_nxp_controller_target_cb = NULL;
	g_nxp_controller_se_cb = NULL;
	g_nxp_controller_llcp_cb = NULL;

	NET_NFC_OEM_CONTROLLER_UNLOCK;

	return true;
}

static bool net_nfc_nxp_controller_get_firmware_version(data_s **data, net_nfc_error_e *result)
{
	NFCSTATUS status = NFCSTATUS_SUCCESS;
	uint8_t version_info[11];
	bool ret = false;

	if (result == NULL)
		return ret;

	*result = NET_NFC_OK;

//	if ((status = net_nfc_nxp_controller_get_stack_information((net_nfc_stack_information_s *)version_info, result)) == true)
//	{
//		DEBUG_MSG("version [%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X]", version_info[0], version_info[1], version_info[2], version_info[3], version_info[4], version_info[5], version_info[6], version_info[7], version_info[8], version_info[9], version_info[10]);

		_net_nfc_util_alloc_mem(*data, sizeof(data_s));

		(*data)->length = 5;
		_net_nfc_util_alloc_mem((*data)->buffer, (*data)->length);

		snprintf((char *)(*data)->buffer, (*data)->length, "%d.%d", 1, 17);

		ret = true;
//	}
//	else
//	{
//		DEBUG_ERR_MSG("get_stack_capability failed \n");
//	}

	return ret;
}

static bool net_nfc_nxp_controller_check_firmware_version(net_nfc_error_e *result)
{
	NFCSTATUS status = NFCSTATUS_SUCCESS;

	uint8_t version_info[11];/*Already Installed*/
	uint32_t i = 0;

	if (result == NULL)
	{
		return false;
	}

	if(nxp_nfc_full_version == NULL)
	{
		DEBUG_MSG("=== There are no firmware file");
		return false;
	}

	*result = NET_NFC_OK;

	DEBUG_MSG("get stack capability");

	if ((status = net_nfc_nxp_controller_get_stack_information((net_nfc_stack_information_s *)version_info, result)) != true)
	{
		DEBUG_MSG("get_stack_capability failed \n");
		return false;
	}

	for (i = 0; i < 11; i++)
	{
		if (version_info[i] != nxp_nfc_full_version[i])
		{
			DEBUG_MSG("version_info[%d] = [%x]\n", i, version_info[i]);
			DEBUG_MSG("nxp_nfc_full_version[%d] = [%x]\n", i, nxp_nfc_full_version[i]);
			/*Plug-In's firmware version*/
			DEBUG_MSG("Need Update");
			return true;
		}
		DEBUG_MSG("[%d]'s hexa is same", i)
	}

	DEBUG_MSG("Don't Need Update");
	return false;
}

static bool net_nfc_nxp_controller_update_firmware(net_nfc_error_e *result)
{
	phNfc_sData_t InParam;
	phNfc_sData_t OutParam;
	int user_context = 0xff;
	NFCSTATUS status = NFCSTATUS_SUCCESS;
	pthread_cond_t controller_cond = PTHREAD_COND_INITIALIZER;
	controller_context_s context = { &controller_cond, &user_context };

	if (result == NULL)
	{
		return false;
	}

	*result = NET_NFC_OK;

	net_nfc_nxp_controller_deinit();

	if (_net_nfc_dal_config() != true)
	{
		DEBUG_MSG("Port config is failed");
		return false;
	}

	_net_nfc_util_alloc_mem(InParam.buffer, 1);
	InParam.length = 1;
	_net_nfc_util_alloc_mem(OutParam.buffer, 1);
	OutParam.length = 1;

	NET_NFC_OEM_CONTROLLER_LOCK;
	phLibNfc_Download_Mode();
	NET_NFC_OEM_CONTROLLER_UNLOCK;

	NET_NFC_OEM_CONTROLLER_LOCK;
	if ((status = phLibNfc_Mgt_IoCtl(psHwRef.p_board_driver, NFC_FW_DOWNLOAD, &InParam, &OutParam, _net_nfc_firmware_cb, (void *)&context)) != NFCSTATUS_PENDING)
	{
		DEBUG_MSG("Firmware update ERROR1");
		NET_NFC_OEM_CONTROLLER_UNLOCK;
		free(InParam.buffer);
		free(OutParam.buffer);
		return false;
	}
	else
	{
		DEBUG_MSG("phLibNfc_Mgt_IoCtl NFC_FW_DOWNLOAD status[%d]", status);

		pthread_cond_wait(&controller_cond, &g_controller_lock);
		//NET_NFC_OEM_CONTROLLER_WAIT(&controller_cond);

		DEBUG_MSG("user_context status[%d]", user_context);
		NET_NFC_OEM_CONTROLLER_UNLOCK;

		if (user_context != NFCSTATUS_SUCCESS)
		{
			DEBUG_MSG("Firmware update ERROR2");
			free(InParam.buffer);
			free(OutParam.buffer);
			return false;
		}
	}
	DEBUG_MSG("Firmware update SUCCESS");

	free(InParam.buffer);
	free(OutParam.buffer);

	return true;
}

/*static net_nfc_stack_information_s *_net_nfc_nxp_convert_stack_information(phLibNfc_StackCapabilities_t *stack_capa)
 {
 net_nfc_stack_information_s *stack_info;
	memcpy(stack_info.net_nfc_fw_version , stack_capa.psDevCapabilities.fw_version , sizeof(uint32_t));
 return stack_info;
 }
 */

static bool net_nfc_nxp_controller_get_stack_information(net_nfc_stack_information_s *stack_info, net_nfc_error_e *result)
{
	if (result == NULL)
	{
		return false;
	}

	DEBUG_MSG("net_nfc_nxp_controller_get_stack_information");

	*result = NET_NFC_OK;

	int user_context = 0;
	NFCSTATUS friReturn;

	phLibNfc_StackCapabilities_t stack_capa;

	NET_NFC_OEM_CONTROLLER_LOCK;
	if((friReturn = phLibNfc_Mgt_GetstackCapabilities (&stack_capa , (void *)&user_context)) != NFCSTATUS_SUCCESS)
	{
		NET_NFC_OEM_CONTROLLER_UNLOCK;
		DEBUG_MSG("net_nfc_nxp_controller_get_stack_information false");
		return false;
	}
	NET_NFC_OEM_CONTROLLER_UNLOCK;

	DEBUG_MSG("net_nfc_nxp_controller_get_stack_information Version = [%x]" , stack_capa.psDevCapabilities.full_version);


	// convert stack capa to stack info
	//stack_info = _net_nfc_nxp_convert_stack_information(&stack_capa);
	memcpy(stack_info ,&stack_capa.psDevCapabilities.full_version , 11*sizeof(uint8_t));
	DEBUG_MSG("net_nfc_nxp_controller_get_stack_information true");
	return true;
}

static void _net_nfc_nxp_set_add_config(net_nfc_event_filter_e config, phLibNfc_sADD_Cfg_t *ADD_config)
{
	DEBUG_MSG("set polling loop configure = [0x%x]", config);

	if (ADD_config == NULL)
		return;

	if (config & NET_NFC_ISO14443A_ENABLE)
	{
		DEBUG_MSG("turn on ISO14443A");
		ADD_config->PollDevInfo.PollCfgInfo.EnableIso14443A = TRUE;
	}
	else
	{
		DEBUG_MSG("turn off ISO14443A");
		ADD_config->PollDevInfo.PollCfgInfo.EnableIso14443A = FALSE;
	}

	if (config & NET_NFC_ISO14443B_ENABLE)
	{
		DEBUG_MSG("turn on ISO14443B");
		ADD_config->PollDevInfo.PollCfgInfo.EnableIso14443B = TRUE;
	}
	else
	{
		DEBUG_MSG("turn off ISO14443B");
		ADD_config->PollDevInfo.PollCfgInfo.EnableIso14443B = FALSE;
	}

	if (config & NET_NFC_FELICA_ENABLE)
	{
		DEBUG_MSG("turn on Felica");
		ADD_config->PollDevInfo.PollCfgInfo.EnableFelica212 = TRUE;
		ADD_config->PollDevInfo.PollCfgInfo.EnableFelica424 = TRUE;
	}
	else
	{
		DEBUG_MSG("turn off Felica");
		ADD_config->PollDevInfo.PollCfgInfo.EnableFelica212 = FALSE;
		ADD_config->PollDevInfo.PollCfgInfo.EnableFelica424 = FALSE;
	}

	if (config & NET_NFC_ISO15693_ENABLE)
	{
		DEBUG_MSG("turn on ISO15693");
		ADD_config->PollDevInfo.PollCfgInfo.EnableIso15693 = TRUE;
	}
	else
	{
		DEBUG_MSG("turn off ISO15693");
		ADD_config->PollDevInfo.PollCfgInfo.EnableIso15693 = FALSE;
	}

	if (config & NET_NFC_IP_ENABLE)
	{
		DEBUG_MSG("turn on NFCIP");
		ADD_config->PollDevInfo.PollCfgInfo.EnableNfcActive = TRUE;
		ADD_config->PollDevInfo.PollCfgInfo.DisableCardEmulation = FALSE;
		ADD_config->NfcIP_Mode = phNfc_eP2P_ALL;
		ADD_config->NfcIP_Tgt_Disable = FALSE;
	}
	else
	{
		DEBUG_MSG("turn off NFCIP");
		ADD_config->PollDevInfo.PollCfgInfo.EnableNfcActive = FALSE;
		ADD_config->PollDevInfo.PollCfgInfo.DisableCardEmulation = FALSE;
		ADD_config->NfcIP_Mode = 0;
		ADD_config->NfcIP_Tgt_Disable = TRUE;
	}

	ADD_config->Duration = 0xF4240; // Card Emulation 1
}

static bool net_nfc_nxp_controller_configure_discovery(net_nfc_discovery_mode_e mode, net_nfc_event_filter_e config, net_nfc_error_e *result)
{
	NFCSTATUS friReturn = NFCSTATUS_SUCCESS;
	pthread_cond_t controller_cond = PTHREAD_COND_INITIALIZER;
	controller_context_s context = { &controller_cond, NULL, 0 };
	phLibNfc_sADD_Cfg_t AddConfig;

	if (result == NULL)
	{
		return false;
	}

	*result = NET_NFC_OK;

	if (mode == NET_NFC_DISCOVERY_MODE_START || mode == NET_NFC_DISCOVERY_MODE_STOP)
	{
		DEBUG_MSG("stop and start is not supported in NXP solution");
		*result = NET_NFC_OUT_OF_BOUND;
		return false;
	}

	DEBUG_MSG("run poll loop");

	_net_nfc_nxp_set_add_config(config, &AddConfig);

	NET_NFC_OEM_CONTROLLER_LOCK;

	__net_nfc_make_invalid_target_handle();
	friReturn = phLibNfc_Mgt_ConfigureDiscovery(mode, AddConfig, _net_nfc_configure_discovery_cb, (void *)&context);
	if (friReturn != NFCSTATUS_PENDING)
	{
		NET_NFC_OEM_CONTROLLER_UNLOCK;

		DEBUG_MSG("discovering config is error");
		*result = _net_nfc_nxp_error_converter(friReturn);
		return false;
	}
	else
	{
		NET_NFC_OEM_CONTROLLER_WAIT(&controller_cond);
		NET_NFC_OEM_CONTROLLER_UNLOCK;

		DEBUG_MSG("discovering config is end");

		if (context.result != NFCSTATUS_SUCCESS)
		{
			*result = _net_nfc_nxp_error_converter(context.result);
			return false;
		}
		else
		{
			return true;
		}
	}
}

static bool net_nfc_nxp_controller_get_secure_element_list(net_nfc_secure_element_info_s *list, int *count, net_nfc_error_e *result)
{
	if (result == NULL)
	{
		return false;
	}

	*result = NET_NFC_OK;

	if (list == NULL || count == NULL)
	{
		*result = NET_NFC_NULL_PARAMETER;
		return false;
	}

	if (g_se_no > 0)
	{
		*count = g_se_no;

		int i = 0;

		for (; i < g_se_no; i++)
		{
			_net_nfc_util_alloc_mem(list[i].handle, sizeof(net_nfc_target_handle_s));
			if (list[i].handle != NULL)
			{
				list[i].handle->connection_id = g_se_list[i].hSecureElement;
				list[i].handle->connection_type = NET_NFC_SE_CONNECTION;
			}

			list[i].secure_element_type = g_se_list[i].eSE_Type;
			list[i].secure_element_state = g_se_list[i].eSE_CurrentState;

			if (list[i].secure_element_type == SECURE_ELEMENT_TYPE_ESE)
			{
				DEBUG_MSG("ESE is found");
			}
			else if (list[i].secure_element_type == SECURE_ELEMENT_TYPE_UICC)
			{
				DEBUG_MSG("UICC is found");
			}
			else
			{
				DEBUG_MSG("unknown type is found = [%d]", list[i].secure_element_type);
			}
		}

		return true;
	}
	else
	{
		NFCSTATUS status = NFCSTATUS_SUCCESS;

		NET_NFC_OEM_CONTROLLER_LOCK;

		if ((status = phLibNfc_SE_GetSecureElementList(g_se_list, &g_se_no)) == NFCSTATUS_SUCCESS)
		{
			int i = 0;

			NET_NFC_OEM_CONTROLLER_UNLOCK;

			DEBUG_MSG("get secure item info is ok. item counts = [%d]", g_se_no);

			*count = g_se_no;

			for (; i < g_se_no; i++)
			{
				_net_nfc_util_alloc_mem(list[i].handle, sizeof(net_nfc_target_handle_s));
				if (list[i].handle != NULL)
				{
					list[i].handle->connection_id = g_se_list[i].hSecureElement;
					list[i].handle->connection_type = NET_NFC_SE_CONNECTION;
				}

				if (g_se_list[i].eSE_Type == phLibNfc_SE_Type_SmartMX)
				{
					DEBUG_MSG("ESE is found");
				}
				else if (g_se_list[i].eSE_Type == phLibNfc_SE_Type_UICC)
				{
					DEBUG_MSG("UICC is found");
				}
				else
				{
					DEBUG_MSG("unknown type is found = [%d]", g_se_list[i].eSE_Type);
				}

				list[i].secure_element_type = g_se_list[i].eSE_Type;
				list[i].secure_element_state = g_se_list[i].eSE_CurrentState;
			}

			return true;
		}
		else
		{
			NET_NFC_OEM_CONTROLLER_UNLOCK;

			*result = NET_NFC_NO_DATA_FOUND;

			DEBUG_MSG("there is no secure item error = [0x%x]", status);

			return false;
		}
	}
}

static bool net_nfc_nxp_controller_set_secure_element_mode(net_nfc_secure_element_type_e element_type, net_nfc_secure_element_mode_e mode, net_nfc_error_e *result)
{
	pthread_cond_t controller_cond = PTHREAD_COND_INITIALIZER;
	controller_context_s context = { &controller_cond, NULL, 0 };
	NFCSTATUS status = NFCSTATUS_SUCCESS;
	phLibNfc_eSE_ActivationMode act_mode = phLibNfc_SE_ActModeOff;
	net_nfc_secure_element_info_s info[PHLIBNFC_MAXNO_OF_SE] = { { 0 } };
	int se_count = 0;
	phLibNfc_Handle secure_element_handle = 0;

	if (result == NULL)
	{
		return false;
	}

	*result = NET_NFC_OK;

	if (net_nfc_nxp_controller_get_secure_element_list(info, &se_count, result) != true)
	{
		DEBUG_MSG("get secure element info is failed");
		return false;
	}
	else
	{
		int i = 0;
		for (; i < se_count; i++)
		{
			if (info[i].secure_element_type == element_type)
			{
				if (info[i].handle != NULL)
				{
					secure_element_handle = (phLibNfc_Handle)info[i].handle->connection_id;
					DEBUG_MSG("SE type is mached, handle = [0x%x]", secure_element_handle);
					break;
				}
				else
				{
					DEBUG_MSG("handle is not vaild");
					*result = NET_NFC_INVALID_HANDLE;

					return false;
				}
			}
		}

		if (i == se_count)
		{
			DEBUG_MSG("no matching SE type");
			*result = NET_NFC_NO_DATA_FOUND;

			return false;
		}
	}

	switch (mode)
	{
	case SECURE_ELEMENT_WIRED_MODE :
		act_mode = phLibNfc_SE_ActModeWired;
		break;
	case SECURE_ELEMENT_VIRTUAL_MODE :
		act_mode = phLibNfc_SE_ActModeVirtual;
		break;
	case SECURE_ELEMENT_OFF_MODE :
		act_mode = phLibNfc_SE_ActModeOff;
		break;
	default :
		act_mode = phLibNfc_SE_ActModeVirtual;
		break;
	}

	NET_NFC_OEM_CONTROLLER_LOCK;

	status = phLibNfc_SE_SetMode(secure_element_handle, act_mode, _net_nfc_se_mode_cb, (void *)&context);

	if (status == NFCSTATUS_PENDING)
	{
		NET_NFC_OEM_CONTROLLER_WAIT(&controller_cond);
		NET_NFC_OEM_CONTROLLER_UNLOCK;

		if (context.result != NFCSTATUS_SUCCESS)
		{
			DEBUG_MSG("set secure element mode is failed = [0x%x]", context.result);
			return false;
		}
	}
	else if (status == NFCSTATUS_SUCCESS)
	{
		NET_NFC_OEM_CONTROLLER_UNLOCK;

		if (context.result != NFCSTATUS_SUCCESS)
		{
			DEBUG_MSG("set secure element mode is failed = [0x%x]", context.result);
			return false;
		}
	}
	else
	{
		NET_NFC_OEM_CONTROLLER_UNLOCK;
		DEBUG_MSG("set SE is failed = [0x%x]", status);

		return false;
	}

	DEBUG_MSG("set secure element mode is good");
	return true;
}

static bool net_nfc_nxp_controller_check_target_presence(net_nfc_target_handle_s *handle, net_nfc_error_e *result)
{
	phLibNfc_Handle hTargetDev;
	NFCSTATUS friReturn = NFCSTATUS_SUCCESS;
	pthread_cond_t controller_cond = PTHREAD_COND_INITIALIZER;
	controller_context_s context = { &controller_cond, NULL, 0 };

	if (result == NULL)
	{
		return false;
	}

	*result = NET_NFC_OK;

	if (handle == NULL)
	{
		*result = NET_NFC_NULL_PARAMETER;
		return false;
	}

	hTargetDev = (phLibNfc_Handle)handle->connection_id;

	NET_NFC_OEM_CONTROLLER_LOCK;
	friReturn = phLibNfc_RemoteDev_CheckPresence(hTargetDev, _net_nfc_target_check_presence_cb, (void *)&context);
	if (friReturn == NFCSTATUS_PENDING)
	{
		NET_NFC_OEM_CONTROLLER_WAIT(&controller_cond);
		NET_NFC_OEM_CONTROLLER_UNLOCK;

		if (context.result == NFCSTATUS_SUCCESS)
		{
			return true;
		}
		else
		{
			*result = _net_nfc_nxp_error_converter(context.result);
		}
	}
	else
	{
		NET_NFC_OEM_CONTROLLER_UNLOCK;
		DEBUG_MSG("check presence operation is failed = [0x%x]", friReturn);
		*result = _net_nfc_nxp_error_converter(friReturn);
	}

	return false;
}

static bool net_nfc_nxp_controller_connect(net_nfc_target_handle_s *handle, net_nfc_error_e *result)
{
	phLibNfc_Handle hTargetDev;
	NFCSTATUS friReturn;
	pthread_cond_t controller_cond = PTHREAD_COND_INITIALIZER;
	controller_context_s context = { &controller_cond, NULL, 0 };

	if (result == NULL)
	{
		return false;
	}

	*result = NET_NFC_OK;

	if (handle == NULL)
	{
		*result = NET_NFC_NULL_PARAMETER;
		return false;
	}

	if (!__net_nfc_is_valid_target_handle(handle))
	{
		*result = NET_NFC_INVALID_HANDLE;
		return false;
	}

	hTargetDev = (phLibNfc_Handle)handle->connection_id;

	NET_NFC_OEM_CONTROLLER_LOCK;
	friReturn = phLibNfc_RemoteDev_Connect(hTargetDev, _net_nfc_connect_cb, (void *)&context);
	if (friReturn == NFCSTATUS_PENDING)
	{
		NET_NFC_OEM_CONTROLLER_WAIT(&controller_cond);
		NET_NFC_OEM_CONTROLLER_UNLOCK;
		if (context.result != NFCSTATUS_SUCCESS)
		{
			*result = _net_nfc_nxp_error_converter(context.result);
			return false;
		}

	}
	else
	{
		NET_NFC_OEM_CONTROLLER_UNLOCK;
		DEBUG_MSG("connecting to target is failed = [0x%x]", friReturn);
		*result = _net_nfc_nxp_error_converter(friReturn);
		return false;
	}

	return true;
}

static bool net_nfc_nxp_controller_disconnect(net_nfc_target_handle_s *handle, net_nfc_error_e *result)
{
	if (result == NULL)
	{
		return false;
	}

	*result = NET_NFC_OK;

	if (handle == NULL)
	{
		*result = NET_NFC_NULL_PARAMETER;
		return false;
	}

	if (!__net_nfc_is_valid_target_handle(handle))
	{
		*result = NET_NFC_INVALID_HANDLE;
		return false;
	}

	if (handle->connection_type == NET_NFC_P2P_CONNECTION_INITIATOR || handle->connection_type == NET_NFC_P2P_CONNECTION_TARGET)
	{
		int idx = 0;
		for (; idx < PHFRINFC_LLCP_NB_SOCKET_MAX; idx++)
		{
			if (socket_info_array[idx].isValid == true && socket_info_array[idx].socket_handle != 0)
			{
				net_nfc_error_e error = NET_NFC_OK;
				net_nfc_nxp_controller_llcp_socket_close(socket_info_array[idx].socket_handle, &error);
			}
		}
	}

	if (handle->connection_type == NET_NFC_P2P_CONNECTION_INITIATOR)
	{
		/* do nothing */
		DEBUG_MSG("I am Target. The remote device is initiator. refuse to call disconnect ");
		__net_nfc_make_invalid_target_handle();

		return true;
	}

	phLibNfc_Handle hTargetDev = (phLibNfc_Handle)handle->connection_id;

	*result = NET_NFC_OK;
	NFCSTATUS friReturn;

	phLibNfc_eReleaseType_t releaseType = NFC_INVALID_RELEASE_TYPE;

	switch (handle->connection_type)
	{
	case NET_NFC_TAG_CONNECTION :
		releaseType = NFC_DISCOVERY_CONTINUE;
		break;
	case NET_NFC_P2P_CONNECTION_INITIATOR :
	case NET_NFC_P2P_CONNECTION_TARGET :
		releaseType = NFC_DISCOVERY_RESTART;
		break;
	case NET_NFC_SE_CONNECTION :
		releaseType = NFC_SMARTMX_RELEASE;
		break;
	default :
		releaseType = NFC_DISCOVERY_CONTINUE;
		break;

	}

	// This handle is not useful anymore
	__net_nfc_make_invalid_target_handle();

	DEBUG_MSG("releaseType = [%d]", releaseType);

	pthread_cond_t controller_cond = PTHREAD_COND_INITIALIZER;
	controller_context_s context = { &controller_cond, NULL, 0 };

	NET_NFC_OEM_CONTROLLER_LOCK;

	DEBUG_MSG("call disconnect");

	friReturn = phLibNfc_RemoteDev_Disconnect(hTargetDev, releaseType, _net_nfc_disconnect_cb, (void *)&context);
	if (friReturn == NFCSTATUS_PENDING)
	{
		NET_NFC_OEM_CONTROLLER_WAIT(&controller_cond);
		NET_NFC_OEM_CONTROLLER_UNLOCK;

		if (context.result != NFCSTATUS_SUCCESS)
		{
			DEBUG_ERR_MSG("disconnect is failed = [0x%x]", context.result);
			*result = _net_nfc_nxp_error_converter(context.result);
			return false;
		}
	}
	else
	{
		NET_NFC_OEM_CONTROLLER_UNLOCK;
		DEBUG_MSG("disconnect is failed = [0x%x]", friReturn);
		*result = _net_nfc_nxp_error_converter(friReturn);
		return false;
	}

	return true;
}

static bool net_nfc_nxp_controller_check_ndef(net_nfc_target_handle_s *handle, uint8_t *ndef_card_state, int *max_data_size, int *real_data_size, net_nfc_error_e *result)
{
	phLibNfc_Handle hTargetDev;
	ndef_info_s ndef_info;
	NFCSTATUS friReturn = NFCSTATUS_SUCCESS;
	pthread_cond_t controller_cond = PTHREAD_COND_INITIALIZER;
	controller_context_s context = { &controller_cond, &ndef_info, 0 };


	if (result == NULL)
	{
		return false;
	}

	*result = NET_NFC_OK;

	if (handle == NULL || ndef_card_state == NULL || max_data_size == NULL || real_data_size == NULL || result == NULL)
	{
		*result = NET_NFC_NULL_PARAMETER;
		return false;
	}

	hTargetDev = (phLibNfc_Handle)handle->connection_id;

	ndef_info.ndef_card_state = 0;
	ndef_info.max_data_size = -1;
	ndef_info.real_data_size = -1;

	if (!__net_nfc_is_valid_target_handle(handle))
	{
		*result = NET_NFC_INVALID_HANDLE;
		return false;
	}

	NET_NFC_OEM_CONTROLLER_LOCK;
	friReturn = phLibNfc_Ndef_CheckNdef(hTargetDev, _net_nfc_target_check_ndef_cb, (void *)&context);
	if (friReturn == NFCSTATUS_PENDING)
	{
		NET_NFC_OEM_CONTROLLER_WAIT(&controller_cond);
		NET_NFC_OEM_CONTROLLER_UNLOCK;

		if (context.result != NFCSTATUS_SUCCESS)
		{
			*result = _net_nfc_nxp_error_converter(context.result);
			return false;
		}
		else
		{
			*ndef_card_state = ndef_info.ndef_card_state;
			*max_data_size = ndef_info.max_data_size;
			*real_data_size = ndef_info.real_data_size;

			DEBUG_MSG("Card State : [%d] MAX data size :[%d] actual data size = [%d]", ndef_info.ndef_card_state, ndef_info.max_data_size, ndef_info.real_data_size);

			return true;
		}
	}
	else
	{
		NET_NFC_OEM_CONTROLLER_UNLOCK;
		DEBUG_MSG("operation is failed = [0x%x]", friReturn);
		*result = _net_nfc_nxp_error_converter(friReturn);
		return false;
	}

	return false;
}

static bool net_nfc_nxp_controller_make_read_only_ndef(net_nfc_target_handle_s *handle, net_nfc_error_e *result)
{
	phLibNfc_Handle hTargetDev;
	NFCSTATUS ret = NFCSTATUS_SUCCESS;
	pthread_cond_t controller_cond = PTHREAD_COND_INITIALIZER;
	controller_context_s context = { &controller_cond, NULL, 0 };

	if (result == NULL)
	{
		return false;
	}

	*result = NET_NFC_OK;

	if (handle == NULL || result == NULL)
	{
		*result = NET_NFC_NULL_PARAMETER;
		return false;
	}

	if (!__net_nfc_is_valid_target_handle(handle))
	{
		*result = NET_NFC_INVALID_HANDLE;
		return false;
	}

	hTargetDev = (phLibNfc_Handle)handle->connection_id;

	DEBUG_MSG("make read only ndef tag");

	NET_NFC_OEM_CONTROLLER_LOCK;

	if ((ret = phLibNfc_ConvertToReadOnlyNdef(hTargetDev, _net_nfc_make_read_only_cb, (void *)&context)) == NFCSTATUS_PENDING)
	{
		NET_NFC_OEM_CONTROLLER_WAIT(&controller_cond);
		NET_NFC_OEM_CONTROLLER_UNLOCK;

		if (context.result != NFCSTATUS_SUCCESS)
		{
			*result = _net_nfc_nxp_error_converter(context.result);
			return false;
		}
	}
	else
	{
		NET_NFC_OEM_CONTROLLER_UNLOCK;
		*result = _net_nfc_nxp_error_converter(ret);
		return false;
	}

	return true;

}

static bool net_nfc_nxp_controller_read_ndef(net_nfc_target_handle_s *handle, data_s **data, net_nfc_error_e *result)
{
	phLibNfc_Handle hTargetDev;
	NFCSTATUS ret = NFCSTATUS_SUCCESS;

	if (result == NULL)
	{
		return false;
	}

	*result = NET_NFC_OK;

	if (handle == NULL || data == NULL || result == NULL)
	{
		*result = NET_NFC_NULL_PARAMETER;
		return false;
	}

	if (!__net_nfc_is_valid_target_handle(handle))
	{
		*result = NET_NFC_INVALID_HANDLE;
		return false;
	}

	hTargetDev = (phLibNfc_Handle)handle->connection_id;

	DEBUG_MSG("read msg from TAG");

	uint8_t ndef_card_state = 0;
	int real_data_size = 0;
	int max_data_size = 0;

	if (net_nfc_nxp_controller_check_ndef(handle, &ndef_card_state, &max_data_size, &real_data_size, result) != true)
	{
		*result = NET_NFC_NO_NDEF_SUPPORT;
		return false;
	}
	else
	{
		if (real_data_size == 0)
		{
			*result = NET_NFC_NO_NDEF_MESSAGE;
			return false;
		}

		_net_nfc_util_alloc_mem(*data, sizeof(data_s));
		if (*data == NULL)
		{
			*result = NET_NFC_ALLOC_FAIL;
			return false;
		}

		(*data)->length = real_data_size;
		_net_nfc_util_alloc_mem((*data)->buffer, real_data_size);
		if ((*data)->buffer == NULL)
		{
			_net_nfc_util_free_mem(*data);
			*result = NET_NFC_ALLOC_FAIL;
			return false;
		}

		pthread_cond_t controller_cond = PTHREAD_COND_INITIALIZER;
		controller_context_s context = { &controller_cond, NULL, 0 };

		NET_NFC_OEM_CONTROLLER_LOCK;
		if ((ret = phLibNfc_Ndef_Read(hTargetDev, (phNfc_sData_t *)*data, phLibNfc_Ndef_EBegin, _net_nfc_ndef_read_cb, (void *)&context)) == NFCSTATUS_PENDING)
		{
			NET_NFC_OEM_CONTROLLER_WAIT(&controller_cond);
			NET_NFC_OEM_CONTROLLER_UNLOCK;

			if (context.result != NFCSTATUS_SUCCESS)
			{
				_net_nfc_util_free_mem((*data)->buffer);
				_net_nfc_util_free_mem(*data);

				*result = _net_nfc_nxp_error_converter(context.result);
				return false;
			}
		}
		else
		{
			NET_NFC_OEM_CONTROLLER_UNLOCK;

			_net_nfc_util_free_mem((*data)->buffer);
			_net_nfc_util_free_mem(*data);

			*result = _net_nfc_nxp_error_converter(ret);
			return false;
		}
	}

	return true;
}

static bool net_nfc_nxp_controller_write_ndef(net_nfc_target_handle_s *handle, data_s *data, net_nfc_error_e *result)
{
	phLibNfc_Handle hTargetDev;
	NFCSTATUS ret = NFCSTATUS_SUCCESS;
	bool is_ndef_support = false;

	uint8_t ndef_card_state = 0;
	int max_data_size = 0;
	int real_data_size = 0;

	if (result == NULL)
	{
		return false;
	}

	*result = NET_NFC_OK;

	if (handle == NULL || data == NULL || result == NULL)
	{
		*result = NET_NFC_NULL_PARAMETER;
		return false;
	}
	if (!__net_nfc_is_valid_target_handle(handle))
	{
		*result = NET_NFC_INVALID_HANDLE;
		return false;
	}

	hTargetDev = (phLibNfc_Handle)handle->connection_id;

	is_ndef_support = net_nfc_nxp_controller_check_ndef(handle, &ndef_card_state, &max_data_size, &real_data_size, result);
	if (is_ndef_support == true)
	{
		pthread_cond_t controller_cond = PTHREAD_COND_INITIALIZER;
		controller_context_s context = { &controller_cond, NULL, 0 };

		if (ndef_card_state == NET_NFC_NDEF_CARD_READ_ONLY)
		{
			DEBUG_MSG("It is read only tag", ndef_card_state);
			*result = NET_NFC_NOT_ALLOWED_OPERATION;

			return false;
		}

		if (data->length > max_data_size)
		{
			DEBUG_MSG("the MAX available size in this tag is = [%d] bytes. but you try to write [%d] bytes", max_data_size, data->length);
			*result = NET_NFC_INSUFFICIENT_STORAGE;

			return false;
		}

		NET_NFC_OEM_CONTROLLER_LOCK;

		if ((ret = phLibNfc_Ndef_Write(hTargetDev, (phNfc_sData_t *)(data), _net_nfc_ndef_write_cb, (void *)&context)) == NFCSTATUS_PENDING)
		{
			NET_NFC_OEM_CONTROLLER_WAIT(&controller_cond);
			NET_NFC_OEM_CONTROLLER_UNLOCK;

			if (context.result != NFCSTATUS_SUCCESS)
			{
				*result = _net_nfc_nxp_error_converter(context.result);
				return false;
			}
		}
		else
		{
			NET_NFC_OEM_CONTROLLER_UNLOCK;

			DEBUG_MSG("operation is failed = [0x%x]", ret);
			*result = _net_nfc_nxp_error_converter(ret);
			return false;
		}
	}
	else
	{
		*result = NET_NFC_NO_NDEF_SUPPORT;
		return false;
	}

	return true;
}

static bool net_nfc_nxp_controller_transceive(net_nfc_target_handle_s *handle, net_nfc_transceive_info_s *info, data_s **data, net_nfc_error_e *result)
{
	phLibNfc_Handle hTargetDev;
	NFCSTATUS friReturn = NFCSTATUS_SUCCESS;
	uint8_t trbuffer[NET_NFC_MAX_TRANCIEVE_BUFFER] = { 0, };
	phLibNfc_sTransceiveInfo_t trans_info;

	if (result == NULL)
	{
		return false;
	}

	*result = NET_NFC_OK;

	if (handle == NULL || info == NULL || data == NULL || result == NULL)
	{
		*result = NET_NFC_NULL_PARAMETER;
		return false;
	}

	if (!__net_nfc_is_valid_target_handle(handle))
	{
		*result = NET_NFC_INVALID_HANDLE;
		return false;
	}

	if (info->trans_data.buffer == NULL || info->trans_data.length == 0)
	{
		*result = NET_NFC_NULL_PARAMETER;
		return false;
	}

	hTargetDev = (phLibNfc_Handle)handle->connection_id;

	*result = NET_NFC_OK;

	_net_nfc_util_alloc_mem(*data, sizeof (data_s));

	(*data)->buffer = NULL;
	(*data)->length = 0;

	memset(&trans_info, 0x00, sizeof(phLibNfc_sTransceiveInfo_t));

	trans_info.sRecvData.length = NET_NFC_MAX_TRANCIEVE_BUFFER;
	trans_info.sRecvData.buffer = trbuffer;

	DEBUG_MSG_PRINT_BUFFER(info->trans_data.buffer, info->trans_data.length);

	switch (info->dev_type)
	{
	case NET_NFC_MIFARE_MINI_PICC :
	case NET_NFC_MIFARE_1K_PICC :
	case NET_NFC_MIFARE_4K_PICC :
	case NET_NFC_MIFARE_ULTRA_PICC :
		{
			trans_info.cmd.MfCmd = info->trans_data.buffer[0];
			trans_info.addr = info->trans_data.buffer[1];

			trans_info.sSendData.buffer = info->trans_data.buffer + 2; // only parametor should be passed to FRI stack with MIFARE cmd
			trans_info.sSendData.length = info->trans_data.length - 4; // remove CMD (1byte) + ADDR (1byte) + CRC_A (2byte)
		}
		break;

	case NET_NFC_FELICA_PICC :
		{
			trans_info.cmd.FelCmd = phNfc_eFelica_Raw;
			trans_info.addr = 0x00;
			trans_info.sSendData.buffer = info->trans_data.buffer;
			trans_info.sSendData.length = info->trans_data.length;
		}
		break;

	case NET_NFC_ISO15693_PICC :
		{
			trans_info.cmd.Iso15693Cmd = phNfc_eIso15693_Cmd;
			trans_info.addr = 0x00;
			trans_info.sSendData.buffer = info->trans_data.buffer;
			trans_info.sSendData.length = info->trans_data.length;
		}
		break;

	default :
		{
			trans_info.sSendData.buffer = info->trans_data.buffer;
			trans_info.sSendData.length = info->trans_data.length;
		}
		break;
	}

	pthread_cond_t controller_cond = PTHREAD_COND_INITIALIZER;
	controller_context_s context = { &controller_cond, *data, 0 };

	NET_NFC_OEM_CONTROLLER_LOCK;
	friReturn = phLibNfc_RemoteDev_Transceive(hTargetDev, &trans_info, _net_nfc_transceive_cb, (void *)&context);
	if (friReturn == NFCSTATUS_PENDING)
	{
		NET_NFC_OEM_CONTROLLER_WAIT(&controller_cond);
		NET_NFC_OEM_CONTROLLER_UNLOCK;

		if (context.result == NFCSTATUS_SUCCESS)
		{
			return true;
		}
		else
		{
			*result = _net_nfc_nxp_error_converter(context.result);

			/* TODO : release data??? */
			return false;
		}
	}
	else
	{
		NET_NFC_OEM_CONTROLLER_UNLOCK;
		DEBUG_MSG("transceiving with remote device is failed %d", friReturn);
		*result = _net_nfc_nxp_error_converter(friReturn);

		/* TODO : release data??? */
		return false;
	}
}

static bool net_nfc_nxp_controller_exception_handler(void)
{
	NFCSTATUS status = NFCSTATUS_SUCCESS;
	net_nfc_error_e error;

	if ((status = phLibNfc_Mgt_DeInitialize(psHwRef.p_board_driver, NULL, NULL)) != NFCSTATUS_SUCCESS)
	{
		DEBUG_MSG("deinit is failed");
	}
	else
	{
		DEBUG_MSG("deinit is success");
	}

	if (phLibNfc_Mgt_UnConfigureDriver(psHwRef.p_board_driver) == NFCSTATUS_SUCCESS)
	{
		DEBUG_MSG("unregister driver is success");
	}
	else
	{
		DEBUG_MSG("unregister driver is failed");
	}

	if (net_nfc_nxp_controller_init(&error) == false)
	{
		exit(0xff);
	}
	else
	{
		net_nfc_llcp_config_info_s config = { 128, 1, 100, 0 };

		if (net_nfc_nxp_controller_llcp_config(&config, &error) == true)
		{
			DEBUG_MSG("set llcp config is success");
		}
		else
		{
			DEBUG_MSG("set llcp config is failed");
		}

		if (net_nfc_nxp_controller_register_listener(g_nxp_controller_target_cb, g_nxp_controller_se_cb, g_nxp_controller_llcp_cb, &error) == true)
		{
			DEBUG_MSG("register event cb is success");
		}
		else
		{
			DEBUG_MSG("register event cb is failed");
		}

		if (net_nfc_nxp_controller_configure_discovery(NET_NFC_DISCOVERY_MODE_CONFIG, NET_NFC_ALL_ENABLE, &error) == true)
		{
			DEBUG_MSG("configure discovery loop is success");
		}
		else
		{
			DEBUG_MSG("configure discovery loop is failed");
		}
	}

	return true;
}

static bool net_nfc_nxp_controller_is_ready(net_nfc_error_e *result)
{
	if (result == NULL)
	{
		return false;
	}

	return g_stack_init_successful;
}

static bool net_nfc_nxp_controller_format_ndef(net_nfc_target_handle_s *handle, data_s *secure_key, net_nfc_error_e *result)
{
	phLibNfc_Handle hTargetDev;
	NFCSTATUS status = NFCSTATUS_SUCCESS;
	pthread_cond_t controller_cond = PTHREAD_COND_INITIALIZER;
	controller_context_s context = { &controller_cond, NULL, 0 };
	uint8_t ndef_card_state = 0;
	int max_data_size = 0;
	int real_data_size = 0;

	if (result == NULL)
	{
		return false;
	}

	*result = NET_NFC_OK;

	if (handle == NULL)
	{
		*result = NET_NFC_NULL_PARAMETER;
		return false;
	}

	if (!__net_nfc_is_valid_target_handle(handle))
	{
		*result = NET_NFC_INVALID_HANDLE;
		return false;
	}

	hTargetDev = (phLibNfc_Handle)handle->connection_id;

	if (net_nfc_nxp_controller_check_ndef(handle, &ndef_card_state, &max_data_size, &real_data_size, result) == true)
	{
		DEBUG_MSG("ndef format : NET_NFC_TAG_IS_ALREADY_FORMATTED");
		*result = NET_NFC_TAG_IS_ALREADY_FORMATTED;
		return false;
	}

	NET_NFC_OEM_CONTROLLER_LOCK;

	if ((status = phLibNfc_RemoteDev_FormatNdef(hTargetDev, (phNfc_sData_t *)secure_key, _net_nfc_format_remote_dev_ndef_cb, &context)) == NFCSTATUS_PENDING)
	{
		NET_NFC_OEM_CONTROLLER_WAIT(&controller_cond);
		NET_NFC_OEM_CONTROLLER_UNLOCK;

		if (context.result == NFCSTATUS_SUCCESS)
		{
			DEBUG_MSG("net_nfc_nxp_controller_format_ndef success");
			*result = NET_NFC_OK;
			return true;
		}
		else
		{
			DEBUG_MSG("ndef format : error is returned from stack #1= [0x%x]", context.result);
			return false;
		}
	}
	else
	{
		DEBUG_MSG("ndef format : error is returned from stack #2 = [0x%x]", status);

		NET_NFC_OEM_CONTROLLER_UNLOCK;
		*result = _net_nfc_nxp_error_converter(status);

		return false;
	}
}

/*******************
 *	LLCP definition     *
 ********************/

void _net_nfc_llcp_config_cb(void *pContext, NFCSTATUS status)
{
	controller_context_s *context = NULL;

	if (pContext != NULL)
	{
		context = (controller_context_s *)pContext;
		context->result = (int)status;
	}

	if (status == NFCSTATUS_SUCCESS)
	{
		DEBUG_MSG("set llcp config is success");
	}
	else
	{
		DEBUG_MSG("set llcp config is failed = [0x%x]", status);
	}

	if (context != NULL)
	{
		NET_NFC_OEM_CONTROLLER_SIGNAL(context->controller_cond);
	}
}

void _net_nfc_llcp_check_llcp_cb(void *pContext, NFCSTATUS status)
{
	if (pContext != NULL)
	{
		controller_context_s *context = (controller_context_s *)pContext;
		context->result = status;
	}

	if (status == NFCSTATUS_SUCCESS)
	{
		DEBUG_MSG("llcp status is success");
	}
	else
	{
		DEBUG_MSG("llcp status is not good = [0x%x]", status);
	}
}

void _net_nfc_llcp_link_status_cb(void *pContext, phLibNfc_Llcp_eLinkStatus_t eLinkStatus)
{
	if (eLinkStatus == phFriNfc_LlcpMac_eLinkActivated)
	{
		DEBUG_MSG("llcp link status is activated");
	}
	else
	{
		DEBUG_MSG("llcp link status is deactivated = [0x%x]", eLinkStatus);

		if (pContext != NULL)
		{
			controller_context_s *context = (controller_context_s *)pContext;
			net_nfc_request_llcp_msg_t *req_msg = NULL;

			_net_nfc_util_alloc_mem(req_msg, sizeof(net_nfc_request_llcp_msg_t));
			if (req_msg != NULL)
			{
				req_msg->length = sizeof(net_nfc_request_llcp_msg_t);
				req_msg->request_type = NET_NFC_MESSAGE_SERVICE_LLCP_DEACTIVATED;
				g_nxp_controller_llcp_cb(req_msg, context->user_context);
			}

			_net_nfc_util_free_mem(context);
		}
	}
}

void _net_nfc_llcp_socket_err_cb(void *pContext, uint8_t nErrCode)
{
	socket_info_s *socket_info = (socket_info_s *)pContext;

	// CB MUST return ASAP
	if (nErrCode == PHFRINFC_LLCP_ERR_FRAME_REJECTED)
	{
		DEBUG_MSG("> ERROR: FRAME REJECTED - DISCONNECTED \n");
	}
	else if (nErrCode == PHFRINFC_LLCP_ERR_DISCONNECTED)
	{
		DEBUG_MSG("> Socket error and DISCONNECTED \n");
	}
	else
	{
		DEBUG_MSG("> Error code: %d \n", nErrCode);
	}

	if (socket_info != NULL)
	{
		if (_net_nfc_find_server_socket(socket_info->socket_handle) == NULL)
		{
			DEBUG_MSG("> socket is already closed \n");
			return;
		}
		if (socket_info->socket_handle != (net_nfc_llcp_socket_t) NULL && socket_info->isValid == true)
		{
			net_nfc_request_llcp_msg_t *req_msg = NULL;

			DEBUG_MSG("close err socket = [0x%x]", socket_info->socket_handle);

			_net_nfc_util_alloc_mem(req_msg, sizeof(net_nfc_request_llcp_msg_t));
			if (req_msg != NULL)
			{
				req_msg->length = sizeof(net_nfc_request_llcp_msg_t);
				req_msg->request_type = NET_NFC_MESSAGE_SERVICE_LLCP_SOCKET_ERROR;
				req_msg->result = net_nfc_nxp_controller_convert_llcp_error_codes(nErrCode);

				g_nxp_controller_llcp_cb(req_msg, socket_info->user_context);
			}
		}
	}
}

void __net_nfc_llcp_accept_cb_from_listen(void *pContext, NFCSTATUS status)
{
	accept_context_s *context = NULL;
	net_nfc_request_accept_socket_t *detail = NULL;

	DEBUG_MSG("accept cb is called");

	_net_nfc_util_alloc_mem(detail, sizeof(net_nfc_request_accept_socket_t));
	if (detail != NULL)
	{
		context = (accept_context_s *)pContext;

		detail->length = sizeof(net_nfc_request_accept_socket_t);
		detail->request_type = NET_NFC_MESSAGE_SERVICE_LLCP_ACCEPT;
		detail->handle = context->handle;
		detail->incomming_socket = context->incomming;
		detail->trans_param = context->user_param;
		detail->result = _net_nfc_nxp_error_converter(status);

		DEBUG_MSG("callback is called, incomming is %X", detail->incomming_socket);

		g_nxp_controller_llcp_cb(detail, context->user_param);
	}
}

void __net_nfc_llcp_accepted_socket_err_cb_from_listen(void *pContext, uint8_t nErrCode)
{
	accept_context_s *context = NULL;

	if (nErrCode == PHFRINFC_LLCP_ERR_FRAME_REJECTED)
	{
		DEBUG_MSG("> ERROR: FRAME REJECTED - DISCONNECTED \n");
	}
	else if (nErrCode == PHFRINFC_LLCP_ERR_DISCONNECTED)
	{
		DEBUG_MSG("> DISCONNECTED \n");
	}
	else
	{
		DEBUG_MSG("> Error code: %d \n", nErrCode);
	}

	if (pContext != NULL)
	{
		context = (accept_context_s *)pContext;

		if (_net_nfc_find_server_socket(context->incomming) == NULL)
		{
			DEBUG_MSG("> socket is already closed \n");
		}
		else
		{
			net_nfc_request_llcp_msg_t *req_msg = NULL;

			_net_nfc_util_alloc_mem(req_msg, sizeof (net_nfc_request_llcp_msg_t));

			if (req_msg != NULL)
			{
				req_msg->length = sizeof(net_nfc_request_llcp_msg_t);
				req_msg->request_type = NET_NFC_MESSAGE_SERVICE_LLCP_SOCKET_ACCEPTED_ERROR;
				req_msg->result = net_nfc_nxp_controller_convert_llcp_error_codes(nErrCode);

				g_nxp_controller_llcp_cb(req_msg, context->user_param);
			}
		}
	}
}

void _net_nfc_llcp_listen_cb(void *pContext, net_nfc_llcp_socket_t IncomingSocket)
{
	DEBUG_MSG("listen cb is called. Incomming Socket is = [0x%x]", IncomingSocket);

	/*	net_nfc_request_msg_t *req_msg = (net_nfc_request_msg_t*)calloc(1, sizeof(net_nfc_request_msg_t));
	 net_nfc_request_accept_socket_t *detail = (net_nfc_request_accept_socket_t *)calloc(1, sizeof(net_nfc_request_accept_socket_t));
	 accept_context_s *context = NULL;
	 socket_info_s *psocket_info = NULL;

	 if(req_msg != NULL && detail != NULL)
	 {
	 context = (accept_context_s *) pContext;
	 detail->incomming_socket = context->server_scket;
	 detail->handle  = context->handle;

	 psocket_info = _net_nfc_find_server_socket(context->server_scket);
	 psocket_info->socket_handle_incomming = IncomingSocket;

	 req_msg->request_type = NET_NFC_MESSAGE_SERVICE_LLCP_ACCEPT;
	 req_msg->detail_message = detail;

	 g_nxp_controller_llcp_cb(req_msg, NULL);
	 }
	 if (context != NULL) {
	 _net_nfc_util_free_mem (context);
	 }
	 */

	if (pContext != NULL)
	{
		accept_context_s *context = (accept_context_s *)pContext;
		socket_info_s *psocket_info = NULL;
		data_s *socket_buffer;
		phLibNfc_Llcp_sSocketOptions_t sOptions;
		accept_context_s *accept_socket_context = NULL;
		NFCSTATUS status;

		psocket_info = _net_nfc_find_server_socket(context->server_scket);
//		psocket_info->socket_handle_incomming = IncomingSocket;
		context->incomming = IncomingSocket;

		sOptions.miu = psocket_info->miu;
		sOptions.rw = psocket_info->rw;

		psocket_info = _net_nfc_get_available_socket_slot();
		if (psocket_info == NULL)
		{
			DEBUG_ERR_MSG("> ERROR: failed to accept incomming socket caused by no available slot \n");
			return;
		}
		psocket_info->socket_handle = IncomingSocket;
		//psocket_info->miu = psocket_info->miu;
		//psocket_info->rw = psocket_info->rw;
		socket_buffer = &(psocket_info->data);

		/*_net_nfc_util_alloc_mem (context->data.buffer, BUFFER_LENGTH_MAX);
		 context->data.length = BUFFER_LENGTH_MAX;
		 if (context->data.buffer == NULL) {
		 DEBUG_MSG("> ERROR: socket buffer allocation is failed  \n");
		 }
		 socket_buffer = &(context->data);
		 */

//		NET_NFC_OEM_CONTROLLER_LOCK;
		_net_nfc_util_alloc_mem(accept_socket_context, sizeof(accept_context_s));
		memcpy(accept_socket_context, context, sizeof(accept_context_s));

		psocket_info->context = accept_socket_context;

		status = phLibNfc_Llcp_Accept(IncomingSocket,
			&sOptions,
			(phNfc_sData_t *)socket_buffer,
			__net_nfc_llcp_accepted_socket_err_cb_from_listen,
			__net_nfc_llcp_accept_cb_from_listen,
			accept_socket_context);
		DEBUG_MSG("Accept in listen callback is called: return [%d]", status);
//		NET_NFC_OEM_CONTROLLER_UNLOCK
	}

	DEBUG_MSG("listen cb is end");
}

void _net_nfc_llcp_accepted_socket_err_cb(void *pContext, uint8_t nErrCode)
{
	controller_context_s *context = NULL;

	if (nErrCode == PHFRINFC_LLCP_ERR_FRAME_REJECTED)
	{
		DEBUG_MSG("> ERROR: FRAME REJECTED - DISCONNECTED \n");
	}
	else if (nErrCode == PHFRINFC_LLCP_ERR_DISCONNECTED)
	{
		DEBUG_MSG("> DISCONNECTED \n");
	}
	else
	{
		DEBUG_MSG("> Error code: %d \n", nErrCode);
	}

	if (pContext != NULL)
	{
		net_nfc_request_llcp_msg_t *req_msg = NULL;
		socket_info_s *socket_info = (socket_info_s *)context->user_context;

		context = (controller_context_s *)pContext;

		_net_nfc_util_alloc_mem(req_msg, sizeof (net_nfc_request_llcp_msg_t));
		if (req_msg != NULL)
		{
			req_msg->length = sizeof(net_nfc_request_llcp_msg_t);
			req_msg->request_type = NET_NFC_MESSAGE_SERVICE_LLCP_SOCKET_ACCEPTED_ERROR;
			req_msg->result = net_nfc_nxp_controller_convert_llcp_error_codes(nErrCode);

			g_nxp_controller_llcp_cb(req_msg, context->user_context);
		}

		if (socket_info != NULL)
		{
			if (socket_info->data.buffer != NULL)
			{
				_net_nfc_util_free_mem(socket_info->data.buffer);
				socket_info->data.buffer = NULL;
			}

			_net_nfc_util_free_mem(socket_info);
		}

		_net_nfc_util_free_mem(context);
	}
}

void _net_nfc_llcp_accept_cb(void *pContext, NFCSTATUS status)
{
	controller_context_s *context = NULL;

	if (pContext != NULL)
	{
		context = (controller_context_s *)pContext;
		socket_info_s *socket_info = (socket_info_s *)context->user_context;

		if (socket_info != NULL)
		{
			context->result = (int)status;
		}

		// TODO : free socket_info

		// free should be done in socket error
		//free(socket_info);
	}

	if (status == NFCSTATUS_SUCCESS)
	{
		DEBUG_MSG(">>>>> client is accepted \n");
	}
	else
	{
		DEBUG_MSG(">>>>> client is not accpeted = [0x%x]\n", status);
	}

	if (context != NULL)
	{
		NET_NFC_OEM_CONTROLLER_SIGNAL(context->controller_cond);
	}
}

void _net_nfc_llcp_connect_sap_cb(void *pContext, uint8_t nErrCode, NFCSTATUS status)
{
	if (pContext != NULL)
	{
		net_nfc_request_llcp_msg_t *req_msg = NULL;

		if (status == NFCSTATUS_SUCCESS)
		{
			DEBUG_MSG(">>>>> SOCKET is CONNECTED \n");
		}
		else
		{
			DEBUG_MSG(">>>>> SOCKET is not CONNECTED status = [0x%x] errCode = [0x%x]\n", status, nErrCode);
		}

		_net_nfc_util_alloc_mem(req_msg, sizeof(net_nfc_request_llcp_msg_t));
		if (req_msg == NULL)
		{
			DEBUG_MSG("Allocation is failed\n");
			return;
		}

		req_msg->length = sizeof(net_nfc_request_llcp_msg_t);
		req_msg->request_type = NET_NFC_MESSAGE_SERVICE_LLCP_CONNECT_SAP;
		req_msg->result = _net_nfc_nxp_error_converter(status);

		g_nxp_controller_llcp_cb(req_msg, pContext);
	}
}

void _net_nfc_llcp_connect_cb(void *pContext, uint8_t nErrCode, NFCSTATUS status)
{
	if (pContext != NULL)
	{
		net_nfc_request_llcp_msg_t *req_msg = NULL;

		if (status == NFCSTATUS_SUCCESS)
		{
			DEBUG_MSG(">>>>> SOCKET is CONNECTED \n");
		}
		else
		{
			DEBUG_MSG(">>>>> SOCKET is not CONNECTED status = [0x%x] errCode = [0x%x]\n", status, nErrCode);
		}

		_net_nfc_util_alloc_mem(req_msg, sizeof(net_nfc_request_llcp_msg_t));
		if (req_msg == NULL)
		{
			DEBUG_MSG("Allocation is failed\n");
			return;
		}

		req_msg->length = sizeof(net_nfc_request_llcp_msg_t);
		req_msg->request_type = NET_NFC_MESSAGE_SERVICE_LLCP_CONNECT;
		req_msg->result = _net_nfc_nxp_error_converter(status);

		g_nxp_controller_llcp_cb(req_msg, pContext);
	}
}

void _net_nfc_llcp_send_cb(void *pContext, NFCSTATUS status)
{
	net_nfc_request_llcp_msg_t *req_msg = NULL;
	transfer_context_s *tcontext = (transfer_context_s *)pContext;

	if (pContext == NULL)
		return;

	_net_nfc_util_alloc_mem(req_msg, sizeof(net_nfc_request_llcp_msg_t));
	if (req_msg != NULL)
	{
		req_msg->length = sizeof(net_nfc_request_llcp_msg_t);
		req_msg->request_type = NET_NFC_MESSAGE_SERVICE_LLCP_SEND;
		req_msg->result = _net_nfc_nxp_error_converter(status);

		g_nxp_controller_llcp_cb(req_msg, tcontext->user_param);
	}

	/* below memory must be freed */
	_net_nfc_util_free_mem(tcontext->data.buffer);
	_net_nfc_util_free_mem(tcontext);
}

void _net_nfc_llcp_sendTo_cb(void *pContext, NFCSTATUS status)
{
	net_nfc_request_llcp_msg_t *req_msg = NULL;
	transfer_context_s *tcontext = (transfer_context_s *)pContext;

	if (pContext == NULL)
		return;

	_net_nfc_util_alloc_mem(req_msg, sizeof(net_nfc_request_llcp_msg_t));
	if (req_msg != NULL)
	{
		req_msg->length = sizeof(net_nfc_request_llcp_msg_t);
		req_msg->request_type = NET_NFC_MESSAGE_SERVICE_LLCP_SEND_TO;
		req_msg->result = _net_nfc_nxp_error_converter(status);

		g_nxp_controller_llcp_cb(req_msg, tcontext->user_param);
	}

	/* below memory must be freed */
	_net_nfc_util_free_mem(tcontext->data.buffer);
	_net_nfc_util_free_mem(tcontext);
}

void _net_nfc_llcp_recv_cb(void *pContext, NFCSTATUS status)
{
	net_nfc_request_llcp_msg_t *req_msg = NULL;

	_net_nfc_util_alloc_mem(req_msg, sizeof(net_nfc_request_llcp_msg_t));
	if (req_msg != NULL)
	{
		req_msg->length = sizeof(net_nfc_request_llcp_msg_t);
		req_msg->request_type = NET_NFC_MESSAGE_SERVICE_LLCP_RECEIVE;
		req_msg->result = _net_nfc_nxp_error_converter(status);

		g_nxp_controller_llcp_cb(req_msg, pContext);
	}
}

void _net_nfc_llcp_recv_from_cb(void *pContext, uint8_t service_access_point, NFCSTATUS status)
{
	net_nfc_request_llcp_msg_t *req_msg = NULL;

	_net_nfc_util_alloc_mem(req_msg, sizeof(net_nfc_request_llcp_msg_t));
	if (req_msg != NULL)
	{
		req_msg->length = sizeof(net_nfc_request_llcp_msg_t);
		req_msg->request_type = NET_NFC_MESSAGE_SERVICE_LLCP_RECEIVE_FROM;
		req_msg->result = _net_nfc_nxp_error_converter(status);

		g_nxp_controller_llcp_cb(req_msg, pContext);
	}
}

void _net_nfc_llcp_disconnect_cb(void *pContext, NFCSTATUS status)
{
	net_nfc_request_llcp_msg_t *req_msg = NULL;

	_net_nfc_util_alloc_mem(req_msg, sizeof(net_nfc_request_llcp_msg_t));
	if (req_msg != NULL)
	{
		req_msg->length = sizeof(net_nfc_request_llcp_msg_t);
		req_msg->request_type = NET_NFC_MESSAGE_SERVICE_LLCP_DISCONNECT;
		req_msg->result = _net_nfc_nxp_error_converter(status);

		g_nxp_controller_llcp_cb(req_msg, pContext);
	}
}

void _net_nfc_llcp_reject_cb(void *pContext, NFCSTATUS status)
{
	controller_context_s *context = NULL;

	if (status == NFCSTATUS_SUCCESS)
	{
		DEBUG_MSG("llcp reject [0x%x] is success", status);
	}
	else
	{
		DEBUG_MSG("llcp reject [0x%x] is failed", status);
	}

	DEBUG_MSG("pContext = [0x%x]", pContext);

	if (pContext != NULL)
	{
		context = (controller_context_s *)pContext;
		context->result = status;

		NET_NFC_OEM_CONTROLLER_SIGNAL(context->controller_cond);
	}
}

static bool net_nfc_nxp_controller_llcp_config(net_nfc_llcp_config_info_s *config, net_nfc_error_e * result)
{
	NFCSTATUS status = NFCSTATUS_SUCCESS;
	phLibNfc_Llcp_sLinkParameters_t LLCP_Config_Info;
	pthread_cond_t controller_cond = PTHREAD_COND_INITIALIZER;
	controller_context_s context = { &controller_cond, NULL, 0 };

	if (result == NULL)
	{
		return false;
	}

	*result = NET_NFC_OK;

	_net_nfc_reset_socket_array();

	memset(&LLCP_Config_Info, 0x00, sizeof(phLibNfc_Llcp_sLinkParameters_t));

	LLCP_Config_Info.miu = config->miu;
	LLCP_Config_Info.lto = config->lto;
	LLCP_Config_Info.wks = config->wks;
	LLCP_Config_Info.option = config->option;

	DEBUG_MSG("> Set LLCP config params (LTO=%d, MIU=%d, OPTION=0x%02x, WKS=0x%02x): ", LLCP_Config_Info.lto,
		LLCP_Config_Info.miu,
		LLCP_Config_Info.option,
		LLCP_Config_Info.wks);

	NET_NFC_OEM_CONTROLLER_LOCK;
	if ((status = phLibNfc_Mgt_SetLlcp_ConfigParams(&LLCP_Config_Info,
		_net_nfc_llcp_config_cb,
		&context)) == NFCSTATUS_PENDING)
	{
		NET_NFC_OEM_CONTROLLER_WAIT(&controller_cond);
		NET_NFC_OEM_CONTROLLER_UNLOCK;

	}
	else if (status == NFCSTATUS_SUCCESS)
	{
		NET_NFC_OEM_CONTROLLER_UNLOCK;
		return true;
	}
	else
	{
		NET_NFC_OEM_CONTROLLER_UNLOCK;
		DEBUG_MSG("set llcp config operation is failed = [0x%x]", status);
		*result = _net_nfc_nxp_error_converter(status);
		return false;
	}

	if (context.result != NFCSTATUS_SUCCESS)
	{
		DEBUG_MSG("set llcp config param is failed");
		*result = _net_nfc_nxp_error_converter(context.result);
		return false;
	}

	return true;
}

static bool net_nfc_nxp_controller_llcp_check_llcp(net_nfc_target_handle_s *handle, net_nfc_error_e *result)
{
	controller_context_s *context = NULL;
	phLibNfc_Handle hRemoteDevice;
	NFCSTATUS status = NFCSTATUS_SUCCESS;

	if (result == NULL)
	{
		return false;
	}

	*result = NET_NFC_OK;

	if (handle == NULL)
	{
		*result = NET_NFC_NULL_PARAMETER;
		return false;
	}

	if (!__net_nfc_is_valid_target_handle(handle))
	{
		*result = NET_NFC_INVALID_HANDLE;
		return false;
	}

	_net_nfc_util_alloc_mem(context, sizeof(controller_context_s));
	if (context == NULL)
	{
		*result = NET_NFC_ALLOC_FAIL;
		return false;
	}

	context->controller_cond = NULL;
	context->user_context = handle;
	context->result = 0;

	hRemoteDevice = handle->connection_id;

	// pass the connection handle value to disconnect

	NET_NFC_OEM_CONTROLLER_LOCK;
	status = phLibNfc_Llcp_CheckLlcp(hRemoteDevice, _net_nfc_llcp_check_llcp_cb, _net_nfc_llcp_link_status_cb, context);

	DEBUG_MSG("llcp check_llcp operation result = [0x%x]", status);

	if (status == NFCSTATUS_SUCCESS || status == NFCSTATUS_PENDING)
	{
		NET_NFC_OEM_CONTROLLER_UNLOCK;

		DEBUG_MSG("check result");

		if (context->result != NFCSTATUS_SUCCESS)
		{
			DEBUG_MSG("Llcp of remote target is not available");
			*result = _net_nfc_nxp_error_converter(context->result);
			return false;
		}
		else
		{
			DEBUG_MSG("Llcp of remote target is activated");
			return true;
		}
	}
	else
	{
		NET_NFC_OEM_CONTROLLER_UNLOCK;
		DEBUG_MSG("check_llcp operation is failed: %d\n", status);
		*result = _net_nfc_nxp_error_converter(status);
		return false;
	}
}

static bool net_nfc_nxp_controller_llcp_activate_llcp(net_nfc_target_handle_s *handle, net_nfc_error_e *result)
{
	phLibNfc_Handle hRemoteDevice;
	NFCSTATUS status = NFCSTATUS_SUCCESS;

	if (result == NULL)
	{
		return false;
	}

	*result = NET_NFC_OK;

	if (handle == NULL)
	{
		*result = NET_NFC_NULL_PARAMETER;
		return false;
	}

	if (!__net_nfc_is_valid_target_handle(handle))
	{
		*result = NET_NFC_INVALID_HANDLE;
		return false;
	}

	hRemoteDevice = handle->connection_id;

	NET_NFC_OEM_CONTROLLER_LOCK;
	status = phLibNfc_Llcp_Activate(hRemoteDevice);
	NET_NFC_OEM_CONTROLLER_UNLOCK;

	if (status == NFCSTATUS_SUCCESS || status == NFCSTATUS_PENDING)
	{
		DEBUG_MSG("llcp layer is activated = [0x%x]", status);
		return true;
	}
	else
	{
		DEBUG_MSG("activating llcp layer is failed = [0x%x]", status);
		*result = _net_nfc_nxp_error_converter(status);
		return false;
	}
}

/*
static bool net_nfc_nxp_controller_llcp_deactivate_llcp(net_nfc_target_handle_s *handle, net_nfc_error_e *result)
{
	if(result == NULL)
	{
		return false;
	}

	*result = NET_NFC_OK;

	if(handle == NULL)
	{
		*result = NET_NFC_NULL_PARAMETER;
		return false;
	}

	if (!__net_nfc_is_valide_target_handle (handle)){
		*result = NET_NFC_INVALID_HANDLE;
		return false;
	}

	NFCSTATUS 	status = NFCSTATUS_SUCCESS;
	phLibNfc_Handle hRemoteDevice = handle->connection_id;

	NET_NFC_OEM_CONTROLLER_LOCK;
	status = phLibNfc_Llcp_Deactivate( hRemoteDevice );
	NET_NFC_OEM_CONTROLLER_UNLOCK;

	if (status == NFCSTATUS_SUCCESS || status == NFCSTATUS_PENDING)
	{
		DEBUG_MSG("llcp layer is deactivated = [0x%x]", status);
		return true;
	}
	else
	{
		DEBUG_MSG("Deactivating llcp layer is failed = [0x%x]", status);
		*result = _net_nfc_nxp_error_converter (status);
		return false;
	}
}
*/

static bool net_nfc_nxp_controller_llcp_create_socket(net_nfc_llcp_socket_t *socket, net_nfc_socket_type_e socketType, uint16_t miu, uint8_t rw, net_nfc_error_e *result, void *user_param)
{
	NFCSTATUS status = NFCSTATUS_SUCCESS;
	phLibNfc_Llcp_sSocketOptions_t sOptions;
	socket_info_s *socket_info = NULL;

	if (result == NULL)
	{
		return false;
	}

	*result = NET_NFC_OK;

	if (socket == NULL)
	{
		*result = NET_NFC_NULL_PARAMETER;
		return false;
	}

	if (socketType < NET_NFC_LLCP_SOCKET_TYPE_CONNECTIONORIENTED || socketType > NET_NFC_LLCP_SOCKET_TYPE_CONNECTIONLESS)
	{
		*result = NET_NFC_NULL_PARAMETER;
		return false;
	}

	// hyogil.kim
	// Where is better to place this available socket slot fucntionality ? nfc-service ? oem_controller ?
	socket_info = _net_nfc_get_available_socket_slot();
	if (socket_info == NULL)
	{
		DEBUG_MSG("mem alloc is failed");
		return 0;
	}

	socket_info->user_context = user_param;
	socket_info->miu = miu;
	socket_info->rw = rw;

	memset(&sOptions, 0x00, sizeof(phLibNfc_Llcp_sSocketOptions_t));

	NET_NFC_OEM_CONTROLLER_LOCK;

	if (socketType == NET_NFC_LLCP_SOCKET_TYPE_CONNECTIONORIENTED)
	{
		sOptions.miu = miu;
		sOptions.rw = rw;
		DEBUG_MSG("> LLCP Create Socket Connection Oriented: MIU:%d RW:%d", miu, rw);
		status = phLibNfc_Llcp_Socket(
			phFriNfc_LlcpTransport_eConnectionOriented,
			&sOptions,
			(phNfc_sData_t*)&(socket_info->data),
			socket,
			_net_nfc_llcp_socket_err_cb,
			socket_info);
	}
	else
	{
		DEBUG_MSG("> LLCP Create Socket ConnectionLess: ");
		status = phLibNfc_Llcp_Socket(
			phFriNfc_LlcpTransport_eConnectionLess,
			NULL,
			NULL,
			socket,
			_net_nfc_llcp_socket_err_cb,

			socket_info);
	}

	if (status == NFCSTATUS_SUCCESS || status == NFCSTATUS_PENDING)
	{
		NET_NFC_OEM_CONTROLLER_UNLOCK;
		DEBUG_MSG("socket is created = [0x%x], result = [0x%x]", *socket, status);
		socket_info->socket_handle = *socket;

		return true;
	}
	else
	{
		NET_NFC_OEM_CONTROLLER_UNLOCK;
		DEBUG_MSG("operation socket creation is failed = [0x%x]", status);
		*result = _net_nfc_nxp_error_converter(status);
		return false;
	}
}

static bool net_nfc_nxp_controller_llcp_bind(net_nfc_llcp_socket_t socket, uint8_t service_access_point, net_nfc_error_e *result)
{
	NFCSTATUS status = NFCSTATUS_SUCCESS;
	if (result == NULL)

	{
		return false;
	}

	*result = NET_NFC_OK;

	NET_NFC_OEM_CONTROLLER_LOCK;
	status = phLibNfc_Llcp_Bind(socket, service_access_point);
	NET_NFC_OEM_CONTROLLER_UNLOCK;

	DEBUG_MSG("llcp bind operation result = [0x%x]", status);

	*result = _net_nfc_nxp_error_converter(status);

	if (status != NFCSTATUS_SUCCESS)
	{
		DEBUG_MSG("phLibNfc_Llcp_Bind is failed = [0x%x]", status);
		return false;
	}

	return true;
}

static bool net_nfc_nxp_controller_llcp_listen(net_nfc_target_handle_s *handle, uint8_t *service_access_name, net_nfc_llcp_socket_t socket, net_nfc_error_e *result, void *user_param)
{
	NFCSTATUS status = NFCSTATUS_SUCCESS;
	accept_context_s *accept_context = NULL;

	if (result == NULL)
	{
		return false;
	}

	if (service_access_name == NULL)
	{
		*result = NET_NFC_NULL_PARAMETER;
		return false;
	}

	if (!__net_nfc_is_valid_target_handle(handle))
	{
		*result = NET_NFC_INVALID_HANDLE;
		return false;
	}

	*result = NET_NFC_OK;

	_net_nfc_util_alloc_mem(accept_context, sizeof(accept_context_s));
	if (accept_context == NULL)
	{
		*result = NET_NFC_ALLOC_FAIL;
		return false;
	}

	accept_context->handle = handle;
	accept_context->server_scket = socket;
	accept_context->user_param = user_param;

	phNfc_sData_t service_name = { service_access_name, strlen((char *)service_access_name) };

	socket_info_s *psocket_info = _net_nfc_find_server_socket(socket);
	if (psocket_info != NULL)
	{
		psocket_info->context = accept_context; // let it free when the socket is closed
	}

	NET_NFC_OEM_CONTROLLER_LOCK;
	status = phLibNfc_Llcp_Listen(socket, (phNfc_sData_t *)&service_name, _net_nfc_llcp_listen_cb, (void *)accept_context);
	NET_NFC_OEM_CONTROLLER_UNLOCK;

	DEBUG_MSG("llcp listen operation result = [0x%x]", status);

	if (status == NFCSTATUS_SUCCESS || status == NFCSTATUS_PENDING)
	{
		DEBUG_MSG("socket listening is good");
		return true;
	}
	else
	{
		DEBUG_MSG("operation socket listening is failed = [0x%x]", status);
		*result = _net_nfc_nxp_error_converter(status);
		return false;
	}
}

/* below accept function does not used. */
static bool net_nfc_nxp_controller_llcp_accept(net_nfc_llcp_socket_t socket, net_nfc_error_e *result)
{
	NFCSTATUS status = NFCSTATUS_SUCCESS;
	phLibNfc_Llcp_sSocketOptions_t sOptions;
	socket_info_s *socket_info = NULL;
	controller_context_s *context = NULL;

	sOptions.miu = PHFRINFC_LLCP_MIU_DEFAULT;
	sOptions.rw = PHFRINFC_LLCP_RW_DEFAULT * 2;

	DEBUG_MSG("> LLCP Accept Socket: ");

	_net_nfc_util_alloc_mem(context, sizeof(controller_context_s));
	if (context == NULL)
	{
		DEBUG_MSG("mem alloc is failed");
		return false;
	}

	_net_nfc_util_alloc_mem(socket_info, sizeof(socket_info_s));
	if (socket_info == NULL)
	{
		DEBUG_MSG("mem alloc is failed");
		_net_nfc_util_free_mem(context);

		return false;
	}

	_net_nfc_util_alloc_mem(socket_info->data.buffer, 1024);
	if (socket_info->data.buffer == NULL)
	{
		DEBUG_MSG("mem alloc is failed");
		_net_nfc_util_free_mem(socket_info);
		_net_nfc_util_free_mem(context);

		return false;
	}

	/*psocket_info = _net_nfc_find_server_socket(socket);
	if (psocket_info == NULL) {
		*result = NET_NFC_LLCP_INVALID_SOCKET;
		return false;
	}
	*/

	socket_info->data.length = BUFFER_LENGTH_MAX;
	socket_info->socket_handle = socket;

	pthread_cond_t controller_cond = PTHREAD_COND_INITIALIZER;

	// TODO :: when context is freed ?

	context->controller_cond = &controller_cond;
	context->user_context = socket_info;
	context->result = 0;

	NET_NFC_OEM_CONTROLLER_LOCK;
	status = phLibNfc_Llcp_Accept(socket,
		&sOptions,
		(phNfc_sData_t *)&(socket_info->data),
		_net_nfc_llcp_accepted_socket_err_cb,
		_net_nfc_llcp_accept_cb,
		context);

	DEBUG_MSG("llcp accept operation result = [0x%x]", status);

	if (status == NFCSTATUS_SUCCESS || status == NFCSTATUS_PENDING)
	{
		if (status == NFCSTATUS_PENDING)
		{
			NET_NFC_OEM_CONTROLLER_WAIT(&controller_cond);
		}

		NET_NFC_OEM_CONTROLLER_UNLOCK;

		if (context->result != NFCSTATUS_SUCCESS)
		{
			DEBUG_MSG("accept socket is failed = [0x%x]", context->result);
			*result = _net_nfc_nxp_error_converter(context->result);
			return false;
		}
	}
	else
	{
		NET_NFC_OEM_CONTROLLER_UNLOCK;
		DEBUG_MSG("operation socket accepting is failed = [0x%x]", status);
		*result = _net_nfc_nxp_error_converter(status);
		return false;
	}

	return true;
}

static bool net_nfc_nxp_controller_llcp_connect(net_nfc_target_handle_s *handle, net_nfc_llcp_socket_t socket, uint8_t service_access_point, net_nfc_error_e *result, void *user_param)
{
	phLibNfc_Handle hRemoteDevice;
	NFCSTATUS status = NFCSTATUS_SUCCESS;

	if (result == NULL)
	{
		return false;
	}

	if (handle == NULL)
	{
		*result = NET_NFC_NULL_PARAMETER;
		return false;
	}

	if (!__net_nfc_is_valid_target_handle(handle))
	{
		*result = NET_NFC_INVALID_HANDLE;
		return false;
	}

	hRemoteDevice = handle->connection_id;

	NET_NFC_OEM_CONTROLLER_LOCK;
	status = phLibNfc_Llcp_Connect(hRemoteDevice,
		socket,
		service_access_point,
		_net_nfc_llcp_connect_sap_cb,
		user_param);

	DEBUG_MSG("llcp connect operation results = [0x%x]", status);

	NET_NFC_OEM_CONTROLLER_UNLOCK;

	*result = _net_nfc_nxp_error_converter(status);

	if (status != NFCSTATUS_SUCCESS && status != NFCSTATUS_PENDING)
	{
		DEBUG_MSG("operation socket connecting is failed = [0x%x]", status);
		return false;
	}

	return true;

}

static bool net_nfc_nxp_controller_llcp_connect_by_url(net_nfc_target_handle_s *handle, net_nfc_llcp_socket_t socket, uint8_t *service_access_name, net_nfc_error_e *result, void *user_param)
{
	phLibNfc_Handle hRemoteDevice;
	NFCSTATUS status = NFCSTATUS_SUCCESS;

	if (result == NULL)
	{
		return false;
	}

	if (handle == NULL)
	{
		*result = NET_NFC_NULL_PARAMETER;
		return false;
	}

	if (!__net_nfc_is_valid_target_handle(handle))
	{
		*result = NET_NFC_INVALID_HANDLE;
		return false;
	}

	hRemoteDevice = handle->connection_id;

	if (service_access_name == NULL)
	{
		return false;
	}

	phNfc_sData_t service_name = { service_access_name, strlen((char *)service_access_name) };

	NET_NFC_OEM_CONTROLLER_LOCK;

	DEBUG_MSG("socket = [0x%x] and service name [%s]", socket, service_access_name);

	status = phLibNfc_Llcp_ConnectByUri(hRemoteDevice,
		socket,
		(phNfc_sData_t *)&service_name,
		_net_nfc_llcp_connect_cb,
		user_param);

	DEBUG_MSG("llcp connect by url operation result = [0x%x]", status);

	NET_NFC_OEM_CONTROLLER_UNLOCK;

	*result = _net_nfc_nxp_error_converter(status);

	if (status != NFCSTATUS_SUCCESS && status != NFCSTATUS_PENDING)
	{
		DEBUG_MSG("operation socket connecting is failed = [0x%x]", status);
		return false;
	}

	return true;
}

static bool net_nfc_nxp_controller_llcp_send(net_nfc_target_handle_s *handle, net_nfc_llcp_socket_t socket, data_s *data, net_nfc_error_e *result, void *user_param)
{
	transfer_context_s *trans_context = NULL;
	phLibNfc_Handle hRemoteDevice;
	NFCSTATUS status = NFCSTATUS_SUCCESS;

	if (result == NULL)
	{
		return false;
	}

	if (handle == NULL)
	{
		*result = NET_NFC_NULL_PARAMETER;
		return false;
	}

	//socket_info_s *psocket_info = _net_nfc_find_server_socket(socket);
	//net_nfc_llcp_socket_t real_socket;

	/*if (psocket_info == NULL ) {
		*result = NET_NFC_LLCP_INVALID_SOCKET;
		return false;
	}*/

	/*if (psocket_info->socket_handle_incomming == 0){
		real_socket = socket;
	}
	else {
		real_socket = psocket_info->socket_handle_incomming;
	}*/

	/* this context and data copy is requried because the data comes from parameter will be freed after this functions */
	_net_nfc_util_alloc_mem(trans_context, sizeof(transfer_context_s));
	if (trans_context == NULL)
	{
		*result = NET_NFC_ALLOC_FAIL;
		return false;
	}

	_net_nfc_util_alloc_mem(trans_context->data.buffer, data->length);
	if (trans_context->data.buffer == NULL)
	{
		_net_nfc_util_free_mem(trans_context);
		*result = NET_NFC_ALLOC_FAIL;

		return false;
	}
	memcpy(trans_context->data.buffer, data->buffer, data->length);
	trans_context->data.length = data->length;

	trans_context->user_param = user_param;
	trans_context->oal_socket = socket;
	/* -- data copy is finished -- */

	hRemoteDevice = handle->connection_id;

	NET_NFC_OEM_CONTROLLER_LOCK;

	status = phLibNfc_Llcp_Send(hRemoteDevice, socket, (phNfc_sData_t *)&(trans_context->data), _net_nfc_llcp_send_cb, trans_context);

	DEBUG_MSG("llcp send operation result = [0x%x]", status);

	NET_NFC_OEM_CONTROLLER_UNLOCK;

	*result = _net_nfc_nxp_error_converter(status);
	if (status != NFCSTATUS_SUCCESS && status != NFCSTATUS_PENDING)
	{
		DEBUG_MSG("operation phLibNfc_Llcp_Send is failed = [0x%x]", status);
		return false;
	}

	return true;
}

static bool net_nfc_nxp_controller_llcp_send_to(net_nfc_target_handle_s *handle, net_nfc_llcp_socket_t socket, data_s *data, uint8_t service_access_point, net_nfc_error_e *result, void *user_param)
{
	transfer_context_s *trans_context = NULL;
	phLibNfc_Handle hRemoteDevice;
	NFCSTATUS status = NFCSTATUS_SUCCESS;

	if (result == NULL)
	{
		return false;
	}

	if (handle == NULL)
	{
		*result = NET_NFC_NULL_PARAMETER;
		return false;
	}

	if (!__net_nfc_is_valid_target_handle(handle))
	{
		*result = NET_NFC_INVALID_HANDLE;
		return false;
	}

	/* this context and data copy is requried because the data comes from parameter will be freed after this functions */
	_net_nfc_util_alloc_mem(trans_context, sizeof(transfer_context_s));
	if (trans_context == NULL)
	{
		*result = NET_NFC_ALLOC_FAIL;
		return false;
	}

	_net_nfc_util_alloc_mem(trans_context->data.buffer, data->length);
	if (trans_context->data.buffer == NULL)
	{
		_net_nfc_util_free_mem(trans_context);
		*result = NET_NFC_ALLOC_FAIL;
		return false;
	}
	memcpy(trans_context->data.buffer, data->buffer, data->length);
	trans_context->data.length = data->length;

	trans_context->user_param = user_param;
	trans_context->oal_socket = socket;
	/* -- data copy is finished -- */

	hRemoteDevice = handle->connection_id;

	NET_NFC_OEM_CONTROLLER_LOCK;

	status = phLibNfc_Llcp_SendTo(hRemoteDevice, socket, service_access_point, (phNfc_sData_t *)&(trans_context->data), _net_nfc_llcp_sendTo_cb, trans_context);

	DEBUG_MSG("llcp send to operation result = [0x%x]", status);

	NET_NFC_OEM_CONTROLLER_UNLOCK;

	*result = _net_nfc_nxp_error_converter(status);

	if (status != NFCSTATUS_SUCCESS && status != NFCSTATUS_PENDING)
	{
		DEBUG_MSG("operation phLibNfc_Llcp_SendTo is failed = [0x%x]", status);
		return false;
	}

	return true;
}

static bool net_nfc_nxp_controller_llcp_recv(net_nfc_target_handle_s *handle, net_nfc_llcp_socket_t socket, data_s *data, net_nfc_error_e *result, void *user_param)
{
	phLibNfc_Handle hRemoteDevice;
	NFCSTATUS status = NFCSTATUS_SUCCESS;

	if (result == NULL)
	{
		return false;
	}

	if (handle == NULL || data == NULL)
	{
		*result = NET_NFC_NULL_PARAMETER;
		return false;
	}

	if (!__net_nfc_is_valid_target_handle(handle))
	{
		*result = NET_NFC_INVALID_HANDLE;
		return false;
	}

	*result = NET_NFC_OK;

	hRemoteDevice = handle->connection_id;

	/*socket_info_s *psocket_info = _net_nfc_find_server_socket(socket);
	 //net_nfc_llcp_socket_t real_socket;

	if (psocket_info == NULL) {
		*result = NET_NFC_LLCP_INVALID_SOCKET;
		return false;
	}*/

	/*if (psocket_info->socket_handle_incomming == 0){
		real_socket = socket;
	}
	else {
		real_socket = psocket_info->socket_handle_incomming;
	}*/

	NET_NFC_OEM_CONTROLLER_LOCK;

	status = phLibNfc_Llcp_Recv(hRemoteDevice,
		socket,
		(phNfc_sData_t *)data,
		_net_nfc_llcp_recv_cb,
		user_param);

	DEBUG_MSG("llcp recv operation result = [0x%x]", status);

	NET_NFC_OEM_CONTROLLER_UNLOCK;

	*result = _net_nfc_nxp_error_converter(status);

	if (status != NFCSTATUS_SUCCESS && status != NFCSTATUS_PENDING)
	{
		DEBUG_MSG("operation phLibNfc_Llcp_Recv is failed = [0x%x]", status);
		return false;
	}

	return true;
}

static bool net_nfc_nxp_controller_llcp_recv_from(net_nfc_target_handle_s *handle, net_nfc_llcp_socket_t socket, data_s *data, net_nfc_error_e *result, void *user_param)
{
	NFCSTATUS status = NFCSTATUS_SUCCESS;

	if (result == NULL)
	{
		return false;
	}

	if (handle == NULL || data == NULL)
	{
		*result = NET_NFC_NULL_PARAMETER;
		return false;
	}
	if (!__net_nfc_is_valid_target_handle(handle))
	{
		*result = NET_NFC_INVALID_HANDLE;
		return false;
	}

	*result = NET_NFC_OK;

	NET_NFC_OEM_CONTROLLER_LOCK;

	status = phLibNfc_Llcp_RecvFrom(handle->connection_id, socket,
		(phNfc_sData_t *)data,
		_net_nfc_llcp_recv_from_cb,
		user_param);

	DEBUG_MSG("llcp recv from operation result = [0x%x]", status);

	NET_NFC_OEM_CONTROLLER_UNLOCK;

	*result = _net_nfc_nxp_error_converter(status);

	if (status != NFCSTATUS_SUCCESS && status != NFCSTATUS_PENDING)
	{
		DEBUG_MSG("operation phLibNfc_Llcp_Recv is failed = [0x%x]", status);
		return false;
	}

	return true;
}

static bool net_nfc_nxp_controller_llcp_disconnect(net_nfc_target_handle_s *handle, net_nfc_llcp_socket_t socket, net_nfc_error_e *result, void *user_param)
{
	phLibNfc_Handle hRemoteDevice;
	NFCSTATUS status = NFCSTATUS_SUCCESS;

	if (result == NULL)
	{
		return false;
	}

	if (handle == NULL)
	{
		*result = NET_NFC_NULL_PARAMETER;
		return false;
	}

	if (!__net_nfc_is_valid_target_handle(handle))
	{
		*result = NET_NFC_INVALID_HANDLE;
		return false;
	}

	/*socket_info_s *psocket_info = _net_nfc_find_server_socket(socket);
	 //net_nfc_llcp_socket_t real_socket;

	if (psocket_info == NULL) {
		*result = NET_NFC_LLCP_INVALID_SOCKET;
		return false;
	}*/

	/*if (psocket_info->socket_handle_incomming == 0){
		real_socket = socket;
	}
	else {
		real_socket = psocket_info->socket_handle_incomming;
	}*/

	*result = NET_NFC_OK;

	hRemoteDevice = handle->connection_id;

	NET_NFC_OEM_CONTROLLER_LOCK;

	DEBUG_MSG("disconnect socket = [0x%x]", socket);

	status = phLibNfc_Llcp_Disconnect(hRemoteDevice, socket,
		_net_nfc_llcp_disconnect_cb,
		user_param);

	DEBUG_MSG("llcp disconnect operation result = [0x%x]", status);

	NET_NFC_OEM_CONTROLLER_UNLOCK;

	*result = _net_nfc_nxp_error_converter(status);

	if (status != NFCSTATUS_SUCCESS && status != NFCSTATUS_PENDING)
	{
		DEBUG_MSG("operation net_nfc_nxp_controller_llcp_disconnect is failed = [0x%x]", status);
		return false;
	}

	return true;
}

static bool net_nfc_nxp_controller_llcp_socket_close(net_nfc_llcp_socket_t socket, net_nfc_error_e *result)
{
	NFCSTATUS status = NFCSTATUS_SUCCESS;
//	net_nfc_llcp_socket_t incomming_socket ;

	DEBUG_MSG("try to close socket = [0x%x]", socket);

	/*socket_info_s *psocket_info = _net_nfc_find_server_socket(socket);

	if (psocket_info == NULL) {
		*result = NET_NFC_LLCP_INVALID_SOCKET;
		return false;
	}*/

	//incomming_socket = psocket_info->socket_handle_incomming;
	_net_nfc_remove_socket_slot(socket);

	/*if (incomming_socket != 0){
		NET_NFC_OEM_CONTROLLER_LOCK;
		status = phLibNfc_Llcp_Close(incomming_socket);
		NET_NFC_OEM_CONTROLLER_UNLOCK;
	}*/

	NET_NFC_OEM_CONTROLLER_LOCK;
	status = phLibNfc_Llcp_Close(socket);
	NET_NFC_OEM_CONTROLLER_UNLOCK;

	*result = _net_nfc_nxp_error_converter(status);

	if (status == NFCSTATUS_SUCCESS || status == NFCSTATUS_PENDING)
	{
		DEBUG_MSG("closing socket is good");
		return true;
	}
	else
	{
		DEBUG_MSG("closing socket is failed = [0x%x]", status);
		return false;
	}
}

static bool net_nfc_nxp_controller_llcp_reject(net_nfc_target_handle_s *handle, net_nfc_llcp_socket_t socket, net_nfc_error_e *result)
{
	phLibNfc_Handle hRemoteDevice;
	NFCSTATUS status = NFCSTATUS_SUCCESS;

	pthread_cond_t controller_cond = PTHREAD_COND_INITIALIZER;
	controller_context_s context = { &controller_cond, NULL, 0 };

	if (result == NULL)
	{
		return false;
	}

	if (handle == NULL)
	{
		*result = NET_NFC_NULL_PARAMETER;
		return false;
	}

	if (!__net_nfc_is_valid_target_handle(handle))
	{
		*result = NET_NFC_INVALID_HANDLE;
		return false;
	}

	*result = NET_NFC_OK;

	/*socket_info_s *psocket_info = _net_nfc_find_server_socket(socket);
	if (psocket_info == NULL) {
		*result = NET_NFC_LLCP_INVALID_SOCKET;
		return false;
	}*/

	hRemoteDevice = handle->connection_id;

	NET_NFC_OEM_CONTROLLER_LOCK;

	DEBUG_MSG("reject socket = [0x%x]", socket);

	status = phLibNfc_Llcp_Reject(hRemoteDevice, socket,
		_net_nfc_llcp_reject_cb,
		&context);

	DEBUG_MSG("llcp disconnect operation result = [0x%x]", status);

	if (status == NFCSTATUS_SUCCESS || status == NFCSTATUS_PENDING)
	{

		if (status == NFCSTATUS_PENDING)
		{
			NET_NFC_OEM_CONTROLLER_WAIT(&controller_cond);
		}

		NET_NFC_OEM_CONTROLLER_UNLOCK;

		if (context.result != NFCSTATUS_SUCCESS)
		{
			DEBUG_MSG("llcp socket reject is failed = [0x%x]", context.result);
			*result = _net_nfc_nxp_error_converter(context.result);
			return false;
		}
	}
	else
	{
		DEBUG_MSG("operation socket reject by llcp is failed = [0x%x]", status);

		NET_NFC_OEM_CONTROLLER_UNLOCK;

		*result = _net_nfc_nxp_error_converter(status);
		return false;
	}

	return true;
}

static bool net_nfc_nxp_controller_llcp_get_remote_config(net_nfc_target_handle_s *handle, net_nfc_llcp_config_info_s *config, net_nfc_error_e *result)
{
	phLibNfc_Handle hRemoteDevice;
	phLibNfc_Llcp_sLinkParameters_t info;

	NFCSTATUS status = NFCSTATUS_SUCCESS;

	if (result == NULL)
	{
		return false;
	}

	if (handle == NULL || config == NULL)
	{
		*result = NET_NFC_NULL_PARAMETER;
		return false;
	}

	if (!__net_nfc_is_valid_target_handle(handle))
	{
		*result = NET_NFC_INVALID_HANDLE;
		return false;
	}

	*result = NET_NFC_OK;

	hRemoteDevice = handle->connection_id;

	NET_NFC_OEM_CONTROLLER_LOCK;

	DEBUG_MSG("get remote info..");

	status = phLibNfc_Llcp_GetRemoteInfo(hRemoteDevice, &info);

	DEBUG_MSG("llcp get remote info operation result = [0x%x]", status);

	NET_NFC_OEM_CONTROLLER_UNLOCK;

	if (status != NFCSTATUS_SUCCESS)
	{
		DEBUG_MSG("operation getinfo by llcp is failed = [0x%x]", status);
		*result = _net_nfc_nxp_error_converter(status);
		return false;
	}

	config->lto = info.lto;
	config->miu = info.miu;
	config->option = info.option;
	config->wks = info.wks;

	return true;

}

static bool net_nfc_nxp_controller_llcp_get_remote_socket_info(net_nfc_target_handle_s *handle, net_nfc_llcp_socket_t socket, net_nfc_llcp_socket_option_s *option, net_nfc_error_e *result)
{
	NFCSTATUS status = NFCSTATUS_SUCCESS;
	phLibNfc_Llcp_sSocketOptions_t psRemoteOptions;

	if (result == NULL)
	{
		return false;
	}

	if (handle == NULL || option == NULL)
	{
		*result = NET_NFC_NULL_PARAMETER;
		return false;
	}

	if (!__net_nfc_is_valid_target_handle(handle))
	{
		*result = NET_NFC_INVALID_HANDLE;
		return false;
	}

	//socket_info_s *psocket_info = _net_nfc_find_server_socket(socket);
	//net_nfc_llcp_socket_t real_socket;

	/*if (psocket_info == NULL ) {
		*result = NET_NFC_LLCP_INVALID_SOCKET;
		return false;
	}

	if (psocket_info->socket_handle_incomming == 0){
		real_socket = socket;
	}
	else {
		real_socket = psocket_info->socket_handle_incomming;
	}*/

	NET_NFC_OEM_CONTROLLER_LOCK;
	status = phLibNfc_Llcp_SocketGetRemoteOptions(handle->connection_id, socket, &psRemoteOptions);
	NET_NFC_OEM_CONTROLLER_UNLOCK;

	if (status != NFCSTATUS_SUCCESS)
	{
		DEBUG_MSG("operation getting socket info is failed = [0x%x]", status);
		*result = _net_nfc_nxp_error_converter(status);
		return false;
	}

	option->miu = psRemoteOptions.miu;
	option->rw = psRemoteOptions.rw;
	option->type = NET_NFC_LLCP_SOCKET_TYPE_CONNECTIONORIENTED;

	return NET_NFC_OK;
}

/*
static void net_nfc_nxp_controller_print_tag_info(phNfc_uRemoteDevInfo_t *devInfo, phNfc_eRemDevType_t RemDevType)
{
	switch(RemDevType)
	{
		case phNfc_eISO14443_A_PICC :
		case phNfc_eISO14443_4A_PICC :
		case phNfc_eISO14443_3A_PICC :
		case phNfc_eMifare_PICC :
		{
			phNfc_sIso14443AInfo_t* info = (phNfc_sIso14443AInfo_t *)(devInfo);

			DEBUG_MSG("ISO14443 UID : ");
			DEBUG_MSG("\tUID : ");
			DEBUG_MSG_PRINT_BUFFER(info->Uid, info->UidLength);

			DEBUG_MSG("\tAppData");
			DEBUG_MSG_PRINT_BUFFER(info->AppData, info->AppDataLength);

			DEBUG_MSG("\tSak = [0x%x]", info->Sak);

			DEBUG_MSG("\tAtqA");
			DEBUG_MSG_PRINT_BUFFER(info->AtqA, PHHAL_ATQA_LENGTH);

			DEBUG_MSG("\tMaxDataRate = [0x%x]", info->MaxDataRate);

			DEBUG_MSG("\tFwi_Sfgt = [0x%x]", info->Fwi_Sfgt);

		}
		break;

		case phNfc_eISO14443_B_PICC :
		case phNfc_eISO14443_4B_PICC :
		case phNfc_eISO14443_BPrime_PICC :
		{
		}
		break;

		case phNfc_eFelica_PICC :
		{
		}
		break;
		case phNfc_eJewel_PICC :
		{
		}
		break;
		case phNfc_eISO15693_PICC :
		{
		}
		break;

		case phNfc_eNfcIP1_Target :
		case phNfc_eNfcIP1_Initiator :
		{
		}
		break;

		default:
		break;
	}
}
*/

static int net_nfc_nxp_controller_tag_device_info(phNfc_uRemoteDevInfo_t *devInfo, phNfc_eRemDevType_t RemDevType, uint8_t **buffer, uint32_t *buffer_length)
{
	int number_of_keys = 0;

	switch (RemDevType)
	{
	case phNfc_eISO14443_A_PICC :
	case phNfc_eISO14443_4A_PICC :
	case phNfc_eISO14443_3A_PICC :
	case phNfc_eMifare_PICC :
		{
			net_nfc_nxp_controller_device_info_ISO14443A(devInfo, buffer, buffer_length);
			number_of_keys = KEYS_ISO14443A_MAX;
		}
		break;

	case phNfc_eISO14443_B_PICC :
	case phNfc_eISO14443_4B_PICC :
	case phNfc_eISO14443_BPrime_PICC :
		{
			net_nfc_nxp_controller_device_info_ISO14443B(devInfo, buffer, buffer_length);
			number_of_keys = KEYS_ISO14443B_MAX;
		}
		break;

	case phNfc_eFelica_PICC :
		{
			net_nfc_nxp_controller_device_info_Felica(devInfo, buffer, buffer_length);
			number_of_keys = KEYS_FELICA_MAX;
		}
		break;
	case phNfc_eJewel_PICC :
		{
			net_nfc_nxp_controller_device_info_Jewel(devInfo, buffer, buffer_length);
			number_of_keys = KEYS_JEWEL_MAX;
		}
		break;
	case phNfc_eISO15693_PICC :
		{
			net_nfc_nxp_controller_device_info_ISO15693(devInfo, buffer, buffer_length);
			number_of_keys = KEYS_ISO15693_MAX;
		}
		break;

	case phNfc_eNfcIP1_Target :
	case phNfc_eNfcIP1_Initiator :
		{
			net_nfc_nxp_controller_device_info_NFCIP(devInfo, buffer, buffer_length);
			number_of_keys = KEYS_NFCIP_MAX;
		}
		break;

	default :
		break;
	}

	return number_of_keys;
}

static void net_nfc_nxp_controller_device_info_ISO14443A(phNfc_uRemoteDevInfo_t *devInfo, uint8_t **buffer, uint32_t *buffer_length)
{
	phNfc_sIso14443AInfo_t *info = (phNfc_sIso14443AInfo_t *)(devInfo);

	/*                          key_name:                       UID                      APP_DATA                    SAK                   ATQA                  MAX_DATA_RATE       FWI_SFGT*/
//	*buffer_length = strlen(keys_ISO14443A)  + info->UidLength + info->AppDataLength + sizeof(uint8_t) + PHHAL_ATQA_LENGTH + sizeof(uint8_t) + sizeof(uint8_t) ;
	*buffer_length = strlen(keys_ISO14443A) + KEYS_ISO14443A_MAX + info->UidLength + info->AppDataLength + sizeof(uint8_t) + PHHAL_ATQA_LENGTH + sizeof(uint8_t) + sizeof(uint8_t);

	_net_nfc_util_alloc_mem(*buffer, *buffer_length);
	if (*buffer != NULL)
	{
		uint8_t *pBuffer = *buffer;
		int key_length = 0;

		//UID

		// set key length
		key_length = strlen("UID");
		*pBuffer = (uint8_t)key_length;
		pBuffer = pBuffer + 1;

		// set key
		memcpy(pBuffer, "UID", key_length);
		pBuffer = pBuffer + key_length;

		// set length
		*pBuffer = (uint8_t)info->UidLength;
		pBuffer = pBuffer + 1;

		// set value
		if (info->UidLength > 0)
		{
			memcpy(pBuffer, info->Uid, info->UidLength);
			pBuffer = pBuffer + info->UidLength;
		}

		//APP_DATA

		key_length = strlen("APP_DATA");
		*pBuffer = (uint8_t)key_length;
		pBuffer = pBuffer + 1;

		// set key
		memcpy(pBuffer, "APP_DATA", key_length);
		pBuffer = pBuffer + key_length;

		// set length
		*pBuffer = (uint8_t)info->AppDataLength;
		pBuffer = pBuffer + 1;

		if (info->AppDataLength != 0)
		{
			memcpy(pBuffer, info->AppData, info->AppDataLength);
			pBuffer = pBuffer + info->AppDataLength;
		}

		// SAK

		key_length = strlen("SAK");
		*pBuffer = (uint8_t)key_length;
		pBuffer = pBuffer + 1;

		memcpy(pBuffer, "SAK", key_length);
		pBuffer = pBuffer + key_length;

		// set length
		*pBuffer = (uint8_t)sizeof(uint8_t);
		pBuffer = pBuffer + 1;

		// set value
		*pBuffer = info->Sak;
		pBuffer = pBuffer + sizeof(uint8_t);

		DEBUG_MSG("MIFARE SAK = [0x%x]", info->Sak);

		// ATQA

		key_length = strlen("ATQA");
		*pBuffer = (uint8_t)key_length;
		pBuffer = pBuffer + 1;

		// set key
		memcpy(pBuffer, "ATQA", key_length);
		pBuffer = pBuffer + key_length;

		// set length
		*pBuffer = (uint8_t)PHHAL_ATQA_LENGTH;
		pBuffer = pBuffer + 1;

		// set value
		memcpy(pBuffer, info->AtqA, PHHAL_ATQA_LENGTH);
		pBuffer = pBuffer + PHHAL_ATQA_LENGTH;

		// MAX_DATA_RATE

		key_length = strlen("MAX_DATA_RATE");
		*pBuffer = (uint8_t)key_length;
		pBuffer = pBuffer + 1;

		// set key
		memcpy(pBuffer, "MAX_DATA_RATE", key_length);
		pBuffer = pBuffer + key_length;

		// set length
		*pBuffer = (uint8_t)sizeof(uint8_t);
		pBuffer = pBuffer + 1;

		// set value
		*pBuffer = info->MaxDataRate;
		pBuffer = pBuffer + sizeof(uint8_t);

		// FWI_SFGT

		key_length = strlen("FWI_SFGT");
		*pBuffer = (uint8_t)key_length;
		pBuffer = pBuffer + 1;

		// set key
		memcpy(pBuffer, "FWI_SFGT", key_length);
		pBuffer = pBuffer + key_length;

		// set length
		*pBuffer = (uint8_t)sizeof(uint8_t);
		pBuffer = pBuffer + 1;

		// set value
		*pBuffer = info->Fwi_Sfgt;
	}
}

static void net_nfc_nxp_controller_device_info_ISO14443B(phNfc_uRemoteDevInfo_t *devInfo, uint8_t **buffer, uint32_t *buffer_length)
{
	phNfc_sIso14443BInfo_t *info = (phNfc_sIso14443BInfo_t *)(devInfo);

	/*                          key length key_name        value  length (1byte)           UID                      APP_DATA                        PROTOCOL_INFO                   HI_LAYER_RESPONSE                  AFI           MAX_DATA_RATE*/
	*buffer_length = strlen(keys_ISO14443B) + KEYS_ISO14443B_MAX + NET_NFC_PUPI_LENGTH + NET_NFC_APP_DATA_B_LENGTH + NET_NFC_PROT_INFO_B_LENGTH + info->HiLayerRespLength + sizeof(uint8_t) + sizeof(uint8_t);

	_net_nfc_util_alloc_mem(*buffer, *buffer_length);
	if (*buffer != NULL)
	{
		uint8_t *pBuffer = *buffer;
		int key_length = 0;

		//UID

		// set key length
		key_length = strlen("UID");
		*pBuffer = (uint8_t)key_length;
		pBuffer = pBuffer + 1;

		// set key
		memcpy(pBuffer, "UID", key_length);
		pBuffer = pBuffer + key_length;

		// set length
		*pBuffer = NET_NFC_PUPI_LENGTH;
		pBuffer = pBuffer + 1;

		// set value
		memcpy(pBuffer, info->AtqB.AtqResInfo.Pupi, NET_NFC_PUPI_LENGTH);
		pBuffer = pBuffer + NET_NFC_PUPI_LENGTH;

		//APP_DATA

		// set key length
		key_length = strlen("APP_DATA");
		*pBuffer = (uint8_t)key_length;
		pBuffer = pBuffer + 1;

		// set key
		memcpy(pBuffer, "APP_DATA", key_length);
		pBuffer = pBuffer + key_length;

		// set length
		*pBuffer = NET_NFC_APP_DATA_B_LENGTH;
		pBuffer = pBuffer + 1;

		// set value
		memcpy(pBuffer, info->AtqB.AtqResInfo.AppData, NET_NFC_APP_DATA_B_LENGTH);
		pBuffer = pBuffer + NET_NFC_APP_DATA_B_LENGTH;

		// PROTOCOL_INFO

		// set key length
		key_length = strlen("PROTOCOL_INFO");
		*pBuffer = (uint8_t)key_length;
		pBuffer = pBuffer + 1;

		// set key
		memcpy(pBuffer, "PROTOCOL_INFO", key_length);
		pBuffer = pBuffer + key_length;

		// set length
		*pBuffer = NET_NFC_PROT_INFO_B_LENGTH;
		pBuffer = pBuffer + 1;

		// set value
		memcpy(pBuffer, info->AtqB.AtqResInfo.ProtInfo, NET_NFC_PROT_INFO_B_LENGTH);
		pBuffer = pBuffer + NET_NFC_PROT_INFO_B_LENGTH;

		// HI_LAYER_RESPONSE

		// set key length
		key_length = strlen("HI_LAYER_RESPONSE");
		*pBuffer = (uint8_t)key_length;
		pBuffer = pBuffer + 1;

		// set key
		memcpy(pBuffer, "HI_LAYER_RESPONSE", key_length);
		pBuffer = pBuffer + key_length;

		// set length
		*pBuffer = info->HiLayerRespLength;
		pBuffer = pBuffer + 1;

		// set value
		if (info->HiLayerRespLength > 0)
		{
			memcpy(pBuffer, info->HiLayerResp, info->HiLayerRespLength);
			pBuffer = pBuffer + info->HiLayerRespLength;
		}

		// AFI

		// set key length
		key_length = strlen("AFI");
		*pBuffer = (uint8_t)key_length;
		pBuffer = pBuffer + 1;

		// set key
		memcpy(pBuffer, "AFI", key_length);
		pBuffer = pBuffer + key_length;

		// set length
		*pBuffer = sizeof(uint8_t);
		pBuffer = pBuffer + 1;

		// set value
		*pBuffer = info->Afi;
		pBuffer = pBuffer + sizeof(uint8_t);

		// MAX_DATA_RATE

		// set key length
		key_length = strlen("MAX_DATA_RATE");
		*pBuffer = (uint8_t)key_length;
		pBuffer = pBuffer + 1;

		// set key
		memcpy(pBuffer, "MAX_DATA_RATE", key_length);
		pBuffer = pBuffer + key_length;

		// set length
		*pBuffer = sizeof(uint8_t);
		pBuffer = pBuffer + 1;

		// set value
		*pBuffer = info->MaxDataRate;
	}
}

static void net_nfc_nxp_controller_device_info_Jewel(phNfc_uRemoteDevInfo_t *devInfo, uint8_t **buffer, uint32_t *buffer_length)
{
	phNfc_sJewelInfo_t *info = (phNfc_sJewelInfo_t *)(devInfo);

	/*                          key_length key_name     length (1byte)         UID              HEADER_ROM0   HEADER_ROM1 */
	*buffer_length = strlen(keys_JEWEL) + KEYS_JEWEL_MAX + info->UidLength + sizeof(uint8_t) + sizeof(uint8_t);

	_net_nfc_util_alloc_mem(*buffer, *buffer_length);
	if (*buffer != NULL)
	{
		uint8_t *pBuffer = *buffer;
		int key_length = 0;

		//UID

		// set key length
		key_length = strlen("UID");
		*pBuffer = (uint8_t)key_length;
		pBuffer = pBuffer + 1;

		// set key
		memcpy(pBuffer, "UID", key_length);
		pBuffer = pBuffer + key_length;

		// set length
		*pBuffer = info->UidLength;
		pBuffer = pBuffer + 1;

		// set value
		if (info->UidLength > 0)
		{
			memcpy(pBuffer, info->Uid, info->UidLength);
			pBuffer = pBuffer + info->UidLength;
		}

		// HEADER_ROM0

		// set key length
		key_length = strlen("HEADER_ROM0");
		*pBuffer = (uint8_t)key_length;
		pBuffer = pBuffer + 1;

		// set key
		memcpy(pBuffer, "HEADER_ROM0", key_length);
		pBuffer = pBuffer + key_length;

		// set length
		*pBuffer = sizeof(uint8_t);
		pBuffer = pBuffer + 1;

		// set value
		*pBuffer = info->HeaderRom0;
		pBuffer = pBuffer + sizeof(uint8_t);

		// HEADER_ROM1

		// set key length
		key_length = strlen("HEADER_ROM1");
		*pBuffer = (uint8_t)key_length;
		pBuffer = pBuffer + 1;

		// set key
		memcpy(pBuffer, "HEADER_ROM1", key_length);
		pBuffer = pBuffer + key_length;

		// set length
		*pBuffer = sizeof(uint8_t);
		pBuffer = pBuffer + 1;

		// set value
		*pBuffer = info->HeaderRom1;

		DEBUG_MSG("header rom 0 = [0x%X] header rom 1 = [0x%X]", info->HeaderRom0, info->HeaderRom1);
	}
}

static void net_nfc_nxp_controller_device_info_Felica(phNfc_uRemoteDevInfo_t *devInfo, uint8_t **buffer, uint32_t *buffer_length)
{
	phNfc_sFelicaInfo_t *info = (phNfc_sFelicaInfo_t *)(devInfo);

	/*                         key length key name        length (1byte)        UID                      PM                       SYSTEM_CODE */
	*buffer_length = strlen(keys_FELICA) + KEYS_FELICA_MAX + info->IDmLength + NET_NFC_FEL_PM_LEN + NET_NFC_FEL_SYS_CODE_LEN;

	_net_nfc_util_alloc_mem(*buffer, *buffer_length);
	if (*buffer != NULL)
	{
		uint8_t *pBuffer = *buffer;
		int key_length = 0;

		//UID

		// set key length
		key_length = strlen("IDm");
		*pBuffer = (uint8_t)key_length;
		pBuffer = pBuffer + 1;

		// set key
		memcpy(pBuffer, "IDm", key_length);
		pBuffer = pBuffer + key_length;

		// set length
		*pBuffer = info->IDmLength;
		pBuffer = pBuffer + 1;

		// set value
		if (info->IDmLength > 0)
		{
			memcpy(pBuffer, info->IDm, info->IDmLength);
			pBuffer = pBuffer + info->IDmLength;
		}

		// PM

		// set key length
		key_length = strlen("PMm");
		*pBuffer = (uint8_t)key_length;
		pBuffer = pBuffer + 1;

		// set key
		memcpy(pBuffer, "PMm", key_length);
		pBuffer = pBuffer + key_length;

		// set length
		*pBuffer = NET_NFC_FEL_PM_LEN;
		pBuffer = pBuffer + 1;

		// set value
		memcpy(pBuffer, info->PMm, NET_NFC_FEL_PM_LEN);
		pBuffer = pBuffer + NET_NFC_FEL_PM_LEN;

		// SYSTEM_CODE

		// set key length
		key_length = strlen("SYSTEM_CODE");
		*pBuffer = (uint8_t)key_length;
		pBuffer = pBuffer + 1;

		// set key
		memcpy(pBuffer, "SYSTEM_CODE", key_length);
		pBuffer = pBuffer + key_length;

		// set length
		*pBuffer = NET_NFC_FEL_SYS_CODE_LEN;
		pBuffer = pBuffer + 1;

		// set value
		memcpy(pBuffer, info->SystemCode, NET_NFC_FEL_SYS_CODE_LEN);
	}
}

static void net_nfc_nxp_controller_device_info_ISO15693(phNfc_uRemoteDevInfo_t *devInfo, uint8_t **buffer, uint32_t *buffer_length)
{
	phNfc_sIso15693Info_t *info = (phNfc_sIso15693Info_t *)(devInfo);

	/*                          key length key_name          length (1byte)                   UID                      DSF_ID       FLAGS                 AFI */
	*buffer_length = strlen(keys_ISO15693) + KEYS_ISO15693_MAX + info->UidLength + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(uint8_t);

	_net_nfc_util_alloc_mem(*buffer, *buffer_length);
	if (*buffer != NULL)
	{
		uint8_t *pBuffer = *buffer;
		int key_length = 0;

		//UID

		// set key length
		key_length = strlen("UID");
		*pBuffer = (uint8_t)key_length;
		pBuffer = pBuffer + 1;

		// set key
		memcpy(pBuffer, "UID", key_length);
		pBuffer = pBuffer + key_length;

		// set legnth
		*pBuffer = info->UidLength;
		pBuffer = pBuffer + 1;

		// set value
		if (info->UidLength > 0)
		{
			memcpy(pBuffer, info->Uid, info->UidLength);
			pBuffer = pBuffer + info->UidLength;
		}

		// DSF_ID

		// set key length
		key_length = strlen("DSF_ID");
		*pBuffer = (uint8_t)key_length;
		pBuffer = pBuffer + 1;

		// set key
		memcpy(pBuffer, "DSF_ID", key_length);
		pBuffer = pBuffer + key_length;

		// set length
		*pBuffer = sizeof(uint8_t);
		pBuffer = pBuffer + 1;

		// set value
		*pBuffer = info->Dsfid;
		pBuffer = pBuffer + sizeof(uint8_t);

		// FLAGS

		// set key length
		key_length = strlen("FLAGS");
		*pBuffer = (uint8_t)key_length;
		pBuffer = pBuffer + 1;

		// set key
		memcpy(pBuffer, "FLAGS", key_length);
		pBuffer = pBuffer + key_length;

		// set length
		*pBuffer = sizeof(uint8_t);
		pBuffer = pBuffer + 1;

		// set value

		*pBuffer = info->Flags;
		pBuffer = pBuffer + sizeof(uint8_t);

		// AFI

		// set key length
		key_length = strlen("AFI");
		*pBuffer = (uint8_t)key_length;
		pBuffer = pBuffer + 1;

		// set key
		memcpy(pBuffer, "AFI", key_length);
		pBuffer = pBuffer + key_length;

		// set length
		*pBuffer = sizeof(uint8_t);
		pBuffer = pBuffer + 1;

		// set value
		*pBuffer = info->Afi;
	}
}

static void net_nfc_nxp_controller_device_info_NFCIP(phNfc_uRemoteDevInfo_t *devInfo, uint8_t **buffer, uint32_t *buffer_length)
{
	phNfc_sNfcIPInfo_t *info = (phNfc_sNfcIPInfo_t *)(devInfo);

	/*                          key length key_name     length (1byte)      UID                              ATR_INFORMATION       SAK                 ATQA                    MAX_DATA_RATE */
	*buffer_length = strlen(keys_NFCIP) + KEYS_NFCIP_MAX + info->NFCID_Length + info->ATRInfo_Length + sizeof(uint8_t) + NET_NFC_ATQA_LENGTH + sizeof(uint16_t);

	_net_nfc_util_alloc_mem(*buffer, *buffer_length);
	if (*buffer != NULL)
	{
		uint8_t *pBuffer = *buffer;
		int key_length = 0;

		//UID

		// set key length
		key_length = strlen("UID");
		*pBuffer = (uint8_t)key_length;
		pBuffer = pBuffer + 1;

		// set key
		memcpy(pBuffer, "UID", key_length);
		pBuffer = pBuffer + key_length;

		// set length
		*pBuffer = info->NFCID_Length;
		pBuffer = pBuffer + 1;

		// set value
		if (info->NFCID_Length > 0)
		{
			memcpy(pBuffer, info->NFCID, info->NFCID_Length);
			pBuffer = pBuffer + info->NFCID_Length;
		}

		// ATR_INFORMATION

		// set key length
		key_length = strlen("ATR_INFORMATION");
		*pBuffer = (uint8_t)key_length;
		pBuffer = pBuffer + 1;

		// set key
		memcpy(pBuffer, "ATR_INFORMATION", key_length);
		pBuffer = pBuffer + key_length;

		// set length
		*pBuffer = info->ATRInfo_Length;
		pBuffer = pBuffer + 1;

		// set value
		if (info->ATRInfo_Length > 0)
		{
			memcpy(pBuffer, info->ATRInfo, info->ATRInfo_Length);
			pBuffer = pBuffer + info->ATRInfo_Length;
		}

		// SAK

		// set key length
		key_length = strlen("SAK");
		*pBuffer = (uint8_t)key_length;
		pBuffer = pBuffer + 1;

		// set key
		memcpy(pBuffer, "SAK", key_length);
		pBuffer = pBuffer + key_length;

		// set length
		*pBuffer = sizeof(uint8_t);
		pBuffer = pBuffer + 1;

		// set value
		*pBuffer = info->SelRes;
		pBuffer = pBuffer + sizeof(uint8_t);

		// ATR_INFORMATION

		// set key length
		key_length = strlen("ATQA");
		*pBuffer = (uint8_t)key_length;
		pBuffer = pBuffer + 1;

		// set key
		memcpy(pBuffer, "ATQA", key_length);
		pBuffer = pBuffer + key_length;

		// set length
		*pBuffer = NET_NFC_ATQA_LENGTH;
		pBuffer = pBuffer + 1;

		// set value
		memcpy(pBuffer, info->SenseRes, NET_NFC_ATQA_LENGTH);
		pBuffer = pBuffer + NET_NFC_ATQA_LENGTH;

		// MAX_DATA_RATE

		// set key length
		key_length = strlen("MAX_DATA_RATE");
		*pBuffer = (uint8_t)key_length;
		pBuffer = pBuffer + 1;

		// set key
		memcpy(pBuffer, "MAX_DATA_RATE", key_length);
		pBuffer = pBuffer + key_length;

		// set length
		*pBuffer = sizeof(uint16_t);
		pBuffer = pBuffer + 1;

		// set value
		if (info->Nfcip_Datarate == phNfc_eDataRate_106)
		{
			*pBuffer = 0;
			pBuffer = pBuffer + 1;

			*pBuffer = 106;
		}
		else if (info->Nfcip_Datarate == phNfc_eDataRate_212)
		{
			*pBuffer = 0;
			pBuffer = pBuffer + 1;

			*pBuffer = 212;
		}
		else if (info->Nfcip_Datarate == phNfc_eDataRate_424)
		{
			*pBuffer = (424 & 0xff00) >> 8;
			pBuffer = pBuffer + 1;

			*pBuffer = (424 & 0x00ff);
		}
		else
		{
			*pBuffer = 0;
			pBuffer = pBuffer + 1;

			*pBuffer = 106;
		}
	}
}

static bool net_nfc_nxp_controller_test_mode_on(net_nfc_error_e *result)
{
#if 0
	NFCSTATUS status = NFCSTATUS_SUCCESS;
	pthread_cond_t controller_cond = PTHREAD_COND_INITIALIZER;
	controller_context_s context = { &controller_cond, NULL, 0 };

	if (result == NULL)
	{
		return false;
	}

	*result = NET_NFC_OK;

	//_net_nfc_nxp_controller_lock_init();

	DEBUG_MSG("net_nfc_nxp_controller_test_mode_on \n");

	phLibNfc_Mgt_UnConfigureDriver(psHwRef.p_board_driver);

	DEBUG_MSG("Dal config \n");

	if (_net_nfc_dal_config() != true)
	{
		DEBUG_MSG("Port config is failed");
		return false;
	}

	NET_NFC_OEM_CONTROLLER_LOCK;
	if ((status = phLibNfc_Mgt_ConfigureTestMode(psHwRef.p_board_driver, _net_nfc_test_mode_on_cb, phLibNfc_TstMode_On, (void *)&context)) != NFCSTATUS_PENDING)
	{
		NET_NFC_OEM_CONTROLLER_UNLOCK;
		DEBUG_MSG("net_nfc_nxp_controller_test_mode_on is failed = [0x%x] \n", status);

		return false;
	}
	else
	{
		NET_NFC_OEM_CONTROLLER_WAIT(&controller_cond);
		NET_NFC_OEM_CONTROLLER_UNLOCK;

		DEBUG_MSG("net_nfc_nxp_controller_test_mode_on is in progress");

		if (context.result != NFCSTATUS_SUCCESS)
		{
			DEBUG_MSG("net_nfc_nxp_controller_test_mode_onis failed = [0x%x] \n", context.result);

			return false;
		}
	}

	DEBUG_MSG("net_nfc_nxp_controller_test_mode_on finished");
#endif
	return true;
}

static bool net_nfc_nxp_controller_test_mode_off(net_nfc_error_e *result)
{
#if 0
	NFCSTATUS status = NFCSTATUS_SUCCESS;
	pthread_cond_t controller_cond = PTHREAD_COND_INITIALIZER;
	controller_context_s context = { &controller_cond, NULL, 0 };

	if (result == NULL)
	{
		return false;
	}

	*result = NET_NFC_OK;

	DEBUG_MSG("net_nfc_nxp_controller_test_mode_off \n");

	NET_NFC_OEM_CONTROLLER_LOCK;
	if ((status = phLibNfc_Mgt_ConfigureTestMode(psHwRef.p_board_driver, _net_nfc_test_mode_off_cb, phLibNfc_TstMode_Off, (void *)&context)) != NFCSTATUS_PENDING)
	{
		NET_NFC_OEM_CONTROLLER_UNLOCK;
		DEBUG_MSG("phLibNfc_Mgt_ConfigureTestMode off is failed = [0x%x] \n", status);

		return false;
	}
	else
	{
		NET_NFC_OEM_CONTROLLER_WAIT(&controller_cond);
		NET_NFC_OEM_CONTROLLER_UNLOCK;

		DEBUG_MSG("phLibNfc_Mgt_ConfigureTestMode off is in progress");

		if (context.result != NFCSTATUS_SUCCESS)
		{
			DEBUG_MSG("phLibNfc_Mgt_ConfigureTestMode off is failed = [0x%x] \n", context.result);

			return false;
		}
	}

	DEBUG_MSG("phLibNfc_Mgt_ConfigureTestMode off finished");
#endif
	return true;
}

static void _net_nfc_test_mode_off_cb(void *pContext, NFCSTATUS status)
{
	controller_context_s *context = NULL;

	if (pContext != NULL)
	{
		context = (controller_context_s *)pContext;
		context->result = (int)status;
	}

	if (status == NFCSTATUS_SUCCESS)
	{
		DEBUG_MSG("test_mode_off is successful");
	}
	else
	{
		DEBUG_MSG("test_mode_off is failed = [0x%x]", status);
	}

	if (context != NULL)
	{
		NET_NFC_OEM_CONTROLLER_SIGNAL(context->controller_cond);
	}
}

static void _net_nfc_test_mode_on_cb(void *pContext, NFCSTATUS status)
{
	controller_context_s *context = NULL;

	if (pContext != NULL)
	{
		context = (controller_context_s *)pContext;
		context->result = (int)status;
	}

	if (status == NFCSTATUS_SUCCESS)
	{
		DEBUG_MSG("test_mode_on is successful");
	}
	else
	{
		DEBUG_MSG("test_mode_on is failed = [0x%x]", status);
	}

	if (context != NULL)
	{
		NET_NFC_OEM_CONTROLLER_SIGNAL(context->controller_cond);
	}
}

static bool net_nfc_nxp_controller_sim_test(net_nfc_error_e *result)
{
	phNfc_sData_t InParam;
	phNfc_sData_t OutParam;
	uint8_t resp[16];
	int user_context = 0;
	NFCSTATUS status = NFCSTATUS_SUCCESS;
	pthread_cond_t controller_cond = PTHREAD_COND_INITIALIZER;
	controller_context_s context = { &controller_cond, &user_context };
	int ret = NFCSTATUS_SUCCESS;

	net_nfc_nxp_controller_configure_discovery_stop();
	DEBUG_MSG("net_nfc_nxp_controller_configure_discovery_stop out");

	if (result == NULL)
	{
		return false;
	}
	DEBUG_MSG("net_nfc_nxp_controller_sim_test Enter");

	*result = NET_NFC_OK;

	gInputParam.buffer  = NULL;
	gInputParam.length  = 0x00;
	gOutputParam.buffer = resp;
	gOutputParam.length = 0x02;

	NET_NFC_OEM_CONTROLLER_LOCK;
	if ((status = phLibNfc_Mgt_IoCtl(psHwRef.p_board_driver, DEVMGMT_SWP_TEST/*PHLIBNFC_SWP_TEST*/, &gInputParam, &gOutputParam, _net_nfc_swp_test_cb, (void *)&context)) != NFCSTATUS_PENDING)
	{
		NET_NFC_OEM_CONTROLLER_UNLOCK;
		DEBUG_MSG("DEVMGMT_SWP_TEST OEM FAIL [%d]\n" , status);
		return false;
	}
	else
	{
		NET_NFC_OEM_CONTROLLER_WAIT(&controller_cond);
		NET_NFC_OEM_CONTROLLER_UNLOCK;
		DEBUG_MSG("DEVMGMT_SWP_TEST 1 PASS gOutputParam.buffer[0] = [%d]" , gOutputParam.buffer[0]);
		//ret = (gOutputParam.buffer[0] == 0) ? TRUE : FALSE;

	}
	DEBUG_MSG("DEVMGMT_SWP_TEST END");
	return ret;
}

static bool net_nfc_nxp_controller_prbs_test(net_nfc_error_e *result , uint32_t tech , uint32_t rate)
{
	phNfc_sData_t InParam;
	phNfc_sData_t OutParam;
	uint8_t resp[16];
	uint8_t PRBS_setting[2] = {0x01,0x01}; // PRBS setting
	int user_context = 0;
	NFCSTATUS status = NFCSTATUS_SUCCESS;
	pthread_cond_t controller_cond = PTHREAD_COND_INITIALIZER;
	controller_context_s context = { &controller_cond, &user_context };

	if (result == NULL)
	{
		return false;
	}

	DEBUG_MSG("net_nfc_nxp_controller_prbs_test Enter");

	*result = NET_NFC_OK;

	PRBS_setting[0] = tech;
	PRBS_setting[1] = rate;

	gInputParam.buffer  = PRBS_setting;
	gInputParam.length  = 0x02;
	gOutputParam.buffer = resp;

	DEBUG_MSG("PRBS_setting[0] = [%d]\n" , PRBS_setting[0]);
	DEBUG_MSG("PRBS_setting[1] = [%d]\n" , PRBS_setting[1]);

	NET_NFC_OEM_CONTROLLER_LOCK;
	if ((status = phLibNfc_Mgt_IoCtl(psHwRef.p_board_driver, DEVMGMT_PRBS_TEST/*PHLIBNFC_SWP_TEST*/, &gInputParam, &gOutputParam, _net_nfc_swp_test_cb, (void *)&context)) != NFCSTATUS_PENDING)
	{
		NET_NFC_OEM_CONTROLLER_UNLOCK;
		return false;
	}
	else
	{
		NET_NFC_OEM_CONTROLLER_WAIT(&controller_cond);
		NET_NFC_OEM_CONTROLLER_UNLOCK;
		DEBUG_MSG("DEVMGMT_PRBS_TEST 1 PASS gOutputParam.buffer[0] = [%d]" , gOutputParam.buffer[0]);

		if (user_context != NFCSTATUS_SUCCESS)
		{
			DEBUG_MSG("DEVMGMT_PRBS_TEST ERROR");
			return false;
		}
	}
	DEBUG_MSG("DEVMGMT_PRBS_TEST End");
	return true;
}


static void _net_nfc_swp_test_cb(void *pContext, phNfc_sData_t *Outparam_Cb, NFCSTATUS status)
{
	controller_context_s *context = NULL;

	if (pContext != NULL)
	{
		context = (controller_context_s *)pContext;
		*((int *)context->user_context) = status;
	}

	if (status == NFCSTATUS_SUCCESS)
	{
		DEBUG_MSG("Callback TEST SUCCESS");

	}
	else
	{
		DEBUG_MSG("Callback TEST FAIL [0x%x]\n", status);
	}

	if (context != NULL)
	{
		NET_NFC_OEM_CONTROLLER_SIGNAL(context->controller_cond);
	}
}

static bool net_nfc_nxp_controller_configure_discovery_stop()
{
	NFCSTATUS friReturn = NFCSTATUS_SUCCESS;
	pthread_cond_t controller_cond = PTHREAD_COND_INITIALIZER;
	controller_context_s context = { &controller_cond, NULL, 0 };
	phLibNfc_sADD_Cfg_t AddConfig;


	AddConfig.PollDevInfo.PollEnabled = 0;
	AddConfig.Duration = 300000; /* in ms */
	AddConfig.NfcIP_Mode = phNfc_eDefaultP2PMode;
	AddConfig.NfcIP_Tgt_Disable = TRUE;


	DEBUG_MSG("stop poll loop");

	NET_NFC_OEM_CONTROLLER_LOCK;

	//__net_nfc_make_invalid_target_handle();
	friReturn = phLibNfc_Mgt_ConfigureDiscovery(NFC_DISCOVERY_CONFIG, AddConfig, _net_nfc_configure_discovery_cb, (void *)&context);
	if (friReturn != NFCSTATUS_PENDING)
	{
		NET_NFC_OEM_CONTROLLER_UNLOCK;

		DEBUG_MSG("discovering config is error");
		return false;
	}
	else
	{
		NET_NFC_OEM_CONTROLLER_WAIT(&controller_cond);
		NET_NFC_OEM_CONTROLLER_UNLOCK;

		DEBUG_MSG("discovering config is end");

		if (context.result != NFCSTATUS_SUCCESS)
		{
			return false;
		}
		else
		{
			return true;
		}
	}
}


static bool net_nfc_nxp_controller_support_nfc(net_nfc_error_e *result)
{
	bool ret = false;
	struct stat st = { 0, };

	if (result == NULL)
	{
		return ret;
	}

	if (stat("/dev/pn544", &st) == 0)
	{
		*result = NET_NFC_OK;
		ret = true;
	}
	else
	{
		*result = NET_NFC_NOT_SUPPORTED;
	}

	return ret;
}

pthread_mutex_t *_nfc_get_fri_lock()
{
	return &g_controller_lock;
}

static bool __net_nfc_is_valid_target_handle(net_nfc_target_handle_s *handle)
{
	bool result = (current_working_handle == handle);
	if (!result)
	{
		DEBUG_MSG("[WARNING]: INVALID HANDLE IS DETECTED!");
	}
	return result;
}

static void __net_nfc_make_valid_target_handle(net_nfc_target_handle_s **handle)
{
	if (current_working_handle != NULL)
	{
		DEBUG_MSG("[WARNING]: HANDLE WAS ALLOCATED ALREADY!");
	}
	_net_nfc_util_alloc_mem(*handle, sizeof(net_nfc_target_handle_s));
	if (*handle != NULL)
	{
		current_working_handle = *handle;
	}
}

static void __net_nfc_make_invalid_target_handle()
{
	if (current_working_handle != NULL)
	{
		_net_nfc_util_free_mem(current_working_handle);
		current_working_handle = NULL;
	}
}

