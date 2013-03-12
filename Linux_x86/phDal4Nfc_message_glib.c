/*
 * {nfc-plugin-nxp}
 *
 * Copyright (c) 2012, 2013 Samsung Electronics Co., Ltd.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 */

#include <phDal4Nfc.h>
#include <phOsalNfc.h>
#include <phOsalNfc_Timer.h>
#include <phDal4Nfc_DeferredCall.h>
#include <phDal4Nfc_messageQueueLib.h>

#include <glib.h>
#include <stdlib.h>
#include <pthread.h>

static int g_msg_q_id = -1;
pthread_mutex_t * _nfc_get_fri_lock ();

typedef struct phOsalNfc_Message_Wrapper{
	long mtype;
	phOsalNfc_Message_t msg;
}phOsalNfc_Message_Wrapper_t;

gboolean DeferredCallback(gpointer data)
{
	phOsalNfc_Message_Wrapper_t wrapper;
	phOsalNfc_Message_t OsalMsg;

	phDal4Nfc_msgrcv(g_msg_q_id, (void *)&wrapper, sizeof(phOsalNfc_Message_t), 0, 0);

	memcpy(&OsalMsg, &wrapper.msg, sizeof(phOsalNfc_Message_t));

	if(OsalMsg.eMsgType == PH_DAL4NFC_MESSAGE_BASE)
	{
		phDal4Nfc_DeferredCall_Msg_t* defer_msg = NULL;
		defer_msg = (phDal4Nfc_DeferredCall_Msg_t *)(OsalMsg.pMsgData);

		if((defer_msg != NULL) && (defer_msg->pCallback != NULL))
		{
			pthread_mutex_lock (_nfc_get_fri_lock());
			defer_msg->pCallback(defer_msg->pParameter);
			pthread_mutex_unlock (_nfc_get_fri_lock());
	    }
	}

	#ifdef NXP_MESSAGING

	else if(OsalMsg.eMsgType == PH_OSALNFC_TIMER_MSG)
	{
		phOsalNfc_Timer_Msg_t *timer_msg = NULL;
		timer_msg = (phOsalNfc_Timer_Msg_t *)(OsalMsg.pMsgData);

		if (timer_msg != NULL && timer_msg->pCallBck !=NULL)
		{
			pthread_mutex_lock (_nfc_get_fri_lock());
			timer_msg->pCallBck (timer_msg->TimerId, timer_msg->pContext);
			pthread_mutex_unlock (_nfc_get_fri_lock());
		}
	}

	#endif

	return FALSE;
}

int InitMessageQueue()
{
	if(g_msg_q_id < 0)
	{
		g_msg_q_id = phDal4Nfc_msgget(IPC_PRIVATE, 0666 | IPC_CREAT);
	}

	return g_msg_q_id;
}


void DeleteMessageQeueu()
{
	phDal4Nfc_msgctl(g_msg_q_id, IPC_RMID, NULL);
}



void PostMessage(void* msgp, size_t msgsz, int msgflg)
{
	phDal4Nfc_msgsnd(g_msg_q_id, msgp, msgsz, msgflg);
/*
	if(g_idle_add(DeferredCallback, NULL))
	{
		g_main_context_wakeup(g_main_context_default()) ;
	}
*/


	if(g_idle_add_full(G_PRIORITY_HIGH_IDLE, DeferredCallback, NULL, NULL))
	{
		g_main_context_wakeup(g_main_context_default()) ;
	}

}


