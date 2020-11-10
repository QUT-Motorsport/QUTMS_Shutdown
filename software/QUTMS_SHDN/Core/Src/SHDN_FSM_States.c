/**
 ******************************************************************************
 * @file AMS_FSM_States.c
 * @brief AMS FSM States
 ******************************************************************************
 */

#include <SHDN_FSM_States.h>
#include "main.h"

state_t deadState = {&state_dead_enter, &state_dead_iterate, &state_dead_exit, "Dead_s"};

void state_dead_enter(fsm_t *fsm)
{
	return;
}

void state_dead_iterate(fsm_t *fsm)
{
	Error_Handler();
	return;
}

void state_dead_exit(fsm_t *fsm)
{
	return;
}

state_t runState = {&state_run_enter, &state_run_iterate, &state_run_exit, "Run_s"};

void state_run_enter(fsm_t *fsm)
{
	if(SHDN_GlobalState == NULL)
	{
		SHDN_GlobalState = malloc(sizeof(SHDN_GlobalState_t));
		memset(SHDN_GlobalState, 0, sizeof(SHDN_GlobalState_t));

		// As SHDN_GlobalState is accessible across threads, we need to use a semaphore to access it
		SHDN_GlobalState->sem = osSemaphoreNew(1U, 1U, NULL);
		if(osSemaphoreAcquire(SHDN_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
		{
			SHDN_GlobalState->segmentStates = 1U;
			SHDN_GlobalState->chainOut = true;

			SHDN_GlobalState->heartbeatTimer = osTimerNew(&heartbeatTimer_cb, osTimerPeriodic, fsm, NULL);
			if(osTimerStart(SHDN_GlobalState->heartbeatTimer, SHDN_HEARTBEAT_PERIOD) != osOK)
			{
				Error_Handler();
			}

			SHDN_GlobalState->CANQueue = osMessageQueueNew(SHDN_CAN_QUEUESIZE, sizeof(CAN_MSG_Generic_t), NULL);
			if(SHDN_GlobalState->CANQueue == NULL)
			{
				Error_Handler();
			}
			osSemaphoreRelease(SHDN_GlobalState->sem);
		}
	}
}

void state_run_iterate(fsm_t *fsm)
{
	if(osSemaphoreAcquire(SHDN_GlobalState->sem, SEM_ACQUIRE_TIMEOUT) == osOK)
	{
		HAL_GPIO_WritePin(LEDA_GPIO_Port, LEDA_Pin, SHDN_GlobalState->chainOut);
		HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, !SHDN_GlobalState->chainOut);

		if(!SHDN_GlobalState->chainOut) {
			SHDN_GlobalState->segmentStates = 255U;
		} else {
			SHDN_GlobalState->segmentStates = 0;
		}

		SHDN_GlobalState->chainOut = HAL_GPIO_ReadPin(CHAIN_OUT_GPIO_Port, CHAIN_OUT_Pin);
		osSemaphoreRelease(SHDN_GlobalState->sem);

	}
}

void state_run_exit(fsm_t *fsm)
{
	return;
}
