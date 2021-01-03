/**
 ******************************************************************************
 * @file SHDN_FSM_States.h
 * @brief SHDN FSM States
 ******************************************************************************
 */

#ifndef INC_SHDN_FSM_STATES_H_
#define INC_SHDN_FSM_STATES_H_

#include "FSM.h"
#include "main.h"
#include "SHDN_CAN_Messages.h"
#include <stdbool.h>


/**
 * @brief SHDN Global State
 * @note SHDN Global State is shared across threads, so use the semaphore to gain control
 */
typedef struct
{
	//CAN
	uint32_t CAN2_TxMailbox;
	uint32_t CAN2_RxMailbox;

	uint8_t segmentStates;

	volatile bool shutdownTriggered;

	osMessageQueueId_t CANQueue;
	osTimerId_t heartbeatTimer;
	osSemaphoreId_t sem;
} SHDN_GlobalState_t;

SHDN_GlobalState_t *SHDN_GlobalState;

/**
 * @brief Dead state enter function
 * @note No implementation as the dead state serves no purpose other than being an initial FSM state
 * @param fsm A pointer to the FSM object
 */
void state_dead_enter(fsm_t *fsm);

/**
 * @brief Dead state iterate function
 * @note No implementation as the dead state serves no purpose other than being an initial FSM state
 * @param fsm A pointer to the FSM object
 */
void state_dead_iterate(fsm_t *fsm);

/**
 * @brief Dead state exit function
 * @note No implementation as the dead state serves no purpose other than being an initial FSM state
 * @param fsm A pointer to the FSM object
 */
void state_dead_exit(fsm_t *fsm);

/**
 * @brief deadState ie. startup state for the fsm
 * @note Initial FSM state, has no functionality
 * @details Next: runState (Instantly)
 */
state_t deadState;

/**
 * @brief Run state enter function. Initialises the SHDN_GlobalState, starts sending hearbeats
 * @param fsm A pointer to the FSM object
 */
void state_run_enter(fsm_t *fsm);

/**
 * @brief Run state iterate function. Checks for breaks in shudown line and stops sending heartbeats
 * @param fsm A pointer to the FSM object
 */
void state_run_iterate(fsm_t *fsm);

/**
 * @brief Run state exit function. Should not be reached
 * @param fsm A pointer to the FSM object
 */
void state_run_exit(fsm_t *fsm);

/**
 * @brief runState i.e. Main state for Shudown board.
 * @note
 * @details Next: none
 */
state_t runState;


#endif /* INC_SHDN_FSM_STATES_H_ */
