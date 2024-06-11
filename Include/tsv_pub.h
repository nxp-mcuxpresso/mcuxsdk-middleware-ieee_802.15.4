/*
 * Copyright 2012-2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/****************************************************************************
 *
 * MODULE:             TSV
 *
 * DESCRIPTION:
 * Timer Server
 * Simple linked list timer server which cannot be used in multi-tasking
 * environment
 ***************************************************************************/

#ifndef TSV_PUB_H_INCLUDED
#define TSV_PUB_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup grp_TSV Timer Server
 */

/** @mainpage TSV : Timer Server module
 *
 * @section page_TSV_Intro Introduction
 * The Timer Server Module is used to implement a linked list of Timers which can
 * all be tied to a basic timing service, which will usually be implemented in
 * a hardware timer. Alternatively, it can be ticked regularly.
 * The Event Timer environment must be initialised by calling TSV_Init().
 * An instance of a Timer is created by the Client, and passed to the
 * TSV_Open() function.
 * The Client will then start and stop this Timer, and receive expiry
 * notification by callback with parameter.
 * The Timer can then be closed by calling TSV_Close() and can then be
 * deleted by the Client.
 */

/***********************/
/**** INCLUDE FILES ****/
/***********************/

#include "jendefs.h"

/**
 * @addtogroup grp_TSV Timer Server
 * Methods and attributes associated with Timer Server
 * @{
 */

/************************/
/**** MACROS/DEFINES ****/
/************************/

/**************************/
/**** TYPE DEFINITIONS ****/
/**************************/

/**
 * Timer Result Code.
 */
typedef enum
{
    /* Information */
    TSV_I_OK = 0x0000,
    /* Warning */
    TSV_W_WARNING = 0x0100,
    TSV_W_ZERO_VALUE,
    /* Error */
    TSV_E_ERROR = 0x0200,
    TSV_E_NOT_REMOVED,
    TSV_E_NOT_RUNNING,
    TSV_E_ALREADY_RUNNING,
    TSV_E_INVALID_PARAM,
    NUM_TSV_RESULT_CODE
} TSV_ResultCode_e;

#define TSV_IS_INFORMATION_RC(x)     (((x) >= TSV_I_OK) && ((x) < TSV_W_WARNING))
#define TSV_IS_WARNING_RC(x)         (((x) >= TSV_W_WARNING) && ((x) < TSV_E_ERROR))
#define TSV_IS_ERROR_RC(x)           (((x) >= TSV_E_ERROR) && ((x) < NUM_TSV_RESULT_CODE))

/**** Elemental types ****/

/**
 * Timer Expire Callback function.
 * Callback function called when Timer expires
 */
typedef void (*TSV_ExpireCb_pr)(void *pvParam);

/**
 * Timer Set Callback function.
 * Callback function called when external timer needs to be set
 */
typedef void (*TSV_SetCb_pr)(void *pvParam, uint32 u32Value);

/**
 * Timer Get Callback function.
 * Callback function called when external timer needs to be read
 */
typedef uint32 (*TSV_GetCb_pr)(void *pvParam);

/**** Objects ****/

/**
 * Event Timer
 * @note strictly opaque
 */
typedef struct tagTSV_Timer_s
{
    struct tagTSV_Timer_s  *psTimerNext;    /**< Linked list pointer */
    uint32                 u32Value;        /**< The current value of the timer */
    /* Static members - remain for duration */
    TSV_ExpireCb_pr prExpireCb;      /**< @sa TSV_TimerPOpen_s */
    void            *pvParam;        /**< @sa TSV_TimerPOpen_s */
} TSV_Timer_s;

/**
 * Event Timer List
 * @note strictly opaque
 */
typedef struct tagTSV_TimerList_s
{
    TSV_Timer_s  *psList;       /**< List pointer to running Timers */
    TSV_SetCb_pr prSetCb;       /**< Callback called when ext. Timer needs to be set */
    TSV_GetCb_pr prGetCb;       /**< Callback called when ext. Timer needs to be got */
    void         *pvParam;      /**< Parameter for Callback */
} TSV_TimerList_s;

/****************************/
/**** EXPORTED VARIABLES ****/
/****************************/

/* @} */

/****************************/
/**** EXPORTED FUNCTIONS ****/
/****************************/

/* Initialise */
PUBLIC TSV_ResultCode_e
TSV_eInit(TSV_SetCb_pr prSetCb,
          TSV_GetCb_pr prGetCb,
          void *pvParam);

#ifdef TSV_DYNAMIC_POOL
/* Construct and destruct methods */
PUBLIC TSV_Timer_s*
TSV_psConstruct(TSV_Timer_s* psTimer);

PUBLIC void
TSV_vDestruct(TSV_Timer_s* psTimer);
#endif /* TSV_DYNAMIC_POOL */

/* Open and close methods */
PUBLIC TSV_ResultCode_e
TSV_eOpen(TSV_Timer_s* psTimer,
          TSV_ExpireCb_pr prExpireCb,
          void *pvParam);

PUBLIC TSV_ResultCode_e
TSV_eClose(TSV_Timer_s* psTimer,
           bool_t bInvokeCBIfRunning);

/* Start/stop */
PUBLIC TSV_ResultCode_e
TSV_eStart(TSV_Timer_s* psTimer,
           uint32 u32Value);

PUBLIC TSV_ResultCode_e
TSV_eStop(TSV_Timer_s* psTimer);

/* Tick procedure */
PUBLIC TSV_ResultCode_e
TSV_eTickProc(int iNumTicks);

/* Expiry procedure */
PUBLIC TSV_ResultCode_e
TSV_eExpireProc(void);

/* Driver functions */
PUBLIC TSV_ResultCode_e TSV_eInitBBT0(void);
PUBLIC TSV_ResultCode_e TSV_eInitBBT1(void);
PUBLIC TSV_ResultCode_e TSV_eInitNoMAC(void);

PUBLIC void TSV_vInterruptHandler(void);
#ifdef __cplusplus
};
#endif

#endif /* TSV_PUB_H_INCLUDED */

/* End of file **************************************************************/
