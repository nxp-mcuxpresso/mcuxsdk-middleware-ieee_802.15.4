/*
 * Copyright 2012-2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*****************************************************************************
 *
 * MODULE:             Application API header
 *
 * DESCRIPTION:        Select correct interface depending on chip / chip family
 *
 ***************************************************************************/

/**
 * @defgroup g_app_sap Application MAC Service Access Point (SAP)
 */
#ifndef  APP_API_H_INCLUDED
#define  APP_API_H_INCLUDED

#if defined __cplusplus
extern "C" {
#endif

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include <jendefs.h>
#include <mac_sap.h>
//#include <mac_pib.h>

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/**
 * @ingroup g_app_sap
 * @brief Get Buffer routine type
 *
 * Type of Get Buffer callback routine
 */
typedef MAC_DcfmIndHdr_s * (*PR_GET_BUFFER)(void *pvParam);

/**
 * @ingroup g_app_sap
 * @brief Post routine type
 *
 * Type of Post callback routine
 */
typedef void (*PR_POST_CALLBACK)(void *pvParam, MAC_DcfmIndHdr_s *psDcfmInd);

typedef enum
{
    E_APPAPI_RADIO_TX_MODE_STD    = 0,
    E_APPAPI_RADIO_TX_MODE_PROP_1 = 1,
    E_APPAPI_RADIO_TX_MODE_PROP_2 = 2,
    E_APPAPI_RADIO_TX_MODE_RESET  = 0xff
} teAppApiRadioTxMode;

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
PUBLIC uint32
u32AppApiInit(PR_GET_BUFFER prMlmeGetBuffer,
              PR_POST_CALLBACK prMlmeCallback,
              void *pvMlmeParam,
              PR_GET_BUFFER prMcpsGetBuffer,
              PR_POST_CALLBACK prMcpsCallback,
              void *pvMcpsParam);

PUBLIC void *pvAppApiGetMacHandle(void);

/* MLME calls */
PUBLIC void
vAppApiMlmeRequest(MAC_MlmeReqRsp_s *psMlmeReqRsp,
                   MAC_MlmeSyncCfm_s *psMlmeSyncCfm);

PUBLIC void
vAppApiMcpsRequest(MAC_McpsReqRsp_s *psMcpsReqRsp,
                   MAC_McpsSyncCfm_s *psMcpsSyncCfm);

/* PLME calls */
PUBLIC PHY_Enum_e eAppApiPlmeSet(PHY_PibAttr_e ePhyPibAttribute,
                                 uint32 u32PhyPibValue);
PUBLIC PHY_Enum_e eAppApiPlmeGet(PHY_PibAttr_e ePhyPibAttribute,
                                 uint32 *pu32PhyPibValue);

PUBLIC void *pvAppApiGetMacAddrLocation(void);
PUBLIC void vAppApiSetMacAddrLocation(void *pvNewMacAddrLocation);
PUBLIC void  vAppApiSaveMacSettings(void);
PUBLIC void  vAppApiRestoreMacSettings(void);

PUBLIC void vAppApiEnableBeaconResponse(bool_t bEnable);

PUBLIC void vAppApiSetSecurityMode(MAC_SecutityMode_e eSecurityMode);

PUBLIC void vMAC_RestoreSettings(void);
PUBLIC bool_t bAppApi_CurrentlyScanning(void);

PUBLIC void vAppApiSetComplianceLimits(int8  i8TxMaxPower,
                                       int8  i8TxMaxPowerCh26,
                                       uint8 u8CcaThreshold);
PUBLIC void vAppApiSetRadioTxModes(teAppApiRadioTxMode eTxMode,
                                   teAppApiRadioTxMode eTxModeCh26);
PUBLIC void vAppApiSetHighPowerMode(uint8 u8ModuleID, bool_t bMode);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#if defined __cplusplus
}
#endif

#endif  /* APP_API_H_INCLUDED */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/

