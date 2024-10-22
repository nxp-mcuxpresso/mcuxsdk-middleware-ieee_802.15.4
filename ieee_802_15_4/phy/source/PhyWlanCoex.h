/*! *********************************************************************************
* Copyright 2021, 2023-2024 NXP
* All rights reserved.
*
* \file
*
* SPDX-License-Identifier: BSD-3-Clause
********************************************************************************** */

#ifndef __PHY_WLAN_COEX_H__
#define __PHY_WLAN_COEX_H__

/************************************************************************************
*************************************************************************************
* Include
*************************************************************************************
************************************************************************************/

#include "EmbeddedTypes.h"

/************************************************************************************
*************************************************************************************
* Public type definitions
*************************************************************************************
************************************************************************************/

/*
 * This flag can be set to 0 in platform file if the following types are already
 * defined somewhere else and don't need to be defined in this file
 */
#ifndef WLAN_COEX_TYPE_DEFINITION
#define WLAN_COEX_TYPE_DEFINITION 1
#endif

#if (WLAN_COEX_TYPE_DEFINITION == 1)
typedef struct coex_WlanToBt_t_
{
    uint8_t  bLoaded;           // WLAN loaded?
    uint8_t  bConnected;        // WLAN STA assocated or uAP enabled?
    uint8_t  bDeepSleep;        // WLAN in deep sleep?
    uint8_t  bTimeActive;       //???
    /* We are keeping bmWLChanUsed to make sure that the existing functionality
    is kept intact untill we move to the newer logic completely */
    uint32_t bmWLChanUsed;          // Bit masks to indicate WLAN channel used
    uint8_t  abmBTChanUsable[10];   // Bit masks to indicate BT channel usable
    uint8_t  bAntennaInfo;      // 0,1-->indicate WLAN antenna used, if WLAN use ant-0, BT can use ant-1
    uint8_t  bCoexMgrTmsMode;   // 1: TMS mode, 0: Spatial mode or None.
} coex_WlanToBt_t;

typedef struct coex_BtToWlan_t_
{
    uint8_t  ACL;               // Indicates if there is ACL link
    uint8_t  SCOeSCO;           // If there is SCO or eSCO link
    uint8_t  Scan;              // BT in page scan and/or inquiry scan?
    uint8_t  PagingInquiry;     // BT doing page or inquiry?
    uint8_t  Sniff;             // All ACL links in sniff mode?
    uint8_t  bSniffHighActivity;// one or more sniff link with tSniff < 20ms
    uint8_t  ucScoInterval;     // SCO interval in number of slots; Tsco
    uint8_t  ucScoDuration;     // Duration of an SCO instance; either 2 or 6 slots
    uint8_t  ucAvailableBuffers; // BT buffer number
    uint8_t  bBufferFull;       // BT TX buffer full/exceeding threshold?
    uint8_t  bFlowOnForAllAcl;  // All ACL L2CAP links flow on?
    uint8_t  bA2dpStreaming;    // A2DP data/media is streaming?
    uint8_t  bA2dpSignaling;    // A2DP link is doing signaling?
    uint8_t  bNeedMoreTime;     // BT needs more time?
    uint8_t  ucHidLinks;        // number of HID links
    uint8_t  bPairingInProcess; // BT Pairing is happening on any link
    uint32_t ulAclTxBytesLastSec;  //bytes sent on all ACL links in last second
    uint32_t ulAclRxBytesLastSec;  //bytes received by all ACL links in last second
    uint8_t  bAligning;         // BT doing coex aligning?
    uint8_t  ucWifiArbTime;     // Coex alignment WIFI/ARB time in ms
    uint8_t  ucAllLinks;        // number of All links
    uint8_t  btLoaded;          // BT loaded
    uint8_t  bBleActive;        // Any BLE activity?
    uint8_t  bBleHighActivity;  // High Activity BLE link(s)
    uint16_t uiBleInterval;     // Smallest BLE connection interval
    uint8_t  bA2dpSink;         // Indicates if it is A2DP SNK or SRC 
    uint8_t  bA2dpSinkMaster;   // If the A2DP SNK is BT Master or Slave
    uint8_t  bInquiring;        // BT doing inquiring
    uint8_t  bPaging;           // BT doing page
    uint8_t  bLEContinueScan;
} coex_BtToWlan_t;

typedef struct coex_OTToWlan_t_
{
    uint8_t OTChanUsed;
} coex_OTToWlan_t;

typedef struct {
    uint32_t gIcc_sync_lock;
    coex_WlanToBt_t wlanToBt;
    coex_BtToWlan_t btToWlan;
    coex_OTToWlan_t otToWlan;
} cpu12_shared_var_t;

typedef enum coex_MsgSubtype_t_
{
    coexMsgWl2OTChannelInfo_c = 0x10,
} coex_MsgSubtype_t;
#endif

/*! *********************************************************************************
*************************************************************************************
* Public prototypes
*************************************************************************************
********************************************************************************** */

void PhyWlanCoex_Init(void);
void PhyWlanCoex_ChannelUpdate(uint8_t channel);


#endif /* __PHY_WLAN_COEX_H__ */
