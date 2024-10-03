# Copyright 2024 NXP
#
# SPDX-License-Identifier: BSD-3-Clause

if (CONFIG_MCUX_COMPONENT_middleware.wireless.ieee_802_15_4)
    mcux_add_include(
        INCLUDES
            Include
            ieee_802_15_4/phy/interface
            ieee_802_15_4/phy/source
            ieee_802_15_4/phy/source/SerialDevice
            ieee_802_15_4/mac/interface
            ieee_802_15_4/mac/source/App
            smac/interface
            smac/common
            smac/source
            utils
    )

    mcux_add_source(
        SOURCES
            ieee_802_15_4/phy/interface/*.h
            ieee_802_15_4/phy/source/SerialDevice/*.h
            ieee_802_15_4/mac/interface/*.h
            Include/MiniMac.h
            Include/MMAC.h
            Include/MiniMac_Pib.h
            Include/tsv_pub.h
            Include/mac_sap.h
            Include/mac_pib.h
            Include/AppApi.h
    )
endif()

if (CONFIG_MCUX_COMPONENT_middleware.wireless.ieee_802_15_4.k32w1_mcxw71.split_PHY)
    mcux_add_source(
        SOURCES
            ieee_802_15_4/phy/source/SerialDevice/Phy.c
            ieee_802_15_4/phy/source/SerialDevice/ASP.c
    )
endif()

if (CONFIG_MCUX_COMPONENT_middleware.wireless.ieee_802_15_4.mcxw72.split_PHY)
    mcux_add_source(
        SOURCES
            ieee_802_15_4/phy/source/SerialDevice/Phy.c
            ieee_802_15_4/phy/source/SerialDevice/ASP.c
    )
endif()

if (CONFIG_MCUX_COMPONENT_middleware.wireless.ieee_802_15_4.k32w1_mcxw71.split_MAC)
    mcux_add_library(
        LIBS
            ieee_802_15_4/lib/libieee-802.15.4_MAC_intf.a
    )
endif()

if (CONFIG_MCUX_COMPONENT_middleware.wireless.ieee_802_15_4.k32w1_mcxw71.MAC_split_PHY)
    mcux_add_library(
        LIBS
            ieee_802_15_4/lib/libieee-802.15.4_split.a
    )
endif()

if (CONFIG_MCUX_COMPONENT_middleware.wireless.ieee_802_15_4.mcxw72.split_MAC)
    mcux_add_library(
        LIBS
            ieee_802_15_4/lib/libieee-802.15.4_MAC_intf_MCXW72.a
    )
endif()

if (CONFIG_MCUX_COMPONENT_middleware.wireless.ieee_802_15_4.mcxw72.MAC_split_PHY)
    mcux_add_library(
        LIBS
            ieee_802_15_4/lib/libieee-802.15.4_split_MCXW72.a
    )
endif()

if (CONFIG_MCUX_COMPONENT_middleware.wireless.ieee_802_15_4.rw61x.split_MAC)
    mcux_add_library(
        LIBS
            ieee_802_15_4/lib/libieee-802.15.4_MAC_intf_RW612.a
    )
endif()
