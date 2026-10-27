/*-----------------------------------------------------------------------------
 * selectLinkLayer.h
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Paul Bussmann
 * Description              EC-Master link layer selection
 *---------------------------------------------------------------------------*/

#ifndef INC_SELECTLINKAYER
#define INC_SELECTLINKAYER 1

/*-INCLUDES------------------------------------------------------------------*/


#ifndef INC_ECOS
#include "EcOs.h"
#endif
#include "stdio.h"
#include "stdlib.h"
#ifndef INC_ECLINK
#include "EcLink.h"
#endif

#if (defined INCLUDE_DUMMY)
#include "EcLinkDummy.h"
#endif

/*-DEFINES-------------------------------------------------------------------*/
#if (!defined EXCLUDE_EMLL_ALL)

#if (defined EC_VERSION_CMSIS_RTOS)
 #define INCLUDE_EMLL_STATIC_LIBRARY
 #define INCLUDE_EMLLSTM32ETH
#elif (defined EC_VERSION_ECOS)
 #define INCLUDE_EMLL_STATIC_LIBRARY
 #ifndef EXCLUDE_EMLLANTAIOS
 #define INCLUDE_EMLLANTAIOS
 #endif
#elif (defined EC_VERSION_FREERTOS)
 #define INCLUDE_EMLLTIENETCPSWG
 #define INCLUDE_EMLLTIENETICSSG
 #define INCLUDE_EMLL_SOC_NXP
 #define INCLUDE_EMLL_SOC_XILINX
#elif (defined EC_VERSION_INTEGRITY)
 #define INCLUDE_EMLL_STATIC_LIBRARY
 #define INCLUDE_EMLLI8254X
 #define INCLUDE_EMLLINTELGBE
#elif (defined EC_VERSION_INTIME)
 #define INCLUDE_EMLL_PCI_ALL
 #ifndef EXCLUDE_EMLLSIMULATOR
 #define INCLUDE_EMLLSIMULATOR
 #endif
 #ifndef EXCLUDE_EMLLBCMNETXTREME
 #define INCLUDE_EMLLBCMNETXTREME
 #endif
#elif (defined EC_VERSION_LINUX)
 #define INCLUDE_EMLL_PCI_ALL
 #define INCLUDE_EMLL_SOC_ALL
#ifndef EXCLUDE_EMLLMULTIPLIER
#define INCLUDE_EMLLMULTIPLIER
#endif
 #ifndef EXCLUDE_EMLLLAN743X
 #define INCLUDE_EMLLLAN743X
 #endif
 #ifndef EXCLUDE_EMLLDW3504
 #define INCLUDE_EMLLDW3504
 #endif
 #ifndef EXCLUDE_EMLLALTERATSE
 #define INCLUDE_EMLLALTERATSE
 #endif
 #ifndef EXCLUDE_EMLLREMOTE
 #define INCLUDE_EMLLREMOTE
 #endif
 #ifndef EXCLUDE_EMLLSIMULATOR
 #define INCLUDE_EMLLSIMULATOR
 #endif
 #ifndef EXCLUDE_EMLLSOCKRAW
 #define INCLUDE_EMLLSOCKRAW
 #endif
 #ifndef EXCLUDE_EMLLDPDK
 #define INCLUDE_EMLLDPDK
 #endif
 #ifndef EXCLUDE_EMLLBCMNETXTREME
 #define INCLUDE_EMLLBCMNETXTREME
 #endif
  #ifndef EXCLUDE_EMLLVLAN
  #define INCLUDE_EMLLVLAN
  #endif
#elif (defined EC_VERSION_QNX)
 #define INCLUDE_EMLL_PCI_ALL
 #define INCLUDE_EMLL_SOC_ALL
 #ifndef EXCLUDE_EMLLSIMULATOR
 #define INCLUDE_EMLLSIMULATOR
 #endif
 #ifndef EXCLUDE_EMLLBCMNETXTREME
 #define INCLUDE_EMLLBCMNETXTREME
 #endif
 #ifndef EXCLUDE_EMLLBPF
 #define INCLUDE_EMLLBPF
 #endif
#elif (defined EC_VERSION_SYLIXOS)
 #ifndef EXCLUDE_EMLLDW3504
  #define INCLUDE_EMLLDW3504
 #endif
#elif (defined EC_VERSION_RIN32)
 #define INCLUDE_EMLLRIN32
#elif (defined EC_VERSION_RTOS32)
 #if !(EC_DLL)
  #define INCLUDE_EMLL_STATIC_LIBRARY
 #endif
 #define INCLUDE_EMLL_PCI_ALL
 #ifndef EXCLUDE_EMLLMULTIPLIER
 #define INCLUDE_EMLLMULTIPLIER
 #endif
#elif (defined EC_VERSION_RTX)
 #define INCLUDE_EMLL_PCI_ALL
 #ifndef EXCLUDE_EMLLBCMNETXTREME
 #define INCLUDE_EMLLBCMNETXTREME
 #endif
#elif (defined EC_VERSION_TIRTOS)
 #define INCLUDE_EMLL_STATIC_LIBRARY
 /* #define INCLUDE_EMLL_SOC_TI */ /* currently set in specific demo project file */
#elif (defined EC_VERSION_UC3)
 #define INCLUDE_EMLL_STATIC_LIBRARY
 #if (defined SOC_RZT1)
 #define INCLUDE_EMLLRZT1
 #else
 #define INCLUDE_EMLL_SOC_SYNOPSYS
 #endif
#elif (defined EC_VERSION_UCOS)
 #define INCLUDE_EMLL_STATIC_LIBRARY
 #define INCLUDE_EMLL_SOC_NXP
#elif (defined EC_VERSION_VXWORKS)
 #define INCLUDE_EMLL_PCI_ALL
 #define INCLUDE_EMLL_SOC_ALL
 #define INCLUDE_EMLLSNARF
 #if ((EC_VERSION_VXWORKS == 690) || (EC_VERSION_VXWORKS == 700))
  #ifndef EXCLUDE_EMLLSIMULATOR
  #define INCLUDE_EMLLSIMULATOR
  #endif
  #ifndef EXCLUDE_EMLLMULTIPLIER
  #define INCLUDE_EMLLMULTIPLIER
  #endif
 #endif
#elif (defined EC_VERSION_WINCE)
 #define INCLUDE_EMLL_PCI_ALL
  #ifndef EXCLUDE_EMLLR6040
 #define INCLUDE_EMLLR6040
 #endif
#elif (defined EC_VERSION_WINDOWS)
 #define INCLUDE_EMLL_PCI_ALL
 #ifndef EXCLUDE_EMLLNDIS
 #define INCLUDE_EMLLNDIS
 #endif
 #ifndef EXCLUDE_EMLLREMOTE
 #define INCLUDE_EMLLREMOTE
 #endif
 #ifndef EXCLUDE_EMLLSIMULATOR
 #define INCLUDE_EMLLSIMULATOR
 #endif
 #if (defined INCLUDE_EC_SIMULATOR_HIL)
  #ifndef EXCLUDE_EMLLTAP
  #define INCLUDE_EMLLTAP
  #endif
 #endif
 #ifndef EXCLUDE_EMLLWINPCAP
 #define INCLUDE_EMLLWINPCAP
 #endif
 #ifndef EXCLUDE_EMLLMULTIPLIER
 #define INCLUDE_EMLLMULTIPLIER
 #endif
 #ifndef EXCLUDE_EMLLBCMNETXTREME
 #define INCLUDE_EMLLBCMNETXTREME
 #endif
 #ifndef EXCLUDE_EMLLVLAN
 #define INCLUDE_EMLLVLAN
 #endif
#elif (defined EC_VERSION_XENOMAI)
 #define INCLUDE_EMLL_PCI_ALL
 #define INCLUDE_EMLL_SOC_ALL
 #ifndef EXCLUDE_EMLLSOCKRAW
 #define INCLUDE_EMLLSOCKRAW
 #endif
 #ifndef EXCLUDE_EMLLSIMULATOR
 #define INCLUDE_EMLLSIMULATOR
 #endif
 #ifndef EXCLUDE_EMLLMULTIPLIER
 #define INCLUDE_EMLLMULTIPLIER
 #endif
#elif (defined EC_VERSION_XILINX_STANDALONE)
 #define INCLUDE_EMLL_SOC_XILINX
#elif (defined EC_VERSION_ZEPHYR)
 #define INCLUDE_EMLL_STATIC_LIBRARY
 #define INCLUDE_EMLLI8254X
 #define INCLUDE_EMLLINTELGBE
 #define INCLUDE_EMLLRTL8169
#endif

#if (defined INCLUDE_EMLL_PCI_ALL)
 #define INCLUDE_EMLL_PCI_BECKHOFF
 #define INCLUDE_EMLL_PCI_INTEL
 #define INCLUDE_EMLL_PCI_REALTEK
 
#endif
#if (defined INCLUDE_EMLL_PCI_BECKHOFF)
 #ifndef EXCLUDE_EMLLCCAT
 #define INCLUDE_EMLLCCAT
 #endif
#endif
#if (defined INCLUDE_EMLL_PCI_INTEL)
 #ifndef EXCLUDE_EMLLEG20T
 #define INCLUDE_EMLLEG20T
 #endif
 #ifndef EXCLUDE_EMLLI8254X
 #define INCLUDE_EMLLI8254X
 #endif
 #ifndef EXCLUDE_EMLLINTELGBE
 #define INCLUDE_EMLLINTELGBE
 #endif
 #ifndef EXCLUDE_EMLLI8255X
 #define INCLUDE_EMLLI8255X
 #endif
#endif /* INCLUDE_EMLL_PCI_INTEL */

#if (defined INCLUDE_EMLL_PCI_REALTEK)
 #ifndef EXCLUDE_EMLLRTL8169
 #define INCLUDE_EMLLRTL8169
 #endif
 #ifndef EXCLUDE_EMLLRTL8139
 #define INCLUDE_EMLLRTL8139
 #endif
#endif

#if (defined INCLUDE_EMLL_SOC_ALL)
 #define INCLUDE_EMLL_SOC_NXP
 #define INCLUDE_EMLL_SOC_SYNOPSYS
 #define INCLUDE_EMLL_SOC_TI
 #define INCLUDE_EMLL_SOC_XILINX
 #define INCLUDE_EMLL_SOC_BCMGENET
 #define INCLUDE_EMLL_SOC_RENESAS
#endif

#if (EC_ARCH == EC_ARCH_ARM)
 #if (defined INCLUDE_EMLL_SOC_NXP)
  #ifndef EXCLUDE_EMLLFSLFEC
  #define INCLUDE_EMLLFSLFEC
  #endif
 #endif
 #if (defined INCLUDE_EMLL_SOC_SYNOPSYS)
  #ifndef EXCLUDE_EMLLDW3504
  #define INCLUDE_EMLLDW3504
  #endif
 #endif
 #if (defined INCLUDE_EMLL_SOC_RENESAS)
  #ifndef EXCLUDE_EMLLSHETH
  #define INCLUDE_EMLLSHETH
  #endif
 #endif
 #if (defined INCLUDE_EMLL_SOC_TI)
   #ifndef EXCLUDE_EMLLCPSWG
    #define INCLUDE_EMLLCPSWG
   #endif
   #ifndef EXCLUDE_EMLLCPSW
    #define INCLUDE_EMLLCPSW
   #endif
   #ifndef EXCLUDE_EMLLICSS
    #define INCLUDE_EMLLICSS
 #endif
 #endif
 #if (defined INCLUDE_EMLL_SOC_XILINX)
  #ifndef EXCLUDE_EMLLEMAC
  #define INCLUDE_EMLLEMAC
  #endif
  #ifndef EXCLUDE_EMLLGEM
  #define INCLUDE_EMLLGEM
  #endif
 #endif
#endif

#if (EC_ARCH == EC_ARCH_ARM64)
 #if (defined INCLUDE_EMLL_SOC_NXP)
  #ifndef EXCLUDE_EMLLFSLFEC
  #define INCLUDE_EMLLFSLFEC
  #endif
 #endif
 #if (defined INCLUDE_EMLL_SOC_XILINX)
  #ifndef EXCLUDE_EMLLGEM
  #define INCLUDE_EMLLGEM
  #endif
 #endif
 #if (defined INCLUDE_EMLL_SOC_TI)
  #ifndef EXCLUDE_EMLLCPSWG
   #define INCLUDE_EMLLCPSWG
  #endif
#endif
#endif

#if (EC_ARCH == EC_ARCH_PPC)
 #if (defined INCLUDE_EMLL_SOC_NXP)
  #ifndef EXCLUDE_EMLLETSEC
  #define INCLUDE_EMLLETSEC
  #endif
  #ifndef EXCLUDE_EMLLFSLFEC
  #define INCLUDE_EMLLFSLFEC
  #endif
 #endif
#endif

#endif /* EXCLUDE_EMLL_ALL */   

/*-FUNCTION DECLARATION------------------------------------------------------*/
EC_T_CHAR* GetNextWord(EC_T_CHAR **ppCmdLine, EC_T_CHAR *pStorage);

EC_T_DWORD CreateLinkParmsFromCmdLine(EC_T_CHAR** ptcWord, EC_T_CHAR** lpCmdLine, EC_T_CHAR* tcStorage, EC_T_BOOL* pbGetNextWord,
                                      EC_T_LINK_PARMS** ppLinkParms);

EC_T_VOID  FreeLinkParms(EC_T_LINK_PARMS* pLinkParms);

EC_T_BOOL  ParseIpAddress(EC_T_CHAR* ptcWord, EC_T_BYTE* pbyIpAddress);

EC_T_VOID  ShowSyntaxLinkLayer(EC_T_VOID);

#if (defined INCLUDE_EMLL_STATIC_LIBRARY)
EC_PF_LLREGISTER DemoGetLinkLayerRegFunc(EC_T_CHAR* szDriverIdent, EC_T_CHAR* szLoadPath);
#endif
#endif /* INC_SELECTLINKAYER */

/*-END OF SOURCE FILE--------------------------------------------------------*/
