/*-----------------------------------------------------------------------------
 * EcFeatures.h
 * Copyright                acontis technologies GmbH, Ravensburg, Germany
 * Response                 Paul Bussmann
 * Description              EC-Master / EC-Simulator feature selection
 *---------------------------------------------------------------------------*/

#ifndef INC_ECFEATURES
#define INC_ECFEATURES

/*-EC-MASTER-----------------------------------------------------------------*/

/* bit mask for supported features (used for license check) */
#define EC_FEATURES_MASTER_MASK     0x00000001
#define EC_FEATURES_RED_MASK        0x00000002

/* Class B */
#ifdef  EXCLUDE_EC_MASTER_CLASS_B
#define EXCLUDE_EC_MASTER_CLASS_A           /* Class A needs Class B */
#define EXCLUDE_AOE_SUPPORT                 /* ADS over EtherCAT */
#define EXCLUDE_EOE_SUPPORT                 /* Ethernet over EtherCAT */
#define EXCLUDE_FOE_SUPPORT                 /* File access over EtherCAT */
#define EXCLUDE_SOE_SUPPORT                 /* ServoDrive over EtherCAT */
#define EXCLUDE_VOE_SUPPORT                 /* Vendor specific access over EtherCAT */
#define EXCLUDE_FRAME_LOGGING               /* Callback interface to log frames */
#define EXCLUDE_FRAME_LOSS_SIMULATION
#define EXCLUDE_MASTER_OBD                  /* Master Object Dictionary */
#define EXCLUDE_MEMORY_PROVIDER             /* Memory provider */
#define EXCLUDE_SLAVE_STATISTICS            /* Cyclic reading of slave error registers */
#define EXCLUDE_VARREAD                     /* Read PD variable tags from XML */
#define EXCLUDE_EEPROM_SUPPORT              /* Eeprom */
#endif

/* Class A */
#ifdef  EXCLUDE_EC_MASTER_CLASS_A
#define EXCLUDE_DC_SUPPORT                  /* Distributed Clocks */
#define EXCLUDE_MULTIPLE_CYC_ENTRIES        /* Multiple cyclic entries */
#define EXCLUDE_RED_DEVICE                  /* Cable Redundancy */
#define EXCLUDE_MASTER_RED                  /* Master Redundancy */
#endif

/* base features */
#ifdef  EXCLUDE_AOE_SUPPORT
#define EXCLUDE_ADS_ADAPTER                 /* ADS adapter needs AoE */
#else
#define INCLUDE_AOE_SUPPORT                 /* ADS over EtherCAT */
#endif
#ifndef EXCLUDE_DC_SUPPORT
#define INCLUDE_DC_SUPPORT                  /* Distributed Clocks */
#endif
#ifndef EXCLUDE_DC_ADD_ACYC_DISTRIBUTION
#define INCLUDE_DC_ADD_ACYC_DISTRIBUTION    /* Additional system time distribution in acyc frame */
#endif
#ifndef EXCLUDE_EOE_SUPPORT
#define INCLUDE_EOE_SUPPORT                 /* Ethernet over EtherCAT */
#endif
#ifndef EXCLUDE_EEPROM_SUPPORT
#define INCLUDE_EEPROM_SUPPORT              /* Eeprom */
#endif
#ifndef EXCLUDE_COE_SUPPORT
#define INCLUDE_COE_SUPPORT                 /* CAN application protocol over EtherCAT */
#endif
#ifndef EXCLUDE_FOE_SUPPORT
#define INCLUDE_FOE_SUPPORT                 /* File access over EtherCAT */
#endif
#ifndef EXCLUDE_FRAME_LOGGING
#define INCLUDE_FRAME_LOGGING               /* Callback interface to log frames */
#endif
#ifndef EXCLUDE_FRAME_LOSS_SIMULATION
#define FRAME_LOSS_SIMULATION               /* Frame loss simulation */
#endif
#ifndef EXCLUDE_GEN_OP_ENI
#define INCLUDE_GEN_OP_ENI                  /* Generation of ENI file to reach OP state */
#endif
#ifdef  EXCLUDE_LOG_MESSAGES
#define EXCLUDE_TEXT
#else
#define INCLUDE_LOG_MESSAGES                /* Log messages */
#endif
#ifndef EXCLUDE_TEXT
#define INCLUDE_TEXT                        /* Text returned by e.g. EcErrorText() */
#endif
#ifndef EXCLUDE_MASTER_OBD
#define INCLUDE_MASTER_OBD                  /* Master Object Dictionary */
#endif
#ifndef EXCLUDE_MASTERSYNCUNITS
#define INCLUDE_MASTERSYNCUNITS             /* Master Sync Units */
#endif
#ifndef EXCLUDE_MEMORY_PROVIDER
#define INCLUDE_MEMORY_PROVIDER             /* Memory provider */
#endif
#ifndef EXCLUDE_MULTIPLE_CYC_ENTRIES
#define INCLUDE_MULTIPLE_CYC_ENTRIES        /* Multiple cyclic entries */
#endif
#ifndef EXCLUDE_PORT_OPERATION
#define INCLUDE_PORT_OPERATION              /* Port operation (close/open ports blocknode) */
#endif
#ifndef EXCLUDE_RAWMBX_SUPPORT
#define INCLUDE_RAWMBX_SUPPORT              /* Raw mailbox transfer (e.g. used by mailbox gateway) */
#endif
#ifndef EXCLUDE_S2SMBX_SUPPORT
#define INCLUDE_S2SMBX_SUPPORT              /* S2S mailbox transfer */
#endif
#ifndef EXCLUDE_SOE_SUPPORT
#define INCLUDE_SOE_SUPPORT                 /* ServoDrive over EtherCAT */
#endif
#ifndef EXCLUDE_SLAVE_STATISTICS
#define INCLUDE_SLAVE_STATISTICS            /* Cyclic reading of slave error registers */
#endif
#ifndef EXCLUDE_TRACE_DATA
#define INCLUDE_TRACE_DATA                  /* Trace data */
#ifndef EXCLUDE_TRACE_DATA_VARINFO
#define INCLUDE_TRACE_DATA_VARINFO          /* Trace data variable info */
#endif
#endif
#ifndef EXCLUDE_VARREAD
#define INCLUDE_VARREAD                     /* Read PD variable tags from XML */
#endif
#ifndef EXCLUDE_VOE_SUPPORT
#define INCLUDE_VOE_SUPPORT                 /* Vendor specific access over EtherCAT */
#endif
#ifndef EXCLUDE_WKCSTATE
#define INCLUDE_WKCSTATE                    /* WkcState bits in diagnosis image */
#endif
#ifndef EXCLUDE_XPAT
#define INCLUDE_XPAT                        /* ENI file parser */
#endif
#ifndef EXCLUDE_RAS_SPOCSUPPORT
#define INCLUDE_RAS_SPOCSUPPORT
#endif
#ifndef EXCLUDE_FORCE_PROCESSDATA
#define INCLUDE_FORCE_PROCESSDATA           /* Force/Release process data */
#endif
#ifndef EXCLUDE_LINE_CROSSED_DETECTION      /* Line crossed detection */
#define INCLUDE_LINE_CROSSED_DETECTION
#endif
#ifndef EXCLUDE_OEM
#define INCLUDE_OEM
#endif
#ifndef EXCLUDE_JUNCTION_REDUNDANCY
#define INCLUDE_JUNCTION_REDUNDANCY         /* Junction Redundancy */
#endif

#if 0 /* features disabled by default */
#ifndef VLAN_FRAME_SUPPORT
#define VLAN_FRAME_SUPPORT                  /* VLAN frame support */
#endif
#endif /* features disabled by default */

#ifndef EXCLUDE_PERF_MEAS
#define INCLUDE_PERF_MEAS
#endif
#if 0 /* features disabled by default */
#ifndef INCLUDE_EC_PERF_MEAS_DEBUG
#define INCLUDE_EC_PERF_MEAS_DEBUG      /* internal performance measurement */
#endif
#endif

/* feature packs */
#ifndef EXCLUDE_RED_DEVICE
#define INCLUDE_RED_DEVICE                  /* Cable Redundancy */
#endif
#ifndef EXCLUDE_ADS_ADAPTER
#define INCLUDE_ADS_ADAPTER
#endif
#ifndef EXCLUDE_EOE_ENDPOINT
#define INCLUDE_EOE_ENDPOINT                /* Ethernet over EtherCAT end point */
#endif
#ifndef EXCLUDE_HOTCONNECT                  /* Hot Connect */
#define INCLUDE_HOTCONNECT
#endif
#if 0
#define INCLUDE_PASS_THROUGH_SERVER
#endif
#if (!defined INCLUDE_MASTER_RED) && (!defined EXCLUDE_MASTER_RED) && ((defined EC_VERSION_WINDOWS) || (defined EC_VERSION_LINUX)) && ((EC_ARCH_X86 == EC_ARCH) || (EC_ARCH_X64 == EC_ARCH) || (EC_ARCH_ARM == EC_ARCH))
#define INCLUDE_MASTER_RED                  /* Master Redundancy */
#endif
#ifndef EXCLUDE_DC_SUPPORT
#ifndef EXCLUDE_DCX                         /* DCX external synchronization */
#define INCLUDE_DCX
#endif
#endif /* !EXCLUDE_DC_SUPPORT */
#ifndef EXCLUDE_RESCUE_SCAN                 /* Rescue Scan */
#define INCLUDE_RESCUE_SCAN
#endif
#ifndef EXCLUDE_CONFIG_EXTEND
#define INCLUDE_CONFIG_EXTEND               /* Extend configuration to be able to set unexpected bus slaves to PREOP */
#endif
#ifndef EXCLUDE_MAILBOX_STATISTICS
#define INCLUDE_MAILBOX_STATISTICS          /* Collect statistics of mailbox transfers */
#endif
#ifndef EXCLUDE_SPLITTED_FRAME_PROCESSING   /* eUsrJob_ProcessRxFramesByTaskId instead of eUsrJob_ProcessAllRxFrames */
#define INCLUDE_SPLITTED_FRAME_PROCESSING
#endif
#ifndef EXCLUDE_EOE_DEFFERED_SWITCHING      /* EoE frames processed in eUsrJob_SwitchEoeFrames instead of eUsrJob_ProcessAllRxFrames */
#define INCLUDE_EOE_DEFFERED_SWITCHING
#endif
#ifndef EXCLUDE_SLAVE_HANDLING              /* emSetSlaveDisabled() / emSetSlaveDisconnected() */
#define INCLUDE_SLAVE_HANDLING
#endif
#ifndef EXCLUDE_SLAVE_IDENTIFICATION        /* Explicit device identification mechanism */
#define INCLUDE_SLAVE_IDENTIFICATION
#endif
#ifndef EXCLUDE_EXECJOB_REENTRANCY_SUPPORT  /* emExecJob() can be called multiple time in parallel */
#define INCLUDE_EXECJOB_REENTRANCY_SUPPORT
#endif
#ifndef EXCLUDE_INTERFACE_LOCK              /* API calls are locked against ecatMasterInit() / ecatMasterDeinit() */
#define INCLUDE_INTERFACE_LOCK
#endif
#ifndef EXCLUDE_BAD_CONNECTIONS             /* Bad connection detection */
#define INCLUDE_BAD_CONNECTIONS
#endif


/*-EC-SIMULATOR--------------------------------------------------------------*/

/* EC-Simulator HiL / SiL */
#ifndef EXCLUDE_EC_SIMULATOR_HIL            /* EC-Simulator HiL */
#define INCLUDE_EC_SIMULATOR_HIL
#endif
#ifndef EXCLUDE_EC_SIMULATOR_SIL            /* EC-Simulator SiL */
#define INCLUDE_EC_SIMULATOR_SIL
#endif

#endif /* INC_ECFEATURES */

/*-END OF SOURCE FILE--------------------------------------------------------*/
