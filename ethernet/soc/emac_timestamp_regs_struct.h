// Copyright 2015-2018 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief This register controls the operation of the System Time generator and the processing of PTP packets for timestamping in the Receiver.
 * 
 */
typedef union {
    struct
    {
        uint32_t en_timestamp : 1;                /* Timestamp Enable */
        uint32_t ts_fine_coarse_update : 1;       /* Timestamp Fine or Coarse Update */
        uint32_t ts_initialize : 1;               /* Timestamp Initialize */
        uint32_t ts_update : 1;                   /* Timestamp Update */
        uint32_t en_ts_int_trig : 1;              /* Timestamp Interrupt Trigger Enable */
        uint32_t addend_reg_update : 1;           /* Addend Reg Update */
        uint32_t reserved1 : 2;                   /* Reserved */
        uint32_t en_ts4all : 1;                   /* Enable Timestamp for All Frames */
        uint32_t ts_digit_bin_roll_ctrl : 1;      /* Timestamp Digital or Binary Rollover Control */
        uint32_t en_ptp_pkg_proc_ver2_fmt : 1;    /* Enable PTP packet Processing for Version 2 Format */
        uint32_t en_proc_ptp_ether_frm : 1;       /* Enable Processing of PTP over Ethernet Frames */
        uint32_t en_proc_ptp_ipv6_udp : 1;        /* Enable Processing of PTP Frames Sent over IPv6-UDP */
        uint32_t en_proc_ptp_ipv4_udp : 1;        /* Enable Processing of PTP Frames Sent over IPv4-UDP */
        uint32_t en_ts_snap_event_msg : 1;        /* Enable Timestamp Snapshot for Event Messages */
        uint32_t en_snap_msg_relevant_master : 1; /* Enable Snapshot for Messages Relevant to Master */
        uint32_t sel_snap_type : 2;               /* Select PTP packets for Taking Snapshots */
        uint32_t en_mac_addr_filter : 1;          /* Enable MAC address for PTP Frame Filtering */
        uint32_t reserved2 : 5;                   /* Reserved */
        uint32_t aux_snap_fifo_clear : 1;         /* Auxiliary Snapshot FIFO Clear */
        uint32_t en_aux_snap0 : 1;                /* Auxiliary Snapshot 0 Enable */
        uint32_t en_aux_snap1 : 1;                /* Auxiliary Snapshot 1 Enable */
        uint32_t en_aux_snap2 : 1;                /* Auxiliary Snapshot 2 Enable */
        uint32_t en_aux_snap3 : 1;                /* Auxiliary Snapshot 3 Enable */
        uint32_t reserved3 : 3;                   /* Reserved */
    };
    uint32_t val;
} emac_timestamp_ctrl_reg_t;

/**
 * @brief This register is present only when the IEEE 1588 timestamp feature is selected without an external timestamp input.
 * 
 */
typedef union {
    struct
    {
        uint32_t sub_second_incre_value : 8; /* Sub-second Increment Value */
        uint32_t reserved : 24;              /* Reserved */
    };
    uint32_t val;
} emac_sub_second_incre_reg_t;

/**
 * @brief The System Time—Seconds register, along with System Time—Nanoseconds register, indicates the current value of the system time maintained by the MAC. 
 * 
 */
typedef union {
    struct
    {
        uint32_t ts_second : 32; /* Timestamp Second */
    };
    uint32_t val;
} emac_sys_time_seconds_reg_t;

/**
 * @brief The System Time—Seconds register, along with System Time—Nanoseconds register, indicates the current value of the system time maintained by the MAC. 
 * 
 */
typedef union {
    struct
    {
        uint32_t ts_sub_seconds : 32; /* Timestamp Sub Seconds */
    };
    uint32_t val;
} emac_sys_time_nanosec_reg_t;

/**
 * @brief The System Time—Seconds Update register, along with the System Time—Nanoseconds Update register, initializes or updates the system time maintained by the MAC. 
 * 
 */
typedef union {
    struct
    {
        uint32_t ts_second : 32; /* Timestamp Second */
    };
    uint32_t val;
} emac_sys_time_seconds_update_reg_t;

/**
 * @brief This register is present only when IEEE 1588 timestamp feature is selected without external timestamp input.
 * 
 */
typedef union {
    struct
    {
        uint32_t ts_sub_seconds : 31; /* Timestamp Sub Seconds */
        uint32_t add_sub : 1;         /* Add or Subtract Time */
    };
    uint32_t val;
} emac_sys_time_nanosec_update_reg_t;

/**
 * @brief This register is present only when the IEEE 1588 Timestamp feature is selected without external timestamp input. 
 * 
 */
typedef union {
    struct
    {
        uint32_t ts_addend_val; /* Timestamp Addend Register */
    };
    uint32_t val;
} emac_timestamp_addend_reg_t;

/**
 * @brief The Target Time Seconds register, along with Target Time Nanoseconds register, is used to schedule an interrupt event 
 * 
 */
typedef union {
    struct
    {
        uint32_t tgt_time_second_val : 32; /* Target Time Seconds Register */
    };
    uint32_t val;
} emac_tgt_time_seconds_reg_t;

/**
 * @brief This register is present only when the IEEE 1588 Timestamp feature is selected without external timestamp input.
 * 
 */
typedef union {
    struct
    {
        uint32_t tgt_ts_low_reg : 31;   /* Target Timestamp Low Register */
        uint32_t tgt_time_reg_busy : 1; /* Target Time Register Busy */
    };
    uint32_t val;
} emac_tgt_time_nanosec_reg_t;

/**
 * @brief This register is present only when the IEEE 1588 Advanced Timestamp feature is selected without an external timestamp input.
 * 
 */
typedef union {
    struct
    {
        uint32_t ts_higher_word : 16; /* Timestamp Higher Word Register */
        uint32_t reserved : 16;       /* Reserved */
    };
    uint32_t val;
} emac_sys_time_higher_word_sec_reg_t;

/**
 * @brief This register is present only when the Advanced IEEE 1588 Timestamp feature is selected.
 * 
 */
typedef union {
    struct
    {
        uint32_t ts_secons_ovf : 1;             /* Timestamp Seconds Overflow */
        uint32_t ts_tgt_time_reach : 1;         /* Timestamp Target Time Reached */
        uint32_t aux_ts_trig_snap : 1;          /* Auxiliary Timestamp Trigger Snapshot */
        uint32_t ts_tgt_time_err : 1;           /* Timestamp Target Time Error */
        uint32_t ts_tgt_time_reach_pps1 : 1;    /* Timestamp Target Time Reached for Target Time PPS1 */
        uint32_t ts_tgt_time_err1 : 1;          /* Timestamp Target Time Error */
        uint32_t ts_tgt_time_reach_pps2 : 1;    /* Timestamp Target Time Reached for Target Time PPS2 */
        uint32_t ts_tgt_time_err2 : 1;          /* Timestamp Target Time Error */
        uint32_t ts_tgt_time_reach_pps3 : 1;    /* Timestamp Target Time Reached for Target Time PPS3 */
        uint32_t ts_tgt_time_err3 : 1;          /* Timestamp Target Time Error */
        uint32_t reserved1 : 6;                 /* Reserved */
        uint32_t aux_ts_snap_trig_identify : 4; /* Auxiliary Timestamp Snapshot Trigger Identifier */
        uint32_t reserved2 : 4;                 /* Reserved */
        uint32_t aux_tx_snap_trig_miss : 1;     /* Auxiliary Timestamp Snapshot Trigger Missed */
        uint32_t aux_ts_snap_num : 5;           /* Number of Auxiliary Timestamp Snapshots */
        uint32_t reserved : 2;                  /* Reserved */
    };
    uint32_t val;
} emac_timestamp_status_reg_t;

/**
 * @brief This register is present only when the Advanced Timestamp feature is selected and External Timestamp is not enabled.
 * 
 */
typedef union {
    struct
    {
        uint32_t pps_cmd0 : 4;      /* Flexible PPS0 Output Control */
        uint32_t en_pps0 : 1;       /* Flexible PPS Output Mode Enable */
        uint32_t tgt_mode_sel0 : 2; /* Target Time Register Mode for PPS0 Output */
        uint32_t reserved1 : 1;     /* Reserved */
        uint32_t pps_cmd1 : 3;      /* Flexible PPS1 Output Control */
        uint32_t reserved2 : 2;     /* Reserved */
        uint32_t tgt_mode_sel1 : 2; /* Target Time Register Mode for PPS1 Output */
        uint32_t reserved3 : 1;     /* Reserved */
        uint32_t pps_cmd2 : 3;      /* Flexible PPS2 Output Control */
        uint32_t reserved4 : 2;     /* Reserved */
        uint32_t tgt_mode_sel2 : 2; /* Target Time Register Mode for PPS2 Output */
        uint32_t reserved5 : 1;     /* Reserved */
        uint32_t pps_cmd3 : 3;      /* Flexible PPS3 Output Control */
        uint32_t reserved6 : 2;     /* Reserved */
        uint32_t tgt_mode_sel3 : 2; /* Target Time Register Mode for PPS3 Output */
        uint32_t reserved7 : 1;     /* Reserved */
    };
    uint32_t val;
} emac_pps_ctrl_reg_t;

/**
 * @brief This register, along with Register (Auxiliary Timestamp – Seconds Register), gives the 64-bit timestamp stored as auxiliary snapshot.
 * 
 */
typedef union {
    struct
    {
        uint32_t aux_ts_low : 31; /* Contains the lower 31 bits (nano-seconds field) of the auxiliary timestamp. */
        uint32_t reserved : 1;    /* Reserved */
    };
    uint32_t val;
} emac_aux_timestamp_nanosec_reg_t;

/**
 * @brief This register, along with Register (Auxiliary Timestamp – Nanoseconds Register), gives the 64-bit timestamp stored as auxiliary snapshot.
 * 
 */
typedef union {
    struct
    {
        uint32_t aux_tx_high : 32; /* Contains the lower 32 bits of the Seconds field of the auxiliary timestamp. */
    };
    uint32_t val;
} emac_aux_timestmap_seconds_reg_t;

/**
 * @brief This register controls the AV traffic by identifying the AV traffic and queuing it to appropriate channel. 
 * 
 */
typedef union {
    struct
    {
        uint32_t av_ethertype_val : 16;       /* AV EtherType Value */
        uint32_t ac_queue_pri : 3;            /* AV Priority for Queuing */
        uint32_t en_queue_non_av_pkt : 1;     /* VLAN Tagged Non-AV Packets Queueing Enable */
        uint32_t dis_av_chann : 1;            /* AV Channel Disable */
        uint32_t queue_av_ctrl_pkt_chann : 2; /* Channel for Queuing the AV Control Packets */
        uint32_t reserved1 : 1;               /* Reserved */
        uint32_t queue_ptp_pkt_chann : 2;     /* Channel for Queuing the PTP Packets */
        uint32_t reserved2 : 6;               /* Reserved */
    };
    uint32_t val;
} emac_av_mac_ctrl_reg_t;

/**
 * @brief The PPS0 Interval register contains the number of units of sub-second increment value between the rising edges of PPS0 signal output
 * 
 */
typedef union {
    struct
    {
        uint32_t pps0_interval : 32; /* PPS0 Output Signal Interval */
    };
    uint32_t val;
} emac_pps0_interval_reg_t;

/**
 * @brief The PPS0 Width register contains the number of units of sub-second increment value between the rising and corresponding falling edges of the PPS0 signal output
 * 
 */
typedef union {
    struct
    {
        uint32_t pps0_width : 32; /* PPS0 Output Signal Width */
    };
    uint32_t val;
} emac_pps0_width_reg_t;

/**
 * @brief The PPS1 Target Time Seconds register, along with PPS1 Target Time Nanoseconds register, is used to schedule an interrupt event
 * 
 */
typedef union {
    struct
    {
        uint32_t pps1_tgt_seconds : 32; /* PPS1 Target Time Seconds Register */
    };
    uint32_t val;
} emac_pps1_tgt_time_seconds_reg_t;

/**
 * @brief The PPS1 Target Time Nanoseconds register, along with PPS1 Target Time Seconds register, is used to schedule an interrupt event
 * 
 */
typedef union {
    struct
    {
        uint32_t pps1_tgt_nanosec : 31;  /* Target Time Low for PPS1 Register */
        uint32_t pps1_tgt_time_busy : 1; /* PPS1 Target Time Register Busy */
    };
    uint32_t val;
} emac_pps1_tgt_time_nanosec_reg_t;

/**
 * @brief The PPS1 Interval register contains the number of units of sub-second increment value between the rising edges of PPS1 signal output
 * 
 */
typedef union {
    struct
    {
        uint32_t pps1_interval : 32; /* PPS1 Output Signal Interval */
    };
    uint32_t val;
} emac_pps1_interval_reg_t;

/**
 * @brief The PPS1 Width register contains the number of units of sub-second increment value between the rising and corresponding falling edges of the PPS1 signal output
 * 
 */
typedef union {
    struct
    {
        uint32_t pps1_width : 32; /* PPS1 Output Signal Width */
    };
    uint32_t val;
} emac_pps1_width_reg_t;

/**
 * @brief The PPS2 Target Time Seconds register, along with PPS2 Target Time Nanoseconds register, is used to schedule an interrupt event
 * 
 */
typedef union {
    struct
    {
        uint32_t pps2_tgt_seconds : 32; /* PPS2 Target Time Seconds Register */
    };
    uint32_t val;
} emac_pps2_tgt_time_seconds_reg_t;

/**
 * @brief The PPS2 Target Time Nanoseconds register, along with PPS2 Target Time Seconds register, is used to schedule an interrupt event
 * 
 */
typedef union {
    struct
    {
        uint32_t pps2_tgt_nanosec : 31;  /* Target Time Low for PPS2 Register */
        uint32_t pps2_tgt_time_busy : 1; /* PPS2 Target Time Register Busy */
    };
    uint32_t val;
} emac_pps2_tgt_time_nanosec_reg_t;

/**
 * @brief The PPS2 Interval register contains the number of units of sub-second increment value between the rising edges of PPS2 signal output
 * 
 */
typedef union {
    struct
    {
        uint32_t pps2_interval : 32; /* PPS2 Output Signal Interval */
    };
    uint32_t val;
} emac_pps2_interval_reg_t;

/**
 * @brief The PPS2 Width register contains the number of units of sub-second increment value between the rising and corresponding falling edges of the PPS2 signal output
 * 
 */
typedef union {
    struct
    {
        uint32_t pps2_width : 32; /* PPS2 Output Signal Width */
    };
    uint32_t val;
} emac_pps2_width_reg_t;

/**
 * @brief The PPS3 Target Time Seconds register, along with PPS3 Target Time Nanoseconds register, is used to schedule an interrupt event
 * 
 */
typedef union {
    struct
    {
        uint32_t pps3_tgt_seconds : 32; /* PPS3 Target Time Seconds Register */
    };
    uint32_t val;
} emac_pps3_tgt_time_seconds_reg_t;

/**
 * @brief The PPS3 Target Time Nanoseconds register, along with PPS3 Target Time Seconds register, is used to schedule an interrupt event
 * 
 */
typedef union {
    struct
    {
        uint32_t pps3_tgt_nanosec : 31;  /* Target Time Low for PPS2 Register */
        uint32_t pps3_tgt_time_busy : 1; /* PPS2 Target Time Register Busy */
    };
    uint32_t val;
} emac_pps3_tgt_time_nanosec_reg_t;

/**
 * @brief The PPS3 Interval register contains the number of units of sub-second increment value between the rising edges of PPS3 signal output
 * 
 */
typedef union {
    struct
    {
        uint32_t pps3_interval : 32; /* PPS3 Output Signal Interval */
    };
    uint32_t val;
} emac_pps3_interval_reg_t;

/**
 * @brief The PPS3 Width register contains the number of units of sub-second increment value between the rising and corresponding falling edges of the PPS3 signal output
 * 
 */
typedef union {
    struct
    {
        uint32_t pps3_width : 32; /* PPS3 Output Signal Width */
    };
    uint32_t val;
} emac_pps3_width_reg_t;

/* --------------------------- Registers Map ---------------------------- */

/**
 * @brief Time Stamp Register Map
 * 
 */
typedef struct
{
    emac_timestamp_ctrl_reg_t timestamp_ctrl;              /* Controls the timestamp generation and update logic.  */
    emac_sub_second_incre_reg_t sub_sec_incre;             /* Contains the 8-bit value by which the Sub-Second register is incremented.  */
    emac_sys_time_seconds_reg_t sys_seconds;               /* Contains the lower 32 bits of the seconds field of the system time.  */
    emac_sys_time_nanosec_reg_t sys_nanosec;               /* Contains 32 bits of the nano-seconds field of the system time.  */
    emac_sys_time_seconds_update_reg_t sys_seconds_update; /* Contains the lower 32 bits of the seconds field to be written to, added to, or subtracted from the System Time value.  */
    emac_sys_time_nanosec_update_reg_t sys_nanosec_update; /* Contains 32 bits of the nano-seconds field to be written to, added to, or subtracted from the System Time value. */
    emac_timestamp_addend_reg_t timestamp_addend;          /* This register is used by the software to readjust the clock frequency linearly to match the master clock frequency.  */
    emac_tgt_time_seconds_reg_t tgt_seconds;               /* Contains the higher 32 bits of time to be compared with the system time for interrupt event generation or to start the PPS signal output generation.  */
    emac_tgt_time_nanosec_reg_t tgt_nanosec;               /* Contains the lower 32 bits of time to be compared with the system time for interrupt event generation or to start the PPS signal output generation.  */
    emac_sys_time_higher_word_sec_reg_t sys_seconds_high;  /* Contains the most significant 16-bits of the timestamp seconds value.  */
    emac_timestamp_status_reg_t status;                    /* Contains the PTP status.  */
    emac_pps_ctrl_reg_t pps_ctrl;                          /* This register is used to control the interval of the PPS signal output. */
    emac_aux_timestamp_nanosec_reg_t aux_nanosec;          /* Contains the lower 32 bits (nano-seconds field) of the auxiliary timestamp register. */
    emac_aux_timestmap_seconds_reg_t aux_seconds;          /* Contains the lower 32 bits of the Seconds field of the auxiliary timestamp register. */
    emac_av_mac_ctrl_reg_t av_mac_ctrl;                    /* Controls the AV traffic and queue management in the MAC Receiver.  */
    uint32_t reserved1[9];                                 /* Reserved */
    emac_pps0_interval_reg_t pps0_interval;                /* Contains the number of units of sub-second increment value between the rising edges of PPS0 signal output.  */
    emac_pps0_width_reg_t pps0_width;                      /* Contains the number of units of sub-second increment value between the rising and corresponding falling edges of PPS0 signal output.  */
    uint32_t reserved2[6];                                 /* Reserved */
    emac_pps1_tgt_time_seconds_reg_t pps1_tgt_seconds;     /* Contains the higher 32 bits of time to be compared with the system time to generate the interrupt event or to start generating the PPS1 output signal.  */
    emac_pps1_tgt_time_nanosec_reg_t pps1_tgt_nanosec;     /*Contains the lower 32 bits of time to be compared with the system time to generate the interrupt event or to start generating the PPS1 output signal.*/
    emac_pps1_interval_reg_t pps1_interval;                /* Contains the number of units of sub-second increment value between the rising edges of the PPS1 output signal.  */
    emac_pps1_width_reg_t pps1_width;                      /* Contains the number of units of sub-second increment value between the rising and corresponding falling edges of the PPS1 output signal.  */
    uint32_t reserved3[4];                                 /*Reserved*/
    emac_pps2_tgt_time_seconds_reg_t pps2_tgt_seconds;     /* Contains the higher 32 bits of time to be compared with the system time to generate the interrupt event or to start generating the PPS2 output signal.  */
    emac_pps2_tgt_time_nanosec_reg_t pps2_tgt_nanosec;     /*Contains the lower 32 bits of time to be compared with the system time to generate the interrupt event or to start generating the PPS2 output signal.*/
    emac_pps2_interval_reg_t pps2_interval;                /* Contains the number of units of sub-second increment value between the rising edges of the PPS2 output signal.  */
    emac_pps2_width_reg_t pps2_width;                      /* Contains the number of units of sub-second increment value between the rising and corresponding falling edges of the PPS2 output signal.  */
    uint32_t reserved4[4];                                 /* Reserved */
    emac_pps3_tgt_time_seconds_reg_t pps3_tgt_seconds;     /* Contains the higher 32 bits of time to be compared with the system time to generate the interrupt event or to start generating the PPS3 output signal.  */
    emac_pps3_tgt_time_nanosec_reg_t pps3_tgt_nanosec;     /*Contains the lower 32 bits of time to be compared with the system time to generate the interrupt event or to start generating the PPS3 output signal.*/
    emac_pps3_interval_reg_t pps3_interval;                /* Contains the number of units of sub-second increment value between the rising edges of the PPS3 output signal.  */
    emac_pps3_width_reg_t pps3_width;                      /* Contains the number of units of sub-second increment value between the rising and corresponding falling edges of the PPS3 output signal.  */
} emac_timestamp_regs_t;

extern emac_timestamp_regs_t EMAC_TS; /* Based Address defined in esp32/ld/esp32.peripherals.ld */

#ifdef __cplusplus
}
#endif
