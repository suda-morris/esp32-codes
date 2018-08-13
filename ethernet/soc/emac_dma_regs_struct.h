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
 * @brief The Bus Mode register establishes the bus operating modes for the DMA.
 * 
 */
typedef union {
    struct
    {
        uint32_t sw_rst : 1;              /* Software Reset */
        uint32_t dma_arbitrate_sch : 1;   /* DMA Arbitration Scheme */
        uint32_t desc_skip_len : 5;       /* Descriptor Skip Length */
        uint32_t alt_desc_size : 1;       /* Alternate Descriptor Size */
        uint32_t prog_burst_len : 6;      /* Programmable Burst Length */
        uint32_t pri_ratio : 2;           /* Priority Ratio */
        uint32_t fix_burst : 1;           /* Fixed Burst */
        uint32_t rx_dma_pbl : 6;          /* Rx DMA PBL */
        uint32_t use_separate_pbl : 1;    /* Use Separate PBL */
        uint32_t pblx8_mode : 1;          /* PBLx8 Mode */
        uint32_t addr_align_beat : 1;     /* Address-Aligned Beats */
        uint32_t mix_burst : 1;           /* Mixed Burst */
        uint32_t tx_pri : 1;              /* Transmit Priority */
        uint32_t chann_pri_weight : 2;    /* Channel Priority Weights */
        uint32_t reserved : 1;            /* Reserved */
        uint32_t rebuild_incrx_burst : 1; /* Rebuild INCRx Burst */
    };
    uint32_t val;
} emac_bus_mode_reg_t;

/**
 * @brief The Transmit Poll Demand command is given to wake up the Tx DMA if it is in the Suspend mode. 
 * 
 */
typedef union {
    struct
    {
        uint32_t tx_poll_demand : 32; /* Transmit Poll Demand */
    };
    uint32_t val;
} emac_trans_poll_demand_reg_t;

/**
 * @brief The Receive Poll Demand register enables the Rx DMA to check for new descriptors.
 * 
 */
typedef union {
    struct
    {
        uint32_t rx_poll_demand : 32; /* Receive Poll Demand */
    };
    uint32_t val;
} emac_recv_poll_demand_reg_t;

/**
 * @brief he Receive Descriptor List Address register points to the start of the Receive Descriptor List. 
 * 
 */
typedef union {
    struct
    {
        uint32_t start_of_recv_list : 32; /* Start of Receive List */
    };
    uint32_t val;
} emac_recv_desc_list_addr_reg_t;

/**
 * @brief The Transmit Descriptor List Address register points to the start of the Transmit Descriptor List. 
 * 
 */
typedef union {
    struct
    {
        uint32_t start_of_trans_list : 32; /* Start of Transmit List */
    };
    uint32_t val;
} emac_trans_desc_list_addr_reg_t;

/**
 * @brief The Status register contains all status bits that the DMA reports to the host.
 * 
 */
typedef union {
    struct
    {
        uint32_t tx_int : 1;             /* Transmit Interrupt */
        uint32_t tx_proc_stop : 1;       /* Transmit Process Stopped */
        uint32_t tx_buf_unavail : 1;     /* Transmit Buffer Unavailable */
        uint32_t tx_jabber_to : 1;       /* Transmit Jabber Timeout */
        uint32_t rx_ovf : 1;             /* Receive Overflow */
        uint32_t tx_undf : 1;            /* Transmit Underflow */
        uint32_t rx_int : 1;             /* Receive Interrupt */
        uint32_t rx_buf_unavail : 1;     /* Receive Buffer Unavailable */
        uint32_t rx_proc_stop : 1;       /* Receive Process Stopped */
        uint32_t rx_wdt_to : 1;          /* Receive Watchdog Timeout */
        uint32_t early_tx_int : 1;       /* Early Transmit Interrupt */
        uint32_t reserved1 : 2;          /* Reserved */
        uint32_t fatal_but_err_int : 1;  /* Fatal Bus Error Interrupt */
        uint32_t early_rx_int : 1;       /* Early Receive Interrupt */
        uint32_t abnormal_int_summ : 1;  /* Abnormal Interrupt Summary */
        uint32_t normal_int_summ : 1;    /* Normal Interrupt Summary */
        uint32_t rx_proc_stat : 3;       /* Receive Process State */
        uint32_t tx_proc_stat : 3;       /* Transmit Process State */
        uint32_t err_bits : 3;           /* Error Bits */
        uint32_t line_int : 1;           /* GMAC Line Interface Interrupt */
        uint32_t mmc_int : 1;            /* GMAC MMC Interrupt */
        uint32_t pmt_int : 1;            /* GMAC PMT Interrupt */
        uint32_t timestamp_trig_int : 1; /* Timestamp Trigger Interrupt */
        uint32_t lpi_int : 1;            /* GMAC LPI Interrupt (for Channel 0) */
        uint32_t reserved2 : 1;          /* Reserved */
    };
    uint32_t val;
} emac_dma_status_reg_t;

/**
 * @brief The Operation Mode register establishes the Transmit and Receive operating modes and commands.
 * 
 */
typedef union {
    struct
    {
        uint32_t reserved1 : 1;                       /* Reserved */
        uint32_t start_stop_rx : 1;                   /* Start or Stop Receive */
        uint32_t op_second_frm : 1;                   /* Operate on Second Frame */
        uint32_t rx_thresh_ctrl : 2;                  /* Receive Threshold Control */
        uint32_t drop_giant_frm : 1;                  /* Drop Giant Frames */
        uint32_t fwd_undersize_good_frm : 1;          /* Forward Undersized Good Frames */
        uint32_t fwd_err_frm : 1;                     /* Forward Error Frames */
        uint32_t en_hw_flow_ctrl : 1;                 /* Enable HW Flow Control */
        uint32_t thresh_act_flow_ctrl : 2;            /* Threshold for Activating Flow Control  */
        uint32_t thresh_deact_flow_ctrl : 2;          /* Threshold for Deactivating Flow Control  */
        uint32_t start_stop_tx_cmd : 1;               /* Start or Stop Transmission Command */
        uint32_t tx_thresh_ctrl : 3;                  /* Transmit Threshold Control */
        uint32_t reserved2 : 3;                       /* Reserved */
        uint32_t flush_tx_fifo : 1;                   /* Flush Transmit FIFO */
        uint32_t tx_store_fwd : 1;                    /* Transmit Store and Forward */
        uint32_t msb_thresh_deact_flow_ctrl : 1;      /* MSB of Threshold for Deactivating Flow Control */
        uint32_t msb_thresh_act_flow_ctrl : 1;        /* MSB of Threshold for Activating Flow Control */
        uint32_t dis_flush_recv_frm : 1;              /* Disable Flushing of Received Frames */
        uint32_t rx_store_fwd : 1;                    /* Receive Store and Forward */
        uint32_t dis_drop_tcpip_checksum_err_frm : 1; /* Disable Dropping of TCP/IP Checksum Error Frames */
        uint32_t reserved3 : 5;                       /* Reserved */
    };
    uint32_t val;
} emac_dma_op_mode_reg_t;

/**
 * @brief The Interrupt Enable register enables the interrupts reported by Status Register. 
 * 
 */
typedef union {
    struct
    {
        uint32_t en_tx_int : 1;            /* Transmit Interrupt Enable */
        uint32_t en_tx_stop : 1;           /* Transmit Stopped Enable */
        uint32_t en_tx_buf_unavail : 1;    /* Transmit Buffer Unavailable Enable */
        uint32_t en_tx_jabber_to : 1;      /* Transmit Jabber Timeout Enable */
        uint32_t en_ovf_int : 1;           /* Overflow Interrupt Enable */
        uint32_t en_undf_int : 1;          /* Underflow Interrupt Enable */
        uint32_t en_rx_int : 1;            /* Receive Interrupt Enable */
        uint32_t en_rx_buf_unavail : 1;    /* Receive Buffer Unavailable Enable */
        uint32_t en_rx_stop : 1;           /* Receive Stopped Enable */
        uint32_t en_rx_wdt_to : 1;         /* Receive Watchdog Timeout Enable */
        uint32_t en_early_tx_int : 1;      /* Early Transmit Interrupt Enable */
        uint32_t reserved1 : 2;            /* Reserved */
        uint32_t en_fatal_bus_err : 1;     /* Fatal Bus Error Enable */
        uint32_t en_early_rx_int : 1;      /* Early Receive Interrupt Enable */
        uint32_t en_abnormal_int_summ : 1; /* Abnormal Interrupt Summary Enable */
        uint32_t en_normal_int_summ : 1;   /* Normal Interrupt Summary Enable */
        uint32_t reserved2 : 15;           /* Reserved */
    };
    uint32_t val;
} emac_dma_int_en_reg_t;

/**
 * @brief The DMA maintains two counters to track the number of frames missed during reception.
 * 
 */
typedef union {
    struct
    {
        uint32_t missed_frm_cnt : 16;    /* Missed Frame Counter */
        uint32_t missed_frm_cnt_ovf : 1; /* Overflow Bit for Missed Frame Counter */
        uint32_t ovf_frm_cnt : 11;       /* Overflow Frame Counter */
        uint32_t ovf_frm_cnt_ovf : 1;    /* Overflow Bit for FIFO Overflow Counter */
        uint32_t reserved : 3;           /* Reserved */
    };
    uint32_t val;
} emac_missed_frm_buf_ovf_cnt_reg_t;

/**
 * @brief This register, when written with a non-zero value, enables the watchdog timer for the Receive Interrupt
 * 
 */
typedef union {
    struct
    {
        uint32_t rx_wdt_tmr_cnt : 8; /* RI Watchdog Timer Count */
        uint32_t reserved : 24;      /* Reserved */
    };
    uint32_t val;
} emac_recv_int_wdt_tmr_reg_t;

/**
 * @brief The Current Host Transmit Descriptor register points to the start address of the current Transmit Descriptor read by the DMA.
 * 
 */
typedef union {
    struct
    {
        uint32_t cur_tx_desc_addr_ptr : 32; /* Host Transmit Descriptor Address Pointer */
    };
    uint32_t val;
} emac_cur_trans_desc_reg_t;

/**
 * @brief The Current Host Receive Descriptor register points to the start address of the current Receive Descriptor read by the DMA.
 * 
 */
typedef union {
    struct
    {
        uint32_t cur_rx_desc_addr_ptr : 32; /* Host Receive Descriptor Address Pointer */
    };
    uint32_t val;
} emac_cur_recv_desc_reg_t;

/**
 * @brief The Current Host Transmit Buffer Address register points to the current Transmit Buffer Address being read by the DMA.
 * 
 */
typedef union {
    struct
    {
        uint32_t cur_tx_buf_addr_ptr : 32; /* Host Transmit Buffer Address Pointer */
    };
    uint32_t val;
} emac_cur_trans_buf_addr_reg_t;

typedef union {
    struct
    {
        uint32_t cur_rx_buf_addr_ptr : 32; /* Host Receive Buffer Address Pointer */
    };
    uint32_t val;
} emac_cur_recv_buf_addr_reg_t;

/**
 * @brief This register indicates the presence of the optional features or functions of the emac. 
 * 
 */
typedef union {
    struct
    {
        uint32_t sel_mii_sel : 1;                 /* 10 or 100 Mbps support */
        uint32_t sel_gmii_sel : 1;                /* 1000 Mbps support */
        uint32_t sel_half_duplex : 1;             /* Half-duplex support */
        uint32_t expand_da_hash_filter : 1;       /* Expanded DA Hash filter */
        uint32_t hash_filter : 1;                 /* HASH filter */
        uint32_t multi_mac_addr : 1;              /* Multiple MAC Address registers */
        uint32_t pcs_regs : 1;                    /* PCS registers (TBI, SGMII, or RTBI PHY interface) */
        uint32_t l3_l4_filter : 1;                /* Layer 3 and Layer 4 feature */
        uint32_t sel_sma : 1;                     /* SMA (MDIO) Interface */
        uint32_t sel_remote_wake_up : 1;          /* PMT remote wake-up frame */
        uint32_t sel_magic_packet : 1;            /* PMT magic packet R */
        uint32_t sel_mmc : 1;                     /* RMON module */
        uint32_t sel_timestamp_ver1 : 1;          /* Only IEEE 1588-2002 timestamp R */
        uint32_t sel_timestamp_ver2 : 1;          /* IEEE 1588-2008 Advanced timestamp */
        uint32_t energy_efficient : 1;            /* Energy Efficient Ethernet */
        uint32_t sel_av : 1;                      /* AV feature */
        uint32_t tx_checksum_off : 1;             /* Checksum Offload in Tx */
        uint32_t rx_ip_checksum_off_type1 : 1;    /* IP Checksum Offload (Type 1) in Rx */
        uint32_t rx_ip_checksum_off_type2 : 1;    /* IP Checksum Offload (Type 2) in Rx */
        uint32_t rx_fifo_larger_2k : 1;           /* Rx FIFO > 2,048 Bytes */
        uint32_t extra_rx_chann_count : 2;        /* Number of additional Rx Channels */
        uint32_t extra_tx_chann_count : 2;        /* Number of additional Tx Channels */
        uint32_t enhanced_desc : 1;               /* Alternate (Enhanced Descriptor) */
        uint32_t timestamp_internal_sys_time : 1; /* Timestamping with Internal System Time */
        uint32_t flex_pps_out : 1;                /* Flexible Pulse-Per-Second Output */
        uint32_t src_addr_vlan_insert : 1;        /* Source Address or VLAN Insertion */
        uint32_t sel_act_phy_if : 3;              /* Active or selected PHY interface */
        uint32_t reserved : 1;                    /* Reserved */
    };
    uint32_t val;
} emac_hw_feature_reg_t;

/* ------------------------------ Register Map ------------------------------ */

/**
 * @brief DMA Register Map
 * 
 */
typedef struct
{
    emac_bus_mode_reg_t bus_mode;                             /* Controls the Host Interface Mode. */
    emac_trans_poll_demand_reg_t trans_poll_demand;           /* Used by the host to instruct the DMA to poll the Transmit Descriptor list. */
    emac_recv_poll_demand_reg_t recv_poll_demand;             /* Used by the host to instruct the DMA to poll the Receive Descriptor list. */
    emac_recv_desc_list_addr_reg_t recv_desc_list_addr;       /* Points the DMA to the start of the Receive Descriptor list. */
    emac_trans_desc_list_addr_reg_t trans_desc_list_addr;     /* Points the DMA to the start of the Transmit Descriptor list. */
    emac_dma_status_reg_t dma_status;                         /* The Software driver (application) reads this register during interrupt service routine or polling to determine the status of the DMA. */
    emac_dma_op_mode_reg_t dma_op_mode;                       /* Establishes the Receive and Transmit operating modes and command. */
    emac_dma_int_en_reg_t dma_int_en;                         /* Enables the interrupts reported by the Status Register */
    emac_missed_frm_buf_ovf_cnt_reg_t missed_frm_buf_ovf_cnt; /* Contains the counters for discarded frames because no host Receive Descriptor was available or because of Receive FIFO Overflow. */
    emac_recv_int_wdt_tmr_reg_t recv_int_wdt_tmr;             /* Watchdog timeout for Receive Interrupt (RI) from DMA. */
    uint32_t reserved[8];                                     /* Reserved Registers */
    emac_cur_trans_desc_reg_t cur_trans_desc;                 /* Points to the start of current Transmit Descriptor read by the DMA. */
    emac_cur_recv_desc_reg_t cur_recv_desc;                   /* Points to the start of current Receive Descriptor read by the DMA. */
    emac_cur_trans_buf_addr_reg_t cur_trans_buf;              /* Points to the current Transmit Buffer address read by the DMA */
    emac_cur_recv_buf_addr_reg_t cur_recv_buf;                /* Points to the current Receive Buffer address read by the DMA. */
    emac_hw_feature_reg_t hw_feature;                         /* Indicates the presence of the optional features of the core */
} emac_dma_regs_t;

extern emac_dma_regs_t EMAC_DMA; /* Based Address defined in esp32/ld/esp32.peripherals.ld */

#ifdef __cplusplus
}
#endif
