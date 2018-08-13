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
 * @brief The MAC Configuration register establishes receive and transmit operating modes.
 * 
 */
typedef union {
    struct
    {
        uint32_t preamble_length : 2;           /* Preamble Length for Transmit frames */
        uint32_t rx_en : 1;                     /* Receiver Enable */
        uint32_t tx_en : 1;                     /* Transmitter Enable */
        uint32_t deferral_check : 1;            /* Deferral Check */
        uint32_t backoff_limit : 2;             /* Back-Off Limit */
        uint32_t auto_pad_crc_strip : 1;        /* Automatic Pad or CRC Stripping */
        uint32_t link_up_down : 1;              /* Link Up or Down */
        uint32_t dis_retry : 1;                 /* Disable Retry */
        uint32_t checksum_off : 1;              /* Checksum Offload */
        uint32_t duplex_mode : 1;               /* Duplex Mode */
        uint32_t loop_mode : 1;                 /* Loopback Mode */
        uint32_t dis_rx_own : 1;                /* Disable Receive Own */
        uint32_t speed : 1;                     /* Speed */
        uint32_t port_select : 1;               /* Port Select */
        uint32_t dis_carrier_sense : 1;         /* Disable Carrier Sense During Transmission */
        uint32_t inter_frm_gap : 3;             /* Inter-Frame Gap */
        uint32_t en_jumbo_frm : 1;              /* Jumbo Frame Enable */
        uint32_t en_fram_burst : 1;             /* Frame Burst Enable */
        uint32_t dis_jabber : 1;                /* Jabber Disable */
        uint32_t dis_wdt : 1;                   /* Watchdog Disable */
        uint32_t tx_config_info : 1;            /* Transmit Configuration in RGMII, SGMII, or SMII */
        uint32_t crc_strip : 1;                 /* CRC Stripping for Type Frames */
        uint32_t smii_force_tx_err : 1;         /* SMII Force Transmit Error */
        uint32_t en_ieee802_3as : 1;            /* IEEE 802.3as Support for 2K Packets */
        uint32_t src_addr_ins_replace_ctrl : 3; /* Source Address Insertion or Replacement Control */
        uint32_t reserved : 1;                  /* Reserved */
    };
    uint32_t val;
} emac_config_reg_t;

/**
 * @brief The MAC Frame Filter register contains the filter controls for receiving frames. 
 * 
 */
typedef union {
    struct
    {
        uint32_t promiscuous_mode : 1;    /* Promiscuous Mode */
        uint32_t hash_unicast : 1;        /* Hash Unicast */
        uint32_t hash_multicast : 1;      /* Hash Multicast */
        uint32_t da_inverse : 1;          /* DA Inverse Filtering */
        uint32_t pass_all_multicast : 1;  /* Pass All Multicast */
        uint32_t dis_broadcast : 1;       /* Disable Broadcast Frames */
        uint32_t pass_ctrl_frm : 2;       /* Pass Control Frames */
        uint32_t sa_inverse : 1;          /* SA Inverse Filtering */
        uint32_t en_src_addr_filter : 1;  /* Source Address Filter Enable */
        uint32_t hash_perfect_filter : 1; /* Hash or Perfect Filter */
        uint32_t reserved1 : 5;           /* Reserved */
        uint32_t vlan_tag_filter : 1;     /* VLAN Tag Filter Enable */
        uint32_t reserved2 : 3;           /* Reserved */
        uint32_t l3_l4_filter : 1;        /* Layer 3 and Layer 4 Filter Enable */
        uint32_t drop_non_tcp_ip_frm : 1; /* Drop non-TCP/UDP over IP Frames */
        uint32_t reserved3 : 9;           /* Reserved */
        uint32_t rx_all : 1;              /* Receive All */
    };
    uint32_t val;
} emac_frame_filter_reg_t;

/**
 * @brief The 64-bit Hash table is used for group address filtering. The Hash Table High register contains the higher 32 bits of the Hash table. 
 * 
 */
typedef union {
    struct
    {
        uint32_t hash_tbl_high : 32; /* Hash Table High */
    };
    uint32_t val;
} emac_hash_table_high_reg_t;

/**
 * @brief The Hash Table Low register contains the lower 32 bits of the Hash table. 
 * 
 */
typedef union {
    struct
    {
        uint32_t hash_tbl_low : 32; /* Hash Table Low */
    };
    uint32_t val;
} emac_hash_table_low_reg_t;

/**
 * @brief The MII Address register controls the management cycles to the external PHY through the management interface.
 * 
 */
typedef union {
    struct
    {
        uint32_t mii_busy : 1;        /* MII Busy */
        uint32_t mii_write : 1;       /* MII Write */
        uint32_t csr_clock_range : 4; /* CSR Clock Range */
        uint32_t mii_reg : 5;         /* MII Register */
        uint32_t phy_layer_addr : 5;  /* Physical Layer Address */
        uint32_t reserved : 16;       /* Reserved */
    };
    uint32_t val;
} emac_mii_addr_reg_t;

/**
 * @brief The MII Data register stores Write data to be written to the PHY register located at the address specified in MII Address Register.
 * 
 */
typedef union {
    struct
    {
        uint32_t mii_data : 16; /* MII Data */
        uint32_t reserved : 16; /* Reserved */
    };
    uint32_t val;
} emac_mii_data_reg_t;

/**
 * @brief The Flow Control register controls the generation and reception of the Control (Pause Command) frames by the MAC's Flow control module.
 * 
 */
typedef union {
    struct
    {
        uint32_t flow_ctrl_busy : 1;        /* Flow Control Busy or Backpressure Activate */
        uint32_t en_tx_flow_ctrl : 1;       /* Transmit Flow Control Enable */
        uint32_t en_recv_flow_ctrl : 1;     /* Receive Flow Control Enable */
        uint32_t detect_uni_pause_frm : 1;  /* Unicast Pause Frame Detect */
        uint32_t pause_low_thresh : 2;      /* Pause Low Threshold */
        uint32_t reserved1 : 1;             /* Reserved */
        uint32_t dis_zero_quanta_pause : 1; /* Disable Zero-Quanta Pause */
        uint32_t reserved2 : 8;             /* Reserved */
        uint32_t pause_time : 16;           /* Pause Time */
    };
    uint32_t val;
} emac_flow_ctrl_reg_t;

/**
 * @brief The VLAN Tag register contains the IEEE 802.1Q VLAN Tag to identify the VLAN frames.
 * 
 */
typedef union {
    struct
    {
        uint32_t rx_frm_vlan_tag : 16;          /* VLAN Tag Identifier for Receive Frames */
        uint32_t en_12bit_vlan_tag_compare : 1; /* Enable 12-Bit VLAN Tag Comparison */
        uint32_t en_vlan_tag_inverse_match : 1; /* VLAN Tag Inverse Match Enable */
        uint32_t en_s_vlan : 1;                 /* Enable S-VLAN */
        uint32_t en_vlan_tag_hash_tbl : 1;      /* VLAN Tag Hash Table Match Enable */
        uint32_t reversed : 12;                 /* Reversed */
    };
    uint32_t val;
} emac_vlan_tag_reg_t;

/**
 * @brief The Version registers identifies the version of the emac.
 * 
 */
typedef union {
    struct
    {
        uint32_t synopsys_ver : 8; /* Synopsys-defined Version (3.7) */
        uint32_t user_ver : 8;     /* User-defined Version */
        uint32_t reserved : 16;    /* Reserved */
    };
    uint32_t val;
} emac_version_reg_t;

/**
 * @brief The Debug register gives the status of all main modules of the transmit and receive data-paths and the FIFOs.
 * 
 */
typedef union {
    struct
    {
        uint32_t rx_proto_engine_stat : 1;     /* MAC GMII or MII Receive Protocol Engine Status */
        uint32_t mac_recv_frm_fifo_stat : 2;   /* MAC Receive Frame FIFO Controller Status */
        uint32_t reserved1 : 1;                /* Reversed */
        uint32_t rx_fifo_wr_ctrl_act_stat : 1; /* MTL Rx FIFO Write Controller Active Status */
        uint32_t rx_fifo_rd_ctrl_stat : 2;     /* MTL RxFIFO Read Controller State */
        uint32_t reserved2 : 1;                /* Reversed */
        uint32_t rx_fifo_level_stat : 2;       /* MTL RxFIFO Fill-Level Status */
        uint32_t reserved3 : 6;                /* Reversed */
        uint32_t tx_proto_engine_stat : 1;     /* MAC GMII or MII Transmit Protocol Engine Status */
        uint32_t tx_frm_ctrl_stat : 2;         /* MAC Transmit Frame Controller Status */
        uint32_t mac_tx_in_pause : 1;          /* MAC Transmitter in Pause */
        uint32_t tx_fifo_rd_ctrl_stat : 2;     /* MTL Tx FIFO Read Controller Status */
        uint32_t tx_fifo_wr_ctrl_stat : 1;     /* MTL Tx FIFO Write Controller Status */
        uint32_t reserved4 : 1;                /* Reversed */
        uint32_t tx_fifo_not_empty_stat : 1;   /* MTL Tx FIFO Not Empty Status */
        uint32_t tx_fifo_full_stat : 1;        /* MTL TxStatus FIFO Full Status */
        uint32_t reserved5 : 6;                /* Reversed */
    };
    uint32_t val;
} emac_debug_reg_t;

/**
 * @brief The register wkupfmfilter_reg loads the Wake-up Frame Filter register. 
 * 
 */
typedef union {
    struct
    {
        uint32_t wakeup_packet_filter : 32; /* Wake-Up Frame Filter */
    };
    uint32_t val;
} emac_remote_wakeup_frame_filter_reg_t;

/**
 * @brief The PMT CSR programs the request wake-up events and monitors the wake-up events.
 * 
 */
typedef union {
    struct
    {
        uint32_t power_down : 1;                         /* Power Down */
        uint32_t magic_packet_en : 1;                    /* Magic Packet Enable */
        uint32_t en_remote_wkup_frm : 1;                 /* Remote Wake-Up Frame Enable */
        uint32_t reserved1 : 2;                          /* Reserved */
        uint32_t rx_magic_pakt : 1;                      /* Magic Packet Received */
        uint32_t rx_remote_wkup_frm : 1;                 /* Remote Wake-Up Frame Received */
        uint32_t reserved2 : 2;                          /* Reserved */
        uint32_t global_unicast : 1;                     /* Global Unicast */
        uint32_t reserved3 : 14;                         /* Reserved */
        uint32_t remote_wkup_fifo_ptr : 5;               /* Remote Wake-up FIFO Pointer */
        uint32_t reserved4 : 2;                          /* Reserved */
        uint32_t remote_wkup_frm_filter_reg_ptr_rst : 1; /* Remote Wake-Up Frame Filter Register Pointer Reset */
    };
    uint32_t val;
} emac_pmt_ctrl_status_reg_t;

/**
 * @brief The LPI Control and Status Register controls the LPI functions and provides the LPI interrupt status. 
 * 
 */
typedef union {
    struct
    {
        uint32_t tx_lpi_entry : 1;     /* Transmit LPI Entry */
        uint32_t tx_lpi_exit : 1;      /* Transmit LPI Exit */
        uint32_t rx_lpi_entry : 1;     /* Receive LPI Entry */
        uint32_t rx_lpi_exit : 1;      /* Receive LPI Exit */
        uint32_t reserved1 : 4;        /* Reserved */
        uint32_t tx_lpi_state : 1;     /* Transmit LPI State */
        uint32_t rx_lpi_state : 1;     /* Receive LPI State */
        uint32_t reserved2 : 6;        /* Reserved */
        uint32_t en_lpi : 1;           /* LPI Enable */
        uint32_t phy_link_stat : 1;    /* PHY Link Status */
        uint32_t en_phy_link_stat : 1; /* PHY Link Status Enable */
        uint32_t auto_lpi_tx : 1;      /* LPI TX Automate */
        uint32_t reserved3 : 12;       /* Reserved */
    };
    uint32_t val;
} emac_lpi_ctrl_status_reg_t;

/**
 * @brief The LPI Timers Control register controls the timeout values in the LPI states.
 * 
 */
typedef union {
    struct
    {
        uint32_t lpi_tw_timer : 16; /* LPI TW Timer */
        uint32_t lpi_ls_timer : 10; /* LPI LS Timer */
        uint32_t reserved : 6;      /* Reserved */
    };
    uint32_t val;
} emac_lpi_timers_ctrl_reg_t;

/**
 * @brief The Interrupt Status register identifies the events in the MAC that can generate interrupt.
 * 
 */
typedef union {
    struct
    {
        uint32_t rgmii_smii_int_stat : 1;          /* RGMII or SMII Interrupt Status */
        uint32_t pcs_link_stat_change : 1;         /* PCS Link Status Changed */
        uint32_t pcs_auto_nego_complt : 1;         /* PCS Auto-Negotiation Complete */
        uint32_t pmt_int_stat : 1;                 /* PMT Interrupt Status */
        uint32_t mmc_int_stat : 1;                 /* MMC Interrupt Status */
        uint32_t mmc_rx_int_stat : 1;              /* MMC Receive Interrupt Status */
        uint32_t mmc_tx_int_stat : 1;              /* MMC Transmit Interrupt Status */
        uint32_t mmc_rx_checksum_off_int_stat : 1; /* MMC Receive Checksum Offload Interrupt Status */
        uint32_t reserved1 : 1;                    /* Reserved */
        uint32_t timestamp_int_stat : 1;           /* Timestamp Interrupt Status */
        uint32_t lpi_int_stat : 1;                 /* LPI Interrupt Status */
        uint32_t gpi_int_stat : 1;                 /* GPI Interrupt Status */
        uint32_t reserved2 : 20;                   /* Reserved */
    };
    uint32_t val;
} emac_int_status_reg_t;

/**
 * @brief The Interrupt Mask Register bits enable you to mask the interrupt signal because of the corresponding event in the Interrupt Status Register. 
 * 
 */
typedef union {
    struct
    {
        uint32_t rgmii_smii_int_mask : 1;           /* RGMII or SMII Interrupt Mask */
        uint32_t pcs_link_stat_int_mask : 1;        /* PCS Link Status Interrupt Mask */
        uint32_t pcs_auto_nego_complt_int_mask : 1; /* PCS AN Completion Interrupt Mask */
        uint32_t pmt_int_mask : 1;                  /* PMT Interrupt Mask */
        uint32_t reserved1 : 5;                     /* Reserved */
        uint32_t timestamp_int_mask : 1;            /* Timestamp Interrupt Mask */
        uint32_t lpi_int_mask : 1;                  /* LPI Interrupt Mask */
        uint32_t reserved2 : 21;                    /* Reserved */
    };
    uint32_t val;
} emac_int_mask_reg_t;

/**
 * @brief The MAC Address0 High register holds the upper 16 bits of the first 6-byte MAC address of the station.
 * 
 */
typedef union {
    struct
    {
        uint32_t mac_addr0_high : 16; /* MAC Address0 [47:32] */
        uint32_t reserved : 15;       /* Reserved */
        uint32_t addr_en : 1;         /* Address Enable */
    };
    uint32_t val;
} emac_mac_addr0_high_reg_t;

/**
 * @brief The MAC Address0 Low register holds the lower 32 bits of the 6-byte first MAC address of the station.
 * 
 */
typedef union {
    struct
    {
        uint32_t mac_addr0_low : 32; /* MAC Address0 [31:0] */
    };
    uint32_t val;
} emac_mac_addr0_low_reg_t;

/**
 * @brief The MAC Address High register holds the upper 16 bits of the second 6-byte MAC address of the station.
 * 
 */
typedef union {
    struct
    {
        uint32_t mac_addr_high : 16; /* MAC Address [47:32] */
        uint32_t reserved : 8;       /* Reserved */
        uint32_t mask_byte_ctrl : 6; /* Mask Byte Control */
        uint32_t src_addr : 1;       /* Source Address */
        uint32_t addr_en : 1;        /* Address Enable */
    };
    uint32_t val;
} emac_mac_addr1_15_high_reg_t;

/**
 * @brief The MAC Address Low register holds the lower 32 bits of the second 6-byte MAC address of the station.
 * 
 */
typedef union {
    struct
    {
        uint32_t mac_addr_low : 32; /* MAC Address [31:0] */
    };
    uint32_t val;
} emac_mac_addr1_15_low_reg_t;

/**
 * @brief Other 15 MAC address registers
 * 
 */
typedef struct
{
    emac_mac_addr1_15_high_reg_t high; /* high part */
    emac_mac_addr1_15_low_reg_t low;   /* low part */
} emac_mac_addr1_15_reg_t;

/**
 * @brief The AN Control register enables and/or restarts auto-negotiation. 
 * 
 */
typedef union {
    struct
    {
        uint32_t reserved1 : 9;         /* Reserved */
        uint32_t restart_auto_nego : 1; /* Restart Auto-Negotiation */
        uint32_t reserved2 : 2;         /* Reserved */
        uint32_t en_auto_nego : 1;      /* Auto-Negotiation Enable */
        uint32_t reserved3 : 1;         /* Reserved */
        uint32_t en_external_loop : 1;  /* External Loopback Enable */
        uint32_t reserved4 : 1;         /* Reserved */
        uint32_t en_comma_detect : 1;   /* Enable Comma Detect */
        uint32_t lock_to_refer : 1;     /* Lock to Reference */
        uint32_t sgmii_ral_ctrl : 1;    /* SGMII RAL Control */
        uint32_t reserved5 : 13;        /* Reserved */
    };
    uint32_t val;
} emac_auto_nego_ctrl_reg_t;

/**
 * @brief The AN Status register indicates the link and the auto-negotiation status. 
 * 
 */
typedef union {
    struct
    {
        uint32_t reserved1 : 2;         /* Reserved */
        uint32_t link_stat : 1;         /* This bit indicates whether the data channel (link) is up or down.  */
        uint32_t auto_nego_ability : 1; /* This bit is always high because the MAC supports autonegotiation. */
        uint32_t reserved2 : 1;         /* Reserved */
        uint32_t auto_nego_complt : 1;  /* When set, this bit indicates that the auto-negotiation process is complete. */
        uint32_t reserved3 : 2;         /* Reserved */
        uint32_t entended_stat : 1;     /* This bit is tied to high if the TBI or RTBI interface is selected during core configuration indicating that the MAC supports extended status information */
        uint32_t reserved4 : 23;        /* Reserved */
    };
    uint32_t val;
} emac_auto_nego_stat_reg_t;

/**
 * @brief The Auto-Negotiation Advertisement register indicates the link and the auto-negotiation status. 
 * 
 */
typedef union {
    struct
    {
        uint32_t reserved1 : 5;             /* Reserved */
        uint32_t full_duplex : 1;           /* Full-Duplex */
        uint32_t halp_duplex : 1;           /* Half-Duplex */
        uint32_t pause_encoding : 2;        /* Pause Encoding */
        uint32_t reserved2 : 3;             /* Reserved */
        uint32_t remote_fault_encoding : 2; /* Remote Fault Encoding */
        uint32_t reserved3 : 1;             /* Reserved */
        uint32_t support_next_page : 1;     /* Next Page Support */
        uint32_t reserved4 : 16;            /* Reserved */
    };
    uint32_t val;
} emac_auto_nego_advertise_reg_t;

/**
 * @brief The Auto-Negotiation Link Partner Ability register contains the advertised ability of the link partner. 
 * 
 */
typedef union {
    struct
    {
        uint32_t reserved1 : 5;             /* Reserved */
        uint32_t full_duplex : 1;           /* Full-Duplex */
        uint32_t half_duplex : 1;           /* Half-Duplex */
        uint32_t pause_encoding : 1;        /* Pause Encoding */
        uint32_t reserved2 : 3;             /* Reserved */
        uint32_t remote_fault_encoding : 2; /* Remote Fault Encoding */
        uint32_t ack : 1;                   /* Acknowledge */
        uint32_t support_next_page : 1;     /* Next Page Support */
        uint32_t reserved3 : 16;            /* Reserved */
    };
    uint32_t val;
} emac_auto_nego_partner_ability_reg_t;

/**
 * @brief The Auto-Negotiation Expansion register indicates if the MAC received a new base page from the link partner. 
 * 
 */
typedef union {
    struct
    {
        uint32_t reserved1 : 1;         /* Reserved */
        uint32_t rx_new_page : 1;       /* New Page Received */
        uint32_t next_page_ability : 1; /* Next Page Ability */
        uint32_t reserved2 : 29;        /* Reserved */
    };
    uint32_t val;
} emac_auto_nego_expan_reg_t;

/**
 * @brief The TBI Extended Status register indicates all modes of operation of the MAC.
 * 
 */
typedef union {
    struct
    {
        uint32_t reserved1 : 14;       /* Reserved */
        uint32_t giga_halpdup_cap : 1; /* 1000BASE-X Half-Duplex Capable */
        uint32_t giga_fulldup_cap : 1; /* 1000BASE-X Full-Duplex Capable */
        uint32_t reserved2 : 16;       /* Reserved */
    };
    uint32_t val;
} emac_tbi_extend_stat_reg_t;

/**
 * @brief The SGMII/RGMII/SMII Control and Status register indicates the status signals received by the SGMII, RGMII, or SMII interface (selected at reset) from the PHY.
 * 
 */
typedef union {
    struct
    {
        uint32_t link_mode : 1;     /* Link Mode */
        uint32_t link_speed : 2;    /* Link Speed */
        uint32_t link_stat : 1;     /* Link Status */
        uint32_t jabber_to : 1;     /* Jabber Timeout */
        uint32_t false_carrier : 1; /* False Carrier Detected */
        uint32_t reserved1 : 10;    /* Reserved */
        uint32_t smii_delay_rx : 1; /* Delay SMII RX Data Sampling with respect to the SMII SYNC Signal */
        uint32_t reserved2 : 15;    /* Reserved */
    };
    uint32_t val;
} emac_sgmii_rgmii_smii_ctrl_stat_reg_t;

/**
 * @brief This register controls the watchdog timeout for received frames.
 * 
 */
typedef union {
    struct
    {
        uint32_t wdt_timeout : 14; /* Watchdog Timeout */
        uint32_t reserved1 : 2;    /* Reserved */
        uint32_t en_prog_wdt : 1;  /* Programmable Watchdog Enable */
        uint32_t reserved2 : 15;   /* Reserved */
    };
    uint32_t val;
} emac_wdt_timeout_reg_t;

/**
 * @brief This register provides the control to drive up to 4 bits of output ports (GPO) and the status of up to 4 input ports (GPIS).
 * 
 */
typedef union {
    struct
    {
        uint32_t input_stat : 4; /* General Purpose Input Status */
        uint32_t reserved1 : 4;  /* Reserved */
        uint32_t output : 4;     /* General Purpose Output */
        uint32_t reserved2 : 4;  /* Reserved */
        uint32_t int_en : 4;     /* GPI Interrupt Enable */
        uint32_t reserved3 : 4;  /* Reserved */
        uint32_t type : 4;       /* GPI Type */
        uint32_t reserved4 : 4;  /* Reserved */
    };
    uint32_t val;
} emac_gpio_reg_t;

/* --------------------------- Registers Map ---------------------------- */

/**
 * @brief Common Register Map
 * 
 */
typedef struct
{
    emac_config_reg_t config;                                         /* This is the operation mode register for the MAC. */
    emac_frame_filter_reg_t frame_filter;                             /* Contains the frame filtering controls. */
    emac_hash_table_high_reg_t hash_table_high;                       /* Contains the higher 32 bits of the Multicast Hash table. */
    emac_hash_table_low_reg_t hash_table_low;                         /* Contains the lower 32 bits of the Multicast Hash table.  */
    emac_mii_addr_reg_t mii_addr;                                     /* Controls the management cycles to an external PHY. */
    emac_mii_data_reg_t mii_data;                                     /* Contains the data to be written to or read from the PHY register.  */
    emac_flow_ctrl_reg_t flow_ctrl;                                   /* Controls the generation of control frames. */
    emac_vlan_tag_reg_t valn_tag;                                     /* Identifies IEEE 802.1Q VLAN type frames. */
    emac_version_reg_t version;                                       /* Identifies the version of the Core. */
    emac_debug_reg_t debug;                                           /* Gives the status of various internal blocks for debugging. */
    emac_remote_wakeup_frame_filter_reg_t wkup_frm_filter;            /* This is the address through which the application writes or reads the remote wake-up frame filter registers (wkupfmfilter_reg). */
    emac_pmt_ctrl_status_reg_t pmt_ctrl_stat;                         /* The PMT CSR programs the request wake-up events and monitors the wake-up events. */
    emac_lpi_ctrl_status_reg_t lpi_ctrl_stat;                         /* Controls the Low Power Idle (LPI) operations and provides the LPI status of the core. */
    emac_lpi_timers_ctrl_reg_t lpi_tmr_ctrl;                          /* Controls the timeout values in LPI states.  */
    emac_int_status_reg_t int_stat;                                   /* Contains the interrupt status. */
    emac_int_mask_reg_t int_mask;                                     /* Contains the masks for generating the interrupts. */
    emac_mac_addr0_high_reg_t addr0_high;                             /* Contains the higher 16 bits of the first MAC address */
    emac_mac_addr0_low_reg_t addr0_low;                               /* Contains the lower 32 bits of the first MAC address. */
    emac_mac_addr1_15_reg_t addr1_15[15];                             /* Contains the other 15 MAC address */
    emac_auto_nego_ctrl_reg_t auto_nego_ctrl;                         /* Enables and/or restarts auto-negotiation.  */
    emac_auto_nego_stat_reg_t auto_nego_stat;                         /* ndicates the link and auto-negotiation status. */
    emac_auto_nego_advertise_reg_t auto_nego_advertise;               /*  It contains the advertised ability of the MAC.  */
    emac_auto_nego_partner_ability_reg_t auto_nego_partner_ability;   /* Contains the advertised ability of the link partner.  */
    emac_auto_nego_expan_reg_t auto_nego_expand;                      /* Indicates whether a new base page has been received from the link partner. */
    emac_tbi_extend_stat_reg_t tbi_extend_stat;                       /* Indicates all modes of operation of the MAC.  */
    emac_sgmii_rgmii_smii_ctrl_stat_reg_t sgmii_rgmii_smii_ctrl_stat; /* Indicates the status signals received from the PHY through the SGMII, RGMII, or SMII interface. */
    emac_wdt_timeout_reg_t wdt_timeout;                               /* Controls the watchdog timeout for received frames */
    emac_gpio_reg_t gpio;                                             /* Provides the control to drive up to 4 bits of output ports (GPO) and also provides the status of up to 4 input ports (GPIS). */
} emac_common_regs_t;

extern emac_common_regs_t EMAC_COM; /* Based Address defined in esp32/ld/esp32.peripherals.ld */

#ifdef __cplusplus
}
#endif
