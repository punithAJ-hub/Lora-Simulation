/**
 * @file      sx126x_lr_fhss_ping.c
 *
 * @brief     SX126x LR-FHSS ping demo, main example source code
 *
 * 
 */

#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <stdio.h>
#include <sxlib/Gpio/Led/Generic_Led.h>
#include <sxlib/Debug/Log/Generic_Log.h>

#include "sx126x.h"
#include "sx126x_comp.h"
#include "sx126x_lr_fhss.h"
#include "demos/sx126x_lr_fhss_ping/sx126x_lr_fhss_ping.h"
//#include "C:\Users\hi7349\Downloads\SWDM001-master\SWDM001-master\project\keil_polling_STM32L476\sequencegenerator.c"
#include <time.h>
 /*
  * -----------------------------------------------------------------------------
  * --- PRIVATE CONSTANTS -------------------------------------------------------
  */

  /** @brief Transmit power */
#define POWER_IN_DBM ( 14 )

/** @brief Center frequency */
#define RF_FREQUENCY ( 914200000 )

/** @brief First payload length used. Must be between 1 and the maximum allowed payload length for the LR-FHSS comm
 * parameters.
 */
#define MIN_PAYLOAD_LENGTH (10)

 /** @brief Largest payload length used before returning to MIN_PAYLOAD_LENGTH. Must be between MIN_PAYLOAD_LENGTH and
  * the maximum allowed payload length for the LR-FHSS comm parameters */
#define MAX_PAYLOAD_LENGTH (10)

#define BANDWIDTH (25400)
#define HOPS (5)
#define NO_NODES (8)
#define NO_CARRIERS (10)

#define NODE (1)

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>

#define MAX_NODES 6      // Maximum number of nodes
#define MAX_CARRIERS 9   // Maximum number of carriers
#define MAX_ELEMENTS 5     // Maximum number of elements per node

struct Node {
    int elements[MAX_ELEMENTS];  // Use a fixed-size array
    int edge_count;
};

// Function to generate sequences
static struct Node* generateSequences(int no_nodes, int no_carriers, int elements_size) {
    static struct Node nodes[MAX_NODES]; // Static allocation of nodes
    int NUM_NODES = no_nodes;
    int NUM_CARRIERS = no_carriers;

    for (int i = 0; i < NUM_NODES; ++i) {
        nodes[i].edge_count = 0;

        for (int j = 0; j < elements_size; ++j) {
            int curr_best_cardinality = 1000000;
            int curr_best_cardinality_carrier = -1;

            for (int k = 0; k < NUM_CARRIERS; ++k) {
                int used = 0;
                for (int l = 0; l < j; ++l) {
                    if (nodes[i].elements[l] == k) {
                        used = 1;
                        break;
                    }
                }

                if (used) {
                    continue;
                }

                int max_cardinality = -1;
                for (int l = 0; l < i; ++l) {
                    for (int m = 0; m < elements_size; ++m) {  // Check all elements
                        if (nodes[l].elements[m] == k) {
                            if (max_cardinality < nodes[l].edge_count) {
                                max_cardinality = nodes[l].edge_count;
                            }
                        }
                    }
                }

                max_cardinality += 1;

                if (max_cardinality < curr_best_cardinality) {
                    curr_best_cardinality = max_cardinality;
                    curr_best_cardinality_carrier = k;
                }
            }

            assert(curr_best_cardinality_carrier != -1);

            nodes[i].elements[j] = curr_best_cardinality_carrier;

            for (int l = 0; l < i; ++l) {
                for (int m = 0; m < elements_size; ++m) {  // Check all elements
                    if (nodes[l].elements[m] == curr_best_cardinality_carrier) {
                        nodes[l].edge_count += 1;
                        nodes[i].edge_count += 1;
                    }
                }
            }
        }
    }

    return nodes;
}
  /*
   * -----------------------------------------------------------------------------
   * --- PRIVATE VARIABLES -------------------------------------------------------
   */

   /** @brief LR-FHSS sync word storage */
static const uint8_t lr_fhss_sync_word[LR_FHSS_SYNC_WORD_BYTES] = { 0x2C, 0x0F, 0x79, 0x95 };

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DECLARATIONS -------------------------------------------
 */

 /**
  * @brief Enter standby mode, then clear all interrupts
  *
  * @param [in] context Chip implementation context
  */
static void set_standby_clear_all_irqs(const void* context);

/**
 * @brief Enter standby mode, clear all interrupts, and then optionally sleep
 * if CONFIG_ALLOW_SMTC_RADIO_SLEEP is defined
 *
 * @param [in] state           Pointer to application state
 */
static void enter_standby_then_sleep_mode(const sx126x_lr_fhss_ping_state_t* state);

/**
 * @brief Exit sleep mode if CONFIG_ALLOW_SMTC_RADIO_SLEEP is defined
 *
 * @param [in] state           Pointer to application state
 */
static void exit_sleep_mode(const sx126x_lr_fhss_ping_state_t* state);

/**
 * @brief Send the provided payload over LR-FHSS, using the parameters provided in this function
 *
 * @param [in,out] state      Pointer to application state
 * @param [in] payload        Pointer to application payload
 * @param [in] payload_length Length of application payload
 */
static sx126x_status_t lr_fhss_send_packet(sx126x_lr_fhss_ping_state_t* state, const uint8_t* payload,
    uint16_t payload_length);

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTION DEFINITIONS ---------------------------------------------
 */





sx126x_status_t configure_lora_reception(const sx126x_lr_fhss_ping_state_t* state)
{
    sx126x_mod_params_lora_t lora_params = {
        .sf = SX126X_LORA_SF7,
        .bw = SX126X_LORA_BW_125,
        .cr = SX126X_LORA_CR_4_5,
        .ldro = 0,
    };

    sx126x_pkt_params_lora_t pkt_params = {
        .preamble_len_in_symb = 8,
        .header_type = SX126X_LORA_PKT_EXPLICIT,
        .pld_len_in_bytes = 2,
        .crc_is_on = true,
        .invert_iq_is_on = false,
    };

    sx126x_status_t status;

    status = sx126x_set_pkt_type(state->config->radio, SX126X_PKT_TYPE_LORA);
    if (status != SX126X_STATUS_OK) return status;

    status = sx126x_set_rf_freq(state->config->radio, 914200000);
    if (status != SX126X_STATUS_OK) return status;

    status = sx126x_set_lora_mod_params(state->config->radio, &lora_params);
    if (status != SX126X_STATUS_OK) return status;

    status = sx126x_set_lora_pkt_params(state->config->radio, &pkt_params);
    if (status != SX126X_STATUS_OK) return status;

    status = sx126x_set_lora_sync_word(state->config->radio, 0x12);
    if (status != SX126X_STATUS_OK) return status;

    return SX126X_STATUS_OK;
}


void sx126x_lr_fhss_ping_handle_rx_done(sx126x_lr_fhss_ping_state_t* state)
{
    if (!state->waiting_for_ack) {
        return;  // Ignore if we're not waiting for ACK
    }

    uint8_t buffer[2];
    sx126x_rx_buffer_status_t rx_buffer_status;
    sx126x_pkt_status_lora_t pkt_status;

    sx126x_get_rx_buffer_status(state->config->radio, &rx_buffer_status);
    sx126x_get_lora_pkt_status(state->config->radio, &pkt_status);

    if (rx_buffer_status.pld_len_in_bytes == 2) {
        sx126x_read_buffer(state->config->radio, rx_buffer_status.buffer_start_pointer, buffer, 2);
        if (buffer[0] == '1' && buffer[1] == '1') {
            SXLIB_LOG(SXLIB_LOG_APP, ("ACK detected!" SXLIB_LOG_EOL));
            SXLIB_LOG(SXLIB_LOG_APP, ("SNR: %d, RSSI: %d" SXLIB_LOG_EOL, pkt_status.snr_pkt_in_db, pkt_status.rssi_pkt_in_dbm));
        }
        else {
            SXLIB_LOG(SXLIB_LOG_APP, ("Received packet, but not ACK" SXLIB_LOG_EOL));
        }
    }
    else {
        SXLIB_LOG(SXLIB_LOG_APP, ("Received packet with unexpected length: %d" SXLIB_LOG_EOL, rx_buffer_status.pld_len_in_bytes));
    }

    state->waiting_for_ack = false;
}


void sx126x_lr_fhss_ping_launch(sx126x_lr_fhss_ping_state_t* state)
{
    sxlib_Gpio_Led_on(state->config->interface->led_interface.led_tx);

    sx126x_status_t status = lr_fhss_send_packet(state, state->payload, state->payload_length);
    if (status != SX126X_STATUS_OK)
    {
        sxlib_Gpio_Led_off(state->config->interface->led_interface.led_tx);
        SXLIB_LOG(SXLIB_LOG_APP, ("Failed status=%d" SXLIB_LOG_EOL, status));
    }
}

void sx126x_lr_fhss_ping_init(sx126x_lr_fhss_ping_state_t* state, const sx126x_lr_fhss_ping_config_t* config)
{
    state->config = config;

    state->waiting_for_ack = false; // Added flag for ACK

    SXLIB_LOG(SXLIB_LOG_APP, ("SX126X-LR-FHSS Ping" SXLIB_LOG_EOL));

    sxlib_Gpio_Led_off(state->config->interface->led_interface.led_tx);
    sxlib_Gpio_Led_off(state->config->interface->led_interface.led_rx);

    // Transceiver has already been reset in main()

    uint32_t seed;
    sx126x_get_random_numbers(state->config->radio, &seed, 1);
    srand(seed);

    sx126x_set_dio_irq_params(state->config->radio, SX126X_IRQ_ALL, SX126X_IRQ_TX_DONE | SX126X_IRQ_LR_FHSS_HOP,
        SX126X_IRQ_NONE, SX126X_IRQ_NONE);

    // Initialize payload with random data
    for (int i = 0; i < sizeof(state->payload); ++i)
    {
        state->payload[i] = rand() % 256;
    }
    state->payload_length = MIN_PAYLOAD_LENGTH;
}

void sx126x_lr_fhss_ping_continue(sx126x_lr_fhss_ping_state_t* state)
{
    exit_sleep_mode(state);

    if (state->payload_length >= MAX_PAYLOAD_LENGTH)
    {
        state->payload_length = MIN_PAYLOAD_LENGTH;
    }
    else
    {
        state->payload_length += 1;
    }

    // Randomize payload for each transmission
    for (int i = 0; i < state->payload_length; ++i)
    {
        state->payload[i] = rand() % 256;
    }

    sx126x_lr_fhss_ping_launch(state);
}

void sx126x_lr_fhss_ping_handle_hop(sx126x_lr_fhss_ping_state_t* state)
{
    sx126x_lr_fhss_handle_hop(state->config->radio, &state->sx126x_lr_fhss_params, &state->lr_fhss_state);
    SXLIB_LOG(SXLIB_LOG_APP, ("Hopped." SXLIB_LOG_EOL));
}

void sx126x_lr_fhss_ping_handle_tx_done(sx126x_lr_fhss_ping_state_t* state)
{
		//end_time = clock(); // Record end time

    //elapsed_time = (double)(end_time - start_time) / CLOCKS_PER_SEC * 1000.0;
		
		//SXLIB_LOG(SXLIB_LOG_APP, (" Packet sent duration %.2f ms\n", (elapsed_time)));
		
    sx126x_lr_fhss_handle_tx_done(state->config->radio, &state->sx126x_lr_fhss_params, &state->lr_fhss_state);
    SXLIB_LOG(SXLIB_LOG_APP, ("LR-FHSS Packet sent!" SXLIB_LOG_EOL));
		
		
    sxlib_Gpio_Led_off(state->config->interface->led_interface.led_tx);

    SXLIB_LOG(SXLIB_LOG_APP, ("Listening for LoRa ACK" SXLIB_LOG_EOL));

    sx126x_status_t status = configure_lora_reception(state);
    if (status != SX126X_STATUS_OK) {
        SXLIB_LOG(SXLIB_LOG_APP, ("Failed to configure LoRa reception: %d" SXLIB_LOG_EOL, status));
        return;
    }

   SXLIB_LOG(SXLIB_LOG_APP, ("Entering Receive Mode" SXLIB_LOG_EOL));
   status = sx126x_set_rx(state->config->radio, 1);  // 10 seconds timeout (in ms)
    if (status != SX126X_STATUS_OK) {
       SXLIB_LOG(SXLIB_LOG_APP, ("Failed to enter Rx mode: %d" SXLIB_LOG_EOL, status));
    }

    // Set state to indicate we're waiting for ACK
    state->waiting_for_ack = true;
}

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTION DEFINITIONS --------------------------------------------
 */

static void set_standby_clear_all_irqs(const void* context)
{
    sx126x_set_standby(context, SX126X_STANDBY_CFG_RC);
    sx126x_comp_clear_and_unpend_irqs(context);
}

static void enter_standby_then_sleep_mode(const sx126x_lr_fhss_ping_state_t* state)
{
    set_standby_clear_all_irqs(state->config->radio);
#if defined CONFIG_ALLOW_SMTC_RADIO_SLEEP
    sx126x_set_sleep(state->config->radio, SX126X_SLEEP_CFG_WARM_START);
#endif
}

static void exit_sleep_mode(const sx126x_lr_fhss_ping_state_t* state)
{
#if defined CONFIG_ALLOW_SMTC_RADIO_SLEEP
    sx126x_wakeup(state->config->radio);
#endif
}

static sx126x_status_t lr_fhss_send_packet(sx126x_lr_fhss_ping_state_t* state, const uint8_t* payload,
    uint16_t payload_length)
{
	
	
  
		SXLIB_LOG(SXLIB_LOG_APP, ("Sending from Node A \n\n "));
    SXLIB_LOG(SXLIB_LOG_APP, ("Packet to send: "));
		state->packet_count+=1;
		SXLIB_LOG(SXLIB_LOG_APP, (" Packet Count %d  \n", (state->packet_count)));
    SXLIB_LOG_HEX(SXLIB_LOG_APP, "packet data", payload, payload_length);

    state->sx126x_lr_fhss_params.lr_fhss_params.modulation_type = LR_FHSS_V1_MODULATION_TYPE_GMSK_488;
    state->sx126x_lr_fhss_params.lr_fhss_params.cr = LR_FHSS_V1_CR_2_3;  // CR_2_3 or CR_1_3
    state->sx126x_lr_fhss_params.lr_fhss_params.grid = LR_FHSS_V1_GRID_25391_HZ; // 25391 or 3906
    state->sx126x_lr_fhss_params.lr_fhss_params.enable_hopping = true;
    state->sx126x_lr_fhss_params.lr_fhss_params.bw = LR_FHSS_V1_BW_1523438_HZ; // 1523438 or 136719
    state->sx126x_lr_fhss_params.lr_fhss_params.header_count = 2; //header count
    state->sx126x_lr_fhss_params.lr_fhss_params.sync_word = lr_fhss_sync_word;
    state->sx126x_lr_fhss_params.center_freq_in_pll_steps = sx126x_convert_freq_in_hz_to_pll_step(RF_FREQUENCY);
    state->sx126x_lr_fhss_params.device_offset = 0;
	
		SXLIB_LOG(SXLIB_LOG_APP, ("Parameters: %u " SXLIB_LOG_EOL ,state->sx126x_lr_fhss_params.lr_fhss_params.grid));

    sx126x_status_t status = sx126x_lr_fhss_init(state->config->radio, &state->sx126x_lr_fhss_params);
    if (status != SX126X_STATUS_OK)
    {
        return status;
    }
    const uint16_t hop_sequence_id = (rand() % 100) +1 ;
    SXLIB_LOG(SXLIB_LOG_APP, ("Using hop sequence ID: %u" SXLIB_LOG_EOL, hop_sequence_id)); // To print hop sequence ID


    sx126x_comp_set_tx_cfg(state->config->radio, POWER_IN_DBM, RF_FREQUENCY);

    sx126x_comp_set_tcxo_startup_time_in_steps(state->config->radio, 640 * 10);  // 10ms = 640


    lr_fhss_digest_t digest;
    lr_fhss_process_parameters(&state->sx126x_lr_fhss_params.lr_fhss_params, payload_length, &digest);

    SXLIB_LOG(SXLIB_LOG_APP, ("LR-FHSS Packet Parameters:\n"));
    SXLIB_LOG(SXLIB_LOG_APP, ("  Number of hops: %d\n", digest.nb_hops));
    SXLIB_LOG(SXLIB_LOG_APP, ("  Physical payload size in bytes: %d\n", digest.nb_bytes));
    SXLIB_LOG(SXLIB_LOG_APP, ("  Physical payload size in bits: %d\n", digest.nb_bits));

    float header_duration = 114.0 / 488.28125;
    float data_block_duration = 50.0 / 488.28125;
    int header_bits = 228;  // Two header blocks of 114 bits each
    int data_bits = digest.nb_bits - header_bits;
    int complete_data_blocks = data_bits / 50;
    int last_block_bits = data_bits % 50;

    SXLIB_LOG(SXLIB_LOG_APP, ("Timing Calculations:\n"));
    SXLIB_LOG(SXLIB_LOG_APP, ("  Header block duration: %.3f ms\n", header_duration * 1000));
    SXLIB_LOG(SXLIB_LOG_APP, ("  Complete data block duration: %.3f ms\n", data_block_duration * 1000));

		float last_block_duration=0;
    if (last_block_bits > 0) {
        last_block_duration = last_block_bits / 488.28125;
        SXLIB_LOG(SXLIB_LOG_APP, ("  Last block duration: %.3f ms\n", last_block_duration * 1000));
    }
    else {
        SXLIB_LOG(SXLIB_LOG_APP, ("  No partial last block\n"));
    }

    SXLIB_LOG(SXLIB_LOG_APP, ("Packet Structure:\n"));
    SXLIB_LOG(SXLIB_LOG_APP, ("  Total data bits: %d\n", data_bits));
    SXLIB_LOG(SXLIB_LOG_APP, ("  Complete data blocks: %d\n", complete_data_blocks));
    SXLIB_LOG(SXLIB_LOG_APP, ("  Last block bits: %d\n", last_block_bits));

    float total_duration = (2 * header_duration) + (complete_data_blocks * data_block_duration) + (last_block_bits / 488.28125);
    float avg_hop_duration = total_duration / digest.nb_hops;

    SXLIB_LOG(SXLIB_LOG_APP, ("Overall Timing:\n"));
    SXLIB_LOG(SXLIB_LOG_APP, ("  Total packet duration: %.3f ms\n", total_duration * 1000));
    SXLIB_LOG(SXLIB_LOG_APP, ("  Average hop duration: %.3f ms\n", avg_hop_duration * 1000));
		
		int TF=data_block_duration;  
	  
		int NF= complete_data_blocks;  
		
		float time_on_air =( state->sx126x_lr_fhss_params.lr_fhss_params.header_count * header_duration * 1000 )+ ((NF * TF )+ last_block_duration );
		SXLIB_LOG(SXLIB_LOG_APP, ("  Time on Air: %f\n",time_on_air ));

    SXLIB_LOG(SXLIB_LOG_APP, ("Hopping Sequence is:\n"));

    lr_fhss_hop_params_t hop_params;
    uint16_t initial_state;
    lr_fhss_status_t lr_status = lr_fhss_get_hop_params(&state->sx126x_lr_fhss_params.lr_fhss_params, &hop_params, &initial_state, hop_sequence_id);

    if (lr_status == LR_FHSS_STATUS_OK)
    {
        uint16_t lfsr_state = initial_state;
        uint32_t grid_hz = 25391;  // Correct grid value for US region
			  
			  int nodeElement=0;
			  struct Node* nodes = generateSequences(NO_NODES,NO_CARRIERS,HOPS);

        for (int i = 0; i < digest.nb_hops; i++)
        {
            int16_t freq_in_grid = lr_fhss_get_next_freq_in_grid(&lfsr_state, &hop_params, &state->sx126x_lr_fhss_params.lr_fhss_params);
              
            // Calculate actual frequency
							int32_t freq_offset = (int32_t)freq_in_grid + grid_hz;
            // Original code int32_t hop_freq_in_hz = RF_FREQUENCY + freq_offset;
					
					
					
						int32_t hop_freq_in_hz = (nodes[NODE].elements[nodeElement]) * grid_hz + RF_FREQUENCY;
						nodeElement+=1;
					
					
					
						 SXLIB_LOG(SXLIB_LOG_APP, (" Sub Carrier %d  \n", (nodes[NODE].elements[nodeElement])));
            // Convert to PLL steps
            uint32_t hop_freq_in_pll_steps = sx126x_convert_freq_in_hz_to_pll_step(hop_freq_in_hz);

            SXLIB_LOG(SXLIB_LOG_APP, ("  Hop %d: Grid index %d, Frequency %ld Hz (%u PLL steps)\n",
                i, freq_in_grid, hop_freq_in_hz, hop_freq_in_pll_steps));
        }
			
    }
    else
    {
        SXLIB_LOG(SXLIB_LOG_APP, ("  Failed to get hop parameters\n"));
    }


    status = sx126x_lr_fhss_build_frame(state->config->radio, &state->sx126x_lr_fhss_params, &state->lr_fhss_state,
        hop_sequence_id, payload, payload_length, 0);
    if (status != SX126X_STATUS_OK)
    {
        return status;
    }
 
    
    return sx126x_set_tx(state->config->radio, 0);
}




/* --- EOF ------------------------------------------------------------------ */
