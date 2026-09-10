#include <stdint.h>
#include "sl_component_catalog.h"
#include "simple_rail_assistance.h"
#include "rail.h"
#include "app_process.h"
#include "sl_simple_button_instances.h"
#include "sl_simple_led_instances.h"
#include "sl_flex_rail_package_assistant.h"
#include "sl_flex_rail_config.h"
#include "sl_flex_rail_channel_selector.h"
#include "sl_sleeptimer.h"
#include "sl_udelay.h"


#if defined(SL_CATALOG_KERNEL_PRESENT)
#include "app_task_init.h"
#endif

#include "rail_types.h"
#include "cmsis_compiler.h"

#if defined(SL_CATALOG_RAIL_SIMPLE_CPC_PRESENT)
#include "sl_rail_simple_cpc.h"
#endif

#define TX_PAYLOAD_LENGTH (16U)

typedef enum {
  S_PACKET_RECEIVED,
  S_PACKET_SENT,
  S_RX_PACKET_ERROR,
  S_TX_PACKET_ERROR,
  S_CALIBRATION_ERROR,
  S_IDLE,
} state_t;

volatile bool tx_requested = false;
volatile bool rx_requested = true;

static volatile state_t state = S_IDLE;
static volatile uint64_t error_code = 0;
static volatile RAIL_Status_t calibration_status = 0;

static __ALIGNED(RAIL_FIFO_ALIGNMENT) uint8_t rx_fifo[SL_FLEX_RAIL_RX_FIFO_SIZE];
static __ALIGNED(RAIL_FIFO_ALIGNMENT) uint8_t tx_fifo[SL_FLEX_RAIL_TX_FIFO_SIZE];

//  PAYLOAD
static uint8_t out_packet[TX_PAYLOAD_LENGTH] = {
  0x01, 0x16, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
  0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE,
};

static volatile bool packet_recieved = false;
static volatile bool packet_sent = false;
static volatile bool rx_error = false;
static volatile bool tx_error = false;
static volatile bool cal_error = false;

void app_process_action(RAIL_Handle_t rail_handle)
{
  RAIL_RxPacketHandle_t rx_packet_handle;
  RAIL_RxPacketInfo_t packet_info;
  RAIL_Status_t rail_status = RAIL_STATUS_NO_ERROR;
  RAIL_Status_t calibration_status_buff = RAIL_STATUS_NO_ERROR;
  #if defined(SL_CATALOG_RAIL_SIMPLE_CPC_PRESENT)
  uint8_t success_sent = 0x01;
  #endif

  if (packet_recieved) {
    packet_recieved = false;
    state = S_PACKET_RECEIVED;
  } else if (packet_sent) {
    packet_sent = false;
    state = S_PACKET_SENT;
  } else if (rx_error) {
    rx_error = false;
    state = S_RX_PACKET_ERROR;
  } else if (tx_error) {
    tx_error = false;
    state = S_TX_PACKET_ERROR;
  } else if (cal_error) {
    cal_error = false;
    state = S_CALIBRATION_ERROR;
  }

  switch (state) {
    case S_PACKET_RECEIVED:
      rx_packet_handle = RAIL_GetRxPacketInfo(rail_handle, RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &packet_info);
      while (rx_packet_handle != RAIL_RX_PACKET_HANDLE_INVALID) {
        uint8_t *start_of_packet = 0;
        uint16_t packet_size = unpack_packet(rx_fifo, &packet_info, &start_of_packet);

        if (packet_size > 0 && start_of_packet != NULL) {
          if (start_of_packet[0] == 0x01) {
            sl_led_toggle(&sl_led_led0);
            sl_udelay_wait(500000U);
            sl_led_toggle(&sl_led_led0);

          }
        }

        rail_status = RAIL_ReleaseRxPacket(rail_handle, rx_packet_handle);
        if (rail_status != RAIL_STATUS_NO_ERROR) {
          app_log_warning("RAIL_ReleaseRxPacket() result:%d", rail_status);
        }
        if (rx_requested) {
          printf_rx_packet(start_of_packet, packet_size);
#if defined(SL_CATALOG_RAIL_SIMPLE_CPC_PRESENT)
          sl_rail_simple_cpc_transmit(packet_size, start_of_packet);
#endif
        }
        rx_packet_handle = RAIL_GetRxPacketInfo(rail_handle, RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE, &packet_info);
      }
      state = S_IDLE;
      break;

    case S_PACKET_SENT:
     app_log_info("Packet has been sent\n");
#if defined(SL_CATALOG_RAIL_SIMPLE_CPC_PRESENT)
      sl_rail_simple_cpc_transmit(1, &success_sent);
#endif

      state = S_IDLE;
      break;

    case S_RX_PACKET_ERROR:
      app_log_error("Radio RX Error occurred\nEvents: %llX\n", error_code);
      state = S_IDLE;
      break;

    case S_TX_PACKET_ERROR:
      app_log_error("Radio TX Error occurred\nEvents: %llX\n", error_code);
      state = S_IDLE;
      break;

    case S_IDLE:
      if (tx_requested) {
        prepare_package(rail_handle, out_packet, sizeof(out_packet));
        rail_status = RAIL_StartTx(rail_handle, 1, RAIL_TX_OPTIONS_DEFAULT, NULL);
        if (rail_status != RAIL_STATUS_NO_ERROR) {
          app_log_warning("RAIL_StartTx() result:%d ", rail_status);
        }
        tx_requested = false;
      }
      break;

    case S_CALIBRATION_ERROR:
      calibration_status_buff = calibration_status;
      app_log_error("Radio Calibration Error occurred\nEvents: %llX\nRAIL_Calibrate() result:%d\n",
                    error_code,
                    calibration_status_buff);
      state = S_IDLE;
      break;

    default:
      app_log_error("Unexpected Simple TRX state occurred:%d\n", state);
      break;
  }
}

void sl_rail_util_on_event(RAIL_Handle_t rail_handle, RAIL_Events_t events)
{
  error_code = events;
  if ( events & RAIL_EVENTS_RX_COMPLETION ) {
    if (events & RAIL_EVENT_RX_PACKET_RECEIVED) {
      RAIL_HoldRxPacket(rail_handle);
      packet_recieved = true;
    } else {
      rx_error = true;
    }
  }
  if ( events & RAIL_EVENTS_TX_COMPLETION) {
    if (events & RAIL_EVENT_TX_PACKET_SENT) {
      packet_sent = true;
    } else {
      tx_error = true;
    }
  }

  if ( events & RAIL_EVENT_CAL_NEEDED ) {
    calibration_status = RAIL_Calibrate(rail_handle, NULL, RAIL_CAL_ALL_PENDING);
    if (calibration_status != RAIL_STATUS_NO_ERROR) {
      cal_error = true;
    }
  }
#if defined(SL_CATALOG_KERNEL_PRESENT)
  app_task_notify();
#endif
}

void sl_button_on_change(const sl_button_t *handle)
{
  if (sl_button_get_state(handle) == SL_SIMPLE_BUTTON_PRESSED) {
    tx_requested = true;
  }
#if defined(SL_CATALOG_KERNEL_PRESENT)
  app_task_notify();
#endif
}

#if defined(SL_CATALOG_RAIL_SIMPLE_CPC_PRESENT)
void sl_rail_simple_cpc_receive_cb(sl_status_t status, uint32_t len, uint8_t *data)
{
  if (status == SL_STATUS_OK) {
    if (len == 1) {
      if (data[0] == 0x01 || data[0] == '1') {
        tx_requested = true;
      }
      if (data[0] == 0x00 || data[0] == '0') {
        rx_requested = !rx_requested;
      }
    }
  }
}
#endif

void set_up_tx_fifo(RAIL_Handle_t rail_handle)
{
  uint16_t allocated_tx_fifo_size = 0;
  allocated_tx_fifo_size = RAIL_SetTxFifo(rail_handle, tx_fifo, 0, SL_FLEX_RAIL_TX_FIFO_SIZE);
  app_assert(allocated_tx_fifo_size == SL_FLEX_RAIL_TX_FIFO_SIZE,
             "RAIL_SetTxFifo() failed to allocate a large enough fifo (%d bytes instead of %d bytes)\n",
             allocated_tx_fifo_size,
             SL_FLEX_RAIL_TX_FIFO_SIZE);
}
