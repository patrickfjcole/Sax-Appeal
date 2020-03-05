/* USER CODE BEGIN Header */

/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_MIDI_H
#define __USB_MIDI_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

#define MIDI_IN_EP                                   0x81  	/* EP1 for data IN */
#define MIDI_OUT_EP                                  0x01  	/* EP1 for data OUT */
#define MIDI_DATA_FS_MAX_PACKET_SIZE                 64U  	/* Endpoint IN & OUT Packet size */
#define MIDI_CMD_PACKET_SIZE                         8U  	/* Control Endpoint Packet size */
#define USB_MIDI_CONFIG_DESC_SIZ                     101U 	// Total Length of config_desc
#define MIDI_DATA_IN_PACKET_SIZE                     MIDI_DATA_FS_MAX_PACKET_SIZE
#define MIDI_DATA_OUT_PACKET_SIZE                    MIDI_DATA_FS_MAX_PACKET_SIZE
#define APP_RX_DATA_SIZE                             (MIDI_DATA_FS_MAX_PACKET_SIZE * 32) //buffer to send

#define MIDI_IN_FRAME_INTERVAL		             1
   
#define USB_TX_READY            0       //ready for transfer
#define USB_TX_BUSY             1       //busy
#define USB_TX_CONTINUE         2       //ready to continue tx next packet (64 bit)
   
typedef struct _USBD_MIDI_ItfTypeDef{
  uint16_t (*pIf_MidiRx)    (uint8_t *msg, uint16_t length);
  //uint16_t (*pIf_MidiTx)    (uint8_t *msg, uint16_t length);
}USBD_MIDI_ItfTypeDef;


extern uint8_t APP_Rx_Buffer   [APP_RX_DATA_SIZE];
extern uint32_t APP_Rx_ptr_in;
extern uint32_t APP_Rx_ptr_out;
extern uint32_t APP_Rx_length;
extern uint8_t  USB_Tx_State;

extern USBD_ClassTypeDef  USBD_MIDI;
#define USBD_MIDI_CLASS    &USBD_MIDI


void USBD_MIDI_SendPacket ();

#ifdef __cplusplus
}
#endif

#endif  /* __USB_MIDI_H */
