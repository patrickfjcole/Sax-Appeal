/* USER CODE BEGIN Header */

/* USER CODE END Header */

#include "usbd_midi_if.h"
#include "stm32f4xx_hal.h"

#define NEXTBYTE(idx, mask) (mask & (idx + 1))

extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern USBD_HandleTypeDef hUsbDeviceFS;


//fill midi tx buffer
static uint16_t MIDI_DataTx(uint8_t *msg, uint16_t length)
{
  uint16_t i = 0;
  while (i < length) {
    APP_Rx_Buffer[APP_Rx_ptr_in] = *(msg + i);
    APP_Rx_ptr_in++;
    i++;
    if (APP_Rx_ptr_in == APP_RX_DATA_SIZE) {
      APP_Rx_ptr_in = 0;
    }
  }
  return USBD_OK;
}

void USBD_AddNoteOn(uint8_t note, uint8_t vel)
{
  uint8_t txbuf[4];
  
  txbuf[0] = 0x9;	// cable (<< 4) + 0x9
  txbuf[1] = 0x90;		// 0x90 | ch
  txbuf[2] = 0x7F & note;
  txbuf[3] = 0x7F & vel;
  MIDI_DataTx(txbuf, 4);
}

void USBD_AddNoteOff(uint8_t note)
{
  uint8_t txbuf[4];
  
  txbuf[0] = 0x8;	// cable (<< 4) + 0x8
  txbuf[1] = 0x80;		// 0x80 | ch
  txbuf[2] = 0x7F & note;
  txbuf[3] = 0;
  MIDI_DataTx(txbuf, 4);
}

void USBD_SendMidiMessages(void)
{
  if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
  {
    if (!USB_Tx_State)
      USBD_MIDI_SendPacket();
    else
      USB_Tx_State = USB_TX_CONTINUE;
  }
}


