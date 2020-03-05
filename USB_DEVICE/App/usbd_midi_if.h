/* USER CODE BEGIN Header */

/* USER CODE END Header */
#ifndef __USBD_MIDI_IF_H
#define __USBD_MIDI_IF_H

#ifdef __cplusplus
 extern "C" {
#endif
   
#include "../../Middlewares/ST/STM32_USB_Device_Library/Class/MIDI/Inc/usbd_midi.h"
#include "usbd_desc.h"

//Create NoteOn, NoteOff, transfer buffers
void USBD_AddNoteOn(uint8_t note, uint8_t vel);
void USBD_AddNoteOff(uint8_t note);
void USBD_SendMidiMessages(void);

#ifdef __cplusplus
}
#endif
#endif /* __USBD_MIDI_IF_H */
