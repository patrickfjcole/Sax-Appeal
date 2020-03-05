/* USER CODE BEGIN Header */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "../../MIDI/Inc/usbd_midi.h"
#include "usbd_desc.h"
#include "stm32f4xx_hal_conf.h"
#include "usbd_ctlreq.h"
#include "stm32f4xx_hal.h"
#include "../../AUDIO/Inc/usbd_audio.h"

static uint8_t  USBD_MIDI_Init (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t  USBD_MIDI_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t  USBD_MIDI_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t  USBD_MIDI_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum);

static uint8_t  *USBD_MIDI_GetCfgDesc (uint16_t *length);

USBD_HandleTypeDef *pInstance = NULL; 

uint32_t APP_Rx_ptr_in  = 0;
uint32_t APP_Rx_ptr_out = 0;
uint32_t APP_Rx_length  = 0;
uint8_t  USB_Tx_State = 0;

__ALIGN_BEGIN uint8_t USB_Rx_Buffer[MIDI_DATA_OUT_PACKET_SIZE] __ALIGN_END ;
__ALIGN_BEGIN uint8_t APP_Rx_Buffer[APP_RX_DATA_SIZE] __ALIGN_END ;

/* USB MIDI interface class callbacks structure */
USBD_ClassTypeDef USBD_MIDI =
{
  USBD_MIDI_Init,
  USBD_MIDI_DeInit,
  NULL,
  NULL,
  NULL,
  USBD_MIDI_DataIn,
  USBD_MIDI_DataOut,
  NULL,
  NULL,
  NULL,
  NULL,// HS
  USBD_MIDI_GetCfgDesc,// FS
  NULL,// OTHER SPEED
  NULL,// DEVICE_QUALIFIER
};

/* USB MIDI device Configuration Descriptor, pg. 264 of specs */
__ALIGN_BEGIN uint8_t USBD_MIDI_CfgDesc[USB_MIDI_CONFIG_DESC_SIZ] __ALIGN_END =
{
  // Configuration Descriptor
  0x09,									// bLength
  0x02,									// bDescriptorType: Configuration
  LOBYTE(USB_MIDI_CONFIG_DESC_SIZ),		// wTotalLength: 101 bytes
  HIBYTE(USB_MIDI_CONFIG_DESC_SIZ),
  0x02,									// bNumInterfaces: 2 (AudioControl & MIDIStreaming)
  0x01,									// bConfigurationValue: Configuration value so SetConfiguration(non-zero)
  0x00,									// iConfiguration: Index of string descriptor describing the configuration
  0xC0,									// bmAttributes: Bitmap, MSB always 1
  0x32,									// MaxPower 0x32 == 50, unit is 2mA so 0x32 means 100mA

  // Standard AC Interface Descriptor
  AUDIO_INTERFACE_DESC_SIZE,			// bLength: Interface Descriptor size
  USB_DESC_TYPE_INTERFACE,				// bDescriptorType: Interface
  0x00,									// bInterfaceNumber: Number of Interface, 0 indexed
  0x00,									// bAlternateSetting: Alternate setting
  0x00,									// bNumEndpoints: 0 (defined by USB-MIDI)
  USB_DEVICE_CLASS_AUDIO,				// bInterfaceClass: Audio
  AUDIO_SUBCLASS_AUDIOCONTROL,			// bInterfaceSubClass: AudioControl
  0x00,									// bInterfaceProtocol: Unused (defined by USB-MIDI)
  0x00,									// iInterface: Unused (defined by USB-MIDI)

  // Class-specific AC Interface Descriptor
  AUDIO_INTERFACE_DESC_SIZE,			// bLength: 0x09
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,		// bDescriptorType: CS_interface_type from audio.h
  AUDIO_CONTROL_HEADER,					// bDescriptorSubtype: HEADER subtype
  0x00,									// bcdADC: Audio device class spec revision (1.0 defined by USB-MIDI 1.0)
  0x01,									// 0x0100
  0x09,									// wTotalLength (defined by USB-MIDI)
  0x00,									// 0x0009
  0x01,									// bInCollection: Number of streaming interfaces (1 as defined by USB-MIDI for MIDIStreaming)
  0x01,									// baInterfaceNr: MIDIStreaming interface 1 (defined by USB-MIDI)

  // MIDIStreaming Interface Descriptors
  AUDIO_INTERFACE_DESC_SIZE,			// bLength: 0x09 (same as Standard AC Interface Descriptor)
  USB_DESC_TYPE_INTERFACE,              // bDescriptorType: Interface Descriptor
  0x01,									// bInterfaceNumber: index of this interface, 0 indexed
  0x00,									// bAlternateSetting: Alternate setting 0x00 (defined by USB-MIDI)
  0x02,									// bNumEndpoints: 2 (defined by USB-MIDI)
  USB_DEVICE_CLASS_AUDIO,				// bInterfaceClass: Audio
  AUDIO_SUBCLASS_MIDISTREAMING,			// bInterfaceSubClass: 0x03 (defined by USB-MIDI) AUDIO_SUBCLASS_MIDISTREAMING (self defined in audio.h)
  0x00,									// bInterfaceProtocol: Unused (defined by USB-MIDI)
  0x00,									// iInterface: Unused (defined by USB-MIDI)

  // Class-Specific MS Interface Descriptor (HEADER), all values defined by USB-MIDI
  0x07,									// bLength
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,		// bDescriptorType: CS_interface_type from audio.h
  0x01, 								// bDescriptorSubtype: MS HEADER subtype
  0x00,									// bcdADC: Audio device class spec revision (1.0 defined by USB-MIDI 1.0)
  0x01,									// 0x0100
  0x41,									// wTotalLength: Total size of class specific descriptors (not including AC header)
  0x00,									// 0x0041 (58)

  // MIDI IN Jack Descriptor (defined by USB-MIDI)
  // MIDI Adapter MIDI IN Jack Descriptor (Embedded)
  0x06,									// bLength
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,		// bDescriptorType: CS_interface_type from audio.h
  0x02,									// bDescriptorSubtype: MIDI_IN_JACK subtype
  0x01,									// bJackType: EMBEDDED
  0x01,									// bJackID
  0x00,									// iJack: Unused
  // MIDI Adapter MIDI IN Jack Descriptor (External)
  0x06,									// bLength
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,		// bDescriptorType: CS_interface_type from audio.h
  0x02,									// bDescriptorSubtype: MIDI_IN_JACK subtype
  0x02,									// bJackType: EXTERNAL
  0x02,									// bJackID
  0x00,									// iJack: Unused

  // MIDI OUT Jack Descriptor (defined by USB-MIDI)
  // MIDI Adapter MIDI OUT Jack Descriptor (Embedded)
  0x09,									// bLength
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,		// bDescriptorType: CS_interface_type from audio.h
  0x03,									// bDescriptorSubtype: MIDI_OUT_JACK subtype
  0x01,									// bJackType: EMBEDDED
  0x03,									// bJackID
  0x01,									// bNrInputPins: Number of input pins
  0x02,									// BaSourceID(1): ID of the entity to which this pin is connected
  0x01,									// BaSourcePin(1): Output Pin number of the entity to which this Input Pin is connected
  0x00,									// iJack: Unused
  // MIDI Adapter MIDI OUT Jack Descriptor (External)
  0x09,									// bLength
  AUDIO_INTERFACE_DESCRIPTOR_TYPE,		// bDescriptorType: CS_interface_type from audio.h
  0x03,									// bDescriptorSubtype: MIDI_OUT_JACK subtype
  0x02,									// bJackType: EXTERNAL
  0x04,									// bJackID
  0x01,									// bNrInputPins: Number of input pins
  0x01,									// BaSourceID(1): ID of the entity to which this pin is connected
  0x01,									// BaSourcePin(1): Output Pin number of the entity to which this Input Pin is connected
  0x00,									// iJack: Unused

  // Bulk OUT Endpoint Descriptor (defined by USB-MIDI)
  // Standard Bulk Out Endpoint Descriptor
  0x09,									// bLength
  USB_DESC_TYPE_ENDPOINT,               // bDescriptorType: ENDPOINT descriptor
  MIDI_OUT_EP,							// bEndpointAddress: OUT Endpoint 1 (usbd_midi.h)
  0x02,									// bmAttributes: Bulk, not shared
  LOBYTE(MIDI_DATA_FS_MAX_PACKET_SIZE), // wMaxPacketSize: 64 Bytes per packet
  HIBYTE(MIDI_DATA_FS_MAX_PACKET_SIZE),
  0x00,									// bInterval: ignore for Bulk transfer
  0x00,									// bRefresh: Unused
  0x00,									// bSynchAddress: Unused
  // Class-specific MS Bulk Out Endpoint Descriptor
  0x05,									// bLength
  0x25,									// bDescriptorType: CS ENDPOINT descriptor
  0x01,									// bDescriptorSubtype: MS_GENERAL subtype
  0x01,									// bNumEmbMIDIJack: Number of embedded MIDI IN Jacks
  0x01,									// BaAssocJackID(1): ID of the embedded MIDI IN Jack

  // Bulk IN Endpoint Descriptor (defined by USB-MIDI)
  // Standard Bulk In Endpoint Descriptor
  0x09, 								// bLength
  USB_DESC_TYPE_ENDPOINT,               // bDescriptorType: ENDPOINT descriptor
  MIDI_IN_EP, 							// bEndpointAddress: IN Endpoint 1 (usbd_midi.h)
  0x02,									// bmAttributes: Bulk, not shared
  LOBYTE(MIDI_DATA_FS_MAX_PACKET_SIZE), // wMaxPacketSize: 64 Bytes per packet
  HIBYTE(MIDI_DATA_FS_MAX_PACKET_SIZE),
  0x00,									// bInterval: ignore for Bulk transfer
  0x00,									// bRefresh: Unused
  0x00,									// bSynchAddress: Unused
  // Class-specific MS Bulk In Endpoint Descriptor
  0x05,									// bLength
  0x25,									// bDescriptorType: CS ENDPOINT descriptor
  0x01,									// bDescriptorSubtype: MS_GENERAL subtype
  0x01,									// bNumEmbMIDIJack: Number of embedded MIDI OUT Jacks
  0x03,									// BaAssocJackID(1): ID of the embedded MIDI OUT Jack
};


static uint8_t USBD_MIDI_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx){
  pInstance = pdev;
  USBD_LL_OpenEP(pdev,MIDI_IN_EP,USBD_EP_TYPE_BULK,MIDI_DATA_IN_PACKET_SIZE);
  USBD_LL_OpenEP(pdev,MIDI_OUT_EP,USBD_EP_TYPE_BULK,MIDI_DATA_OUT_PACKET_SIZE);
  USBD_LL_PrepareReceive(pdev,MIDI_OUT_EP,(uint8_t*)(USB_Rx_Buffer),MIDI_DATA_OUT_PACKET_SIZE);
  return 0;
}

static uint8_t USBD_MIDI_DeInit (USBD_HandleTypeDef *pdev, uint8_t cfgidx){
  pInstance = NULL;
  USBD_LL_CloseEP(pdev,MIDI_IN_EP);
  USBD_LL_CloseEP(pdev,MIDI_OUT_EP);
  return 0;
}

static uint8_t USBD_MIDI_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
  if (APP_Rx_length != 0)
  {
    USB_Tx_State = USB_TX_CONTINUE;
  }
  else
  {
    APP_Rx_ptr_out = 0;
    APP_Rx_ptr_in = 0;
    USB_Tx_State = USB_TX_READY;
  }
  return USBD_OK;
}

static uint8_t USBD_MIDI_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{      
  uint16_t USB_Rx_Cnt;
  USBD_MIDI_ItfTypeDef *pmidi;
  
  USB_Rx_Cnt = ((PCD_HandleTypeDef*)pdev->pData)->OUT_ep[epnum].xfer_count;
  if (USB_Rx_Cnt) //
  {
    pmidi = (USBD_MIDI_ItfTypeDef *)(pdev->pUserData);    
    pmidi->pIf_MidiRx((uint8_t *)&USB_Rx_Buffer, USB_Rx_Cnt);
  }
  USBD_LL_PrepareReceive(pdev,MIDI_OUT_EP,(uint8_t*)(USB_Rx_Buffer),MIDI_DATA_OUT_PACKET_SIZE);
  return USBD_OK;
}

static uint8_t *USBD_MIDI_GetCfgDesc (uint16_t *length){
  *length = sizeof (USBD_MIDI_CfgDesc);
  return USBD_MIDI_CfgDesc;
}

//MIDI TX START FUNCTION
void USBD_MIDI_SendPacket()
{
  uint16_t USB_Tx_ptr;
  uint16_t USB_Tx_length;
  
    if (APP_Rx_ptr_out == APP_RX_DATA_SIZE)
    {
      APP_Rx_ptr_out = 0;
    }

    if(APP_Rx_ptr_out == APP_Rx_ptr_in)
    {
      USB_Tx_State = USB_TX_READY;
      return;
    }

    if(APP_Rx_ptr_out > APP_Rx_ptr_in)
    {
      APP_Rx_length = APP_RX_DATA_SIZE - APP_Rx_ptr_out;
    }
    else
    {
      APP_Rx_length = APP_Rx_ptr_in - APP_Rx_ptr_out;
    }

    if (APP_Rx_length > MIDI_DATA_IN_PACKET_SIZE)
    {
      USB_Tx_ptr = APP_Rx_ptr_out;
      USB_Tx_length = MIDI_DATA_IN_PACKET_SIZE;
      APP_Rx_ptr_out += MIDI_DATA_IN_PACKET_SIZE;
      APP_Rx_length -= MIDI_DATA_IN_PACKET_SIZE;
    }
    else
    {
      USB_Tx_ptr = APP_Rx_ptr_out;
      USB_Tx_length = APP_Rx_length;
      APP_Rx_ptr_out += APP_Rx_length;
      APP_Rx_length = 0;
    }
    USB_Tx_State = USB_TX_BUSY;
    
    USBD_LL_Transmit(pInstance, MIDI_IN_EP,&APP_Rx_Buffer[USB_Tx_ptr],USB_Tx_length);
}
