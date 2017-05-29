#include "VCP.h"
#include "stm32f4xx_conf.h"
#include "usbd_cdc_core.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"

#define USB_VCP_RECEIVE_BUFFER_LENGTH 128

typedef enum {
    TM_USB_VCP_OK,                  /*!< Everything ok */
    TM_USB_VCP_ERROR,               /*!< An error occurred */
    TM_USB_VCP_RECEIVE_BUFFER_FULL, /*!< Receive buffer is full */
    TM_USB_VCP_DATA_OK,             /*!< Data OK */
    TM_USB_VCP_DATA_EMPTY,          /*!< Data empty */
    TM_USB_VCP_NOT_CONNECTED,       /*!< Not connected to PC */
    TM_USB_VCP_CONNECTED,           /*!< Connected to PC */
    TM_USB_VCP_DEVICE_SUSPENDED,    /*!< Device is suspended */
    TM_USB_VCP_DEVICE_RESUMED       /*!< Device is resumed */
} TM_USB_VCP_Result;

VCP::VCP(serial_configuration_t *config)
{
  if (config->serial_type == Serial::VCP)
  {
    USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb);
  }
}

bool VCP::bytes_waiting(){
  return true;
}

uint8_t VCP::read_byte(){
  return 0;
}

void VCP::put_byte(uint8_t *ch, uint32_t len)
{
  return;
}

