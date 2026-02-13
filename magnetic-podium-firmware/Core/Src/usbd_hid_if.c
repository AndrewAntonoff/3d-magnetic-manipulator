// usbd_custom_hid_if.c
#include "usbd_custom_hid_if.h"

// HID Report Descriptor для 6 осей (X, Y, Z, Rx, Ry, Rz) и 32 кнопок
__ALIGN_BEGIN static uint8_t REPORT_DESCRIPTOR[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END = {
    0x05, 0x01,                    // Usage Page (Generic Desktop)
    0x09, 0x04,                    // Usage (Joystick) или 0x05 (Game Pad)
    0xA1, 0x01,                    // Collection (Application)
    0x85, 0x01,                    //   Report ID (1) - если нужен ID

    // Оси X, Y, Z, Rx, Ry, Rz (Signed 16-bit)
    0x09, 0x30,                    //   Usage (X)
    0x09, 0x31,                    //   Usage (Y)
    0x09, 0x32,                    //   Usage (Z)
    0x09, 0x33,                    //   Usage (Rx) - или Rz
    0x09, 0x34,                    //   Usage (Ry)
    0x09, 0x35,                    //   Usage (Rz) - или Slider если нужно 7 осей
    0x16, 0x01, 0x80,              //   Logical Minimum (-32767) // Signed 16-bit
    0x26, 0xFF, 0x7F,              //   Logical Maximum (32767)  // Signed 16-bit
    0x75, 0x10,                    //   Report Size (16)
    0x95, 0x06,                    //   Report Count (6)
    0x81, 0x02,                    //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

    // Кнопки (32 шт.) - если нужны
    0x05, 0x09,                    //   Usage Page (Button)
    0x19, 0x01,                    //   Usage Minimum (Button 1)
    0x29, 0x20,                    //   Usage Maximum (Button 32)
    0x15, 0x00,                    //   Logical Minimum (0)
    0x25, 0x01,                    //   Logical Maximum (1)
    0x75, 0x01,                    //   Report Size (1)
    0x95, 0x20,                    //   Report Count (32)
    0x81, 0x02,                    //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

    0xC0,                          // End Collection
};

uint8_t USBD_CUSTOM_HID_GetHidDescriptor(USBD_HandleTypeDef *pdev, uint8_t **pbuf)
{
    *pbuf = REPORT_DESCRIPTOR;
    return USBD_CUSTOM_HID_REPORT_DESC_SIZE;
}

uint8_t USBD_CUSTOM_HID_GetReportDescriptor(USBD_HandleTypeDef *pdev, uint8_t **pbuf)
{
    *pbuf = REPORT_DESCRIPTOR;
    return USBD_CUSTOM_HID_REPORT_DESC_SIZE;
}
