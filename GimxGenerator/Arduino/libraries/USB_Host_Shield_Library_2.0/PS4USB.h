/* Copyright (C) 2014 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#ifndef _ps4usb_h_
#define _ps4usb_h_

#include "hiduniversal.h"
#include "PS4Parser.h"

#define PS4_VID         0x054C // Sony Corporation
#define PS4_PID         0x05C4 // PS4 Controller
#define PS4_PID_SLIM    0x09CC // PS4 Slim Controller

extern bool rx_flag;

/**
 * This class implements support for the PS4 controller via USB.
 * It uses the HIDUniversal class for all the USB communication.
 */
class PS4USB : public HIDUniversal, public PS4Parser {
public:
  /**
   * Constructor for the PS4USB class.
   * @param  p   Pointer to the USB class instance.
   */
  PS4USB(USB *p) :
  HIDUniversal(p) {
    PS4Parser::Reset();
  };

  /**
   * Used to check if a PS4 controller is connected.
   * @return Returns true if it is connected.
   */
  bool connected() {
    // Notify(HIDUniversal::PID);
    // D_PrintHex<uint8_t > (buf[0], 0x80);
    return HIDUniversal::isReady() && HIDUniversal::VID == PS4_VID && (HIDUniversal::PID == PS4_PID || HIDUniversal::PID == PS4_PID_SLIM);
    // return 1;

  };

  /**
   * Used to call your own function when the device is successfully initialized.
   * @param funcOnInit Function to call.
   */
  void attachOnInit(void (*funcOnInit)(void)) {
    pFuncOnInit = funcOnInit;
  };

protected:
  /** @name HIDUniversal implementation */
  /**
   * Used to parse USB HID data.
   * @param hid       Pointer to the HID class.
   * @param is_rpt_id Only used for Hubs.
   * @param len       The length of the incoming data.
   * @param buf       Pointer to the data buffer.
   */
  virtual void ParseHIDData(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) {
    if (HIDUniversal::VID == PS4_VID && (HIDUniversal::PID == PS4_PID || HIDUniversal::PID == PS4_PID_SLIM))
      PS4Parser::Parse(len, buf);
  };

  /**
   * Called when a device is successfully initialized.
   * Use attachOnInit(void (*funcOnInit)(void)) to call your own function.
   * This is useful for instance if you want to set the LEDs in a specific way.
   */
  virtual uint8_t OnInitSuccessful() {
    if (HIDUniversal::VID == PS4_VID && (HIDUniversal::PID == PS4_PID || HIDUniversal::PID == PS4_PID_SLIM)) {
      PS4Parser::Reset();
      if (pFuncOnInit)
        pFuncOnInit(); // Call the user function
      else
        setLed(Blue);
    };
    return 0;
  };
  /**@}*/


  virtual void receiveAuthResponse(uint16_t length, uint8_t *data) {
    uint8_t addr = bAddress;
    uint8_t ep = 0;
    uint8_t bmReqType = 0xa1;
    uint8_t bRequest = 1;
    uint8_t wValHi = 0x03;
    uint8_t wValLo = 0xf1;
    uint16_t wInd = 0;
    uint16_t total = 64;
    pUsb->ctrlReq(addr, ep, bmReqType, bRequest, wValLo, wValHi, wInd, total, length, data, NULL);
  }

  virtual void receiveAuthRequest(uint16_t length, uint8_t *data) {
    uint8_t addr = bAddress;
    uint8_t ep = 0;
    uint8_t bmReqType = 0xa1;
    uint8_t bRequest = 1;
    uint8_t wValHi = 0x03;
    uint8_t wValLo = 0xf2;
    uint16_t wInd = 0;
    uint16_t total = length;
    // uint16_t length = 16;
    pUsb->ctrlReq(addr, ep, bmReqType, bRequest, wValLo, wValHi, wInd, total, length, data, NULL);
    // return (datas[0] == 0xF2 && datas[1] == 0x01 && datas[2] == 0x00);
  }

  virtual void sendAuthRequest(uint16_t length, uint8_t *data) {

    uint8_t addr = bAddress;
    uint8_t ep = 0;
    uint8_t bmReqType = 0x21;
    uint8_t bRequest = 9;
    uint8_t wValHi = 0x03;
    uint8_t wValLo = 0xf0;
    uint16_t wInd = 0;
    // uint16_t total = 64;
    pUsb->ctrlReq(addr, ep, bmReqType, bRequest, wValLo, wValHi, wInd, length, length, data, NULL);
    // uint16_t nbytes = 64;
    // uint8_t dataptr;
    // USBReadParser *p
    // if (seq_num == 0) {
    //   // pUsb->outTransfer(bAddress, ep_number, sizeof(CHALLENGE_0), CHALLENGE_0);
    //   // USB_SETUP_HOST_TO_DEVICE?
    //   // USB::ctrlReq(uint8_t addr, uint8_t ep, uint8_t bmReqType, uint8_t bRequest, uint8_t wValLo, uint8_t wValHi,  uint16_t wInd, uint16_t total, uint16_t nbytes, uint8_t* dataptr, USBReadParser *p)
    //   // pUsb->ctrlReq(addr, ep, bmReqType, bRequest, wValLo, wValHi, wInd, total, sizeof(CHALLENGE_1), CHALLENGE_1, NULL);
    //   pUsb->ctrlReq(addr, ep, bmReqType, bRequest, wValLo, wValHi, wInd, total, sizeof(CHALLENGE_0), (uint8_t *) &CHALLENGE_0, NULL);
    // }
    // else if (seq_num == 1) {
    //   // pUsb->outTransfer(bAddress, ep_number, sizeof(CHALLENGE_1), CHALLENGE_1);
    //   pUsb->ctrlReq(addr, ep, bmReqType, bRequest, wValLo, wValHi, wInd, total, sizeof(CHALLENGE_1), (uint8_t *) &CHALLENGE_1, NULL);
    // }
    // else if (seq_num == 2) {
    //   // pUsb->outTransfer(bAddress, ep_number, sizeof(CHALLENGE_2), CHALLENGE_2);
    //   pUsb->ctrlReq(addr, ep, bmReqType, bRequest, wValLo, wValHi, wInd, total, sizeof(CHALLENGE_2), (uint8_t *) &CHALLENGE_2, NULL);
    // }
    // else if (seq_num == 3) {
    //   // pUsb->outTransfer(bAddress, ep_number, sizeof(CHALLENGE_3), CHALLENGE_3);
    //   pUsb->ctrlReq(addr, ep, bmReqType, bRequest, wValLo, wValHi, wInd, total, sizeof(CHALLENGE_3), (uint8_t *) &CHALLENGE_3, NULL);
    // }
    // else if (seq_num == 4) {
    //   // pUsb->outTransfer(bAddress, ep_number, sizeof(CHALLENGE_4), CHALLENGE_4);
    //   pUsb->ctrlReq(addr, ep, bmReqType, bRequest, wValLo, wValHi, wInd, total, sizeof(CHALLENGE_4), (uint8_t *) &CHALLENGE_4, NULL);
    // }

  }

  /** @name PS4Parser implementation */
  virtual void sendOutputReport(PS4Output *output) { // Source: https://github.com/chrippa/ds4drv
    uint8_t buf[32];
    memset(buf, 0, sizeof(buf));

    buf[0] = 0x05; // Report ID
    buf[1]= 0xFF;

    buf[4] = output->smallRumble; // Small Rumble
    buf[5] = output->bigRumble; // Big rumble

    buf[6] = output->r; // Red
    buf[7] = output->g; // Green
    buf[8] = output->b; // Blue

    buf[9] = output->flashOn; // Time to flash bright (255 = 2.5 seconds)
    buf[10] = output->flashOff; // Time to flash dark (255 = 2.5 seconds)

    output->reportChanged = false;

    // The PS4 console actually set the four last bytes to a CRC32 checksum, but it seems like it is actually not needed

    pUsb->outTransfer(bAddress, epInfo[ hidInterfaces[0].epIndex[epInterruptOutIndex] ].epAddr, sizeof(buf), buf);
  };
  /**@}*/

  /** @name USBDeviceConfig implementation */
  /**
   * Used by the USB core to check what this driver support.
   * @param  vid The device's VID.
   * @param  pid The device's PID.
   * @return     Returns true if the device's VID and PID matches this driver.
   */
  virtual bool VIDPIDOK(uint16_t vid, uint16_t pid) {
    return (vid == PS4_VID && (pid == PS4_PID || HIDUniversal::PID == PS4_PID_SLIM));
  };
  /**@}*/

private:
  void (*pFuncOnInit)(void); // Pointer to function called in onInit()
};
#endif
