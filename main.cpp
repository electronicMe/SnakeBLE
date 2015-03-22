//
//    main.cpp
//    BLE UART Bridge
//
//    Author: Sebastian Mach
//    Created on Mar. 20 2015
//

#include "mbed.h"
#include "BLEDevice.h"
#include "UARTService.h"

BLEDevice   ble;
UARTService *bleUart;
Serial      serial(p9, p11);  // tx, rx


void disconnectionCallback(Gap::Handle_t handle, Gap::DisconnectionReason_t reason);
void onDataWritten(const GattCharacteristicWriteCBParams *params);
void onSerialReceived();


int main(void)
{
    /*************************************************************************/
    /* INITIALIZE BLE                                                        */
    /*************************************************************************/

    ble.init();
    ble.onDisconnection(disconnectionCallback);
    ble.onDataWritten(onDataWritten);

    /* setup advertising */
    ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED);
    ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.accumulateAdvertisingPayload(GapAdvertisingData::SHORTENED_LOCAL_NAME,
                                     (const uint8_t *)"Snake Robot", sizeof("Snake Robot") - 1);
    ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS,
                                     (const uint8_t *)UARTServiceUUID_reversed, sizeof(UARTServiceUUID_reversed));
    
    ble.setDeviceName("Snake Robot");

    ble.setAdvertisingInterval(Gap::MSEC_TO_ADVERTISEMENT_DURATION_UNITS(1000));
    ble.startAdvertising();






    /*************************************************************************/
    /* INITIALIZE BLE UART SERVICE                                           */
    /*************************************************************************/

    bleUart = new UARTService(ble);






    /*************************************************************************/
    /* INITIALIZE UART CONNECTION                                            */
    /*************************************************************************/

    serial.baud(115200);
    serial.attach(&onSerialReceived);






    /*************************************************************************/
    /* MAIN LOOP                                                             */
    /*************************************************************************/

    while (true)
        ble.waitForEvent();
}


/** Called when a Bluetooth device did disconnect. */
void disconnectionCallback(Gap::Handle_t handle, Gap::DisconnectionReason_t reason)
{
    // Restart advertising
    ble.startAdvertising();
}


/** Called when a Bluetooth device did send data */
void onDataWritten(const GattCharacteristicWriteCBParams *params)
{
    if ((bleUart != NULL) && (params->charHandle == bleUart->getTXCharacteristicHandle())) {
        // Send received data to serial port
        serial.printf("%s", params->data);
    }
}


/** Called when the serial device sent a byte. */
void onSerialReceived()
{
    uint8_t c = serial.getc();
    ble.updateCharacteristicValue(bleUart->getRXCharacteristicHandle(), &c, 1);
}
