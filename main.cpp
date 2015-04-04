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

DigitalOut status_led(P0_5);
DigitalOut data_led(P0_4);

bool connected = false;

BLEDevice   ble;
UARTService *bleUart;
Serial      serial(p9, p11);  // tx, rx

#define BUFFER_SIZE 128
uint8_t bleToUARTBuffer[BUFFER_SIZE];

unsigned long long writeBufferPtr;
unsigned long long readBufferPtr;

Timer bleToUARTBufferTimer;
int lastWrittenToBuffer;

void disconnectionCallback(Gap::Handle_t handle, Gap::DisconnectionReason_t reason);
void connectionCallback(Gap::Handle_t handle, Gap::addr_type_t peerAddrType, const Gap::address_t peerAddr, const Gap::ConnectionParams_t *params);
void onDataWritten(const GattCharacteristicWriteCBParams *params);
void onSerialReceived();
void stateLed_function();


int main(void)
{
    /*************************************************************************/
    /* INITIALIZE BLE                                                        */
    /*************************************************************************/

    ble.init();
    ble.onDisconnection(disconnectionCallback);
    ble.onConnection(connectionCallback);
    ble.onDataWritten(onDataWritten);

    /* setup advertising */
    ble.accumulateAdvertisingPayload(GapAdvertisingData::BREDR_NOT_SUPPORTED);
    ble.setAdvertisingType(GapAdvertisingParams::ADV_CONNECTABLE_UNDIRECTED);
    ble.accumulateAdvertisingPayload(GapAdvertisingData::SHORTENED_LOCAL_NAME,
                                     (const uint8_t *)"Snake Robot UART", sizeof("Snake Robot UART") - 1);
    ble.accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS,
                                     (const uint8_t *)UARTServiceUUID_reversed, sizeof(UARTServiceUUID_reversed));
    
    ble.setDeviceName("Snake Robot");

    ble.setAdvertisingInterval(Gap::MSEC_TO_ADVERTISEMENT_DURATION_UNITS(1000));
    ble.startAdvertising();
    
    Ticker stateLed_ticker;
    stateLed_ticker.attach(stateLed_function, 1);






    /*************************************************************************/
    /* INITIALIZE BLE UART SERVICE                                           */
    /*************************************************************************/

    bleToUARTBufferTimer.start();
    lastWrittenToBuffer = bleToUARTBufferTimer.read_us();
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
    {
        // Check if new data is available in the buffer
        if (writeBufferPtr > readBufferPtr)
        {
            // Send buffer if
            if (((bleToUARTBufferTimer.read_us() - lastWrittenToBuffer) > 1000) || // 1000 microseconds passed since last buffer update
                ((writeBufferPtr - readBufferPtr) >= 6))                           // or 6 bytes are written into the buffer
            {
                // Send buffer
                while (readBufferPtr < writeBufferPtr)
                    serial.putc(bleToUARTBuffer[(readBufferPtr++)%BUFFER_SIZE]);
            }
        }
    }
}


/** Periodically called to drive the state led. */
void stateLed_function()
{
    if (connected)
        status_led = 1;
    else
        status_led = !status_led;
        
    data_led = 0;
}


/** Called when a Bluetooth device did disconnect. */
void disconnectionCallback(Gap::Handle_t handle, Gap::DisconnectionReason_t reason)
{
    // Restart advertising
    ble.startAdvertising();
    
    connected = false;
}


/** Called when a Bluetooth device did connect. */
void connectionCallback(Gap::Handle_t handle, Gap::addr_type_t peerAddrType, const Gap::address_t peerAddr, const Gap::ConnectionParams_t *params)
{
    connected = true;
}


/** Called when a Bluetooth device sent data. (BLE to UART) */
void onDataWritten(const GattCharacteristicWriteCBParams *params)
{
    data_led = !data_led;
    
    if ((bleUart != NULL) && (params->charHandle == bleUart->getTXCharacteristicHandle()))
    {
        // Write received data to serial port
        for (int i = 0; i < params->len; i++)
            bleToUARTBuffer[(writeBufferPtr++)%BUFFER_SIZE] = params->data[i];
        
        lastWrittenToBuffer = bleToUARTBufferTimer.read_us();
    }
}


/** Called when the serial device sent a byte. (UART to BLE)*/
void onSerialReceived()
{
    data_led = !data_led;
    
    uint8_t c = serial.getc();
    ble.updateCharacteristicValue(bleUart->getRXCharacteristicHandle(), &c, sizeof(c));
}
