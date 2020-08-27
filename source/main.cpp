/* The central example is based of the mbed Microcontroller Library
 * Copyright (c) 2006-2015 ARM Limited
 *
 * This central example was developed by Gerrikoio (c) August 2020
 * It is designed to work with the BLE Peripheral example found on
 * Gerriko github repository
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <events/mbed_events.h>
#include <mbed.h>
#include "ble/BLE.h"
#include "ble/DiscoveredCharacteristic.h"
#include "ble/DiscoveredService.h"
#include "ble/Gap.h"
#include "ble/gap/AdvertisingDataParser.h"
#include "pretty_printer.h"



const static char PEER_NAME[] = "ButtonLED";
const static uint16_t PEER_UUID = 0x1815;
const static uint16_t PEER_CHAR_UUID = 0x2A56;

DigitalOut _red_led(LED1, 1);
DigitalOut _green_led(LED2, 1);
DigitalOut _blue_led(LED3, 1);

static EventQueue event_queue(/* event count */ 10 * EVENTS_EVENT_SIZE);

static DiscoveredCharacteristic led_characteristic[3];
static GattAttribute::Handle_t _descriptor_handle;
static uint8_t cIndex = 0;
static uint8_t ColourState = 1;
static int8_t wIndex = -1;

typedef CharacteristicDescriptorDiscovery::DiscoveryCallbackParams_t  DiscoveryCallbackParams_t;
typedef CharacteristicDescriptorDiscovery::TerminationCallbackParams_t TerminationCallbackParams_t;

void when_descriptor_discovered(const DiscoveryCallbackParams_t* event);
void when_descriptor_discovery_ends(const TerminationCallbackParams_t *event);
void trigger_hvx(const GattHVXCallbackParams *response);


void Process_NextCharacteristic()
{
    //_descriptor_handle = 0;
    uint8_t i;

    if (cIndex == 3) cIndex = 0;
    else cIndex++;

    // Here we need to register for notification events.
    for (i = cIndex; i < 3; i++) {
        if (led_characteristic[i].getUUID().getShortUUID() == PEER_CHAR_UUID) {
            if (led_characteristic[i].getProperties().notify() | led_characteristic[i].getProperties().indicate()) {
                serial.printf(" - initiating descriptor discovery process for NOTIFY property [%u]\r\n", cIndex);

                ble_error_t error = led_characteristic[i].discoverDescriptors(&when_descriptor_discovered,&when_descriptor_discovery_ends);
                if (error) print_error(error, "discovery_termination discoverDescriptors");                
                break;
            }
            else if (led_characteristic[i].getProperties().write()) {
                wIndex = i;
            }

        } 
    }
    
}

/**
* Handle the discovery of the characteristic descriptors.
* If the descriptor found is a CCCD then stop the discovery. Once the
* process has ended subscribe to server initiated events by writing the
* value of the CCCD.
*/
void when_descriptor_discovered(const DiscoveryCallbackParams_t* event)
{
    serial.printf("   - Descriptor discovered at %u", event->descriptor.getAttributeHandle());
    if (event->descriptor.getUUID() == BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG) {
        BLE::Instance().gattClient().terminateCharacteristicDescriptorDiscovery(event->characteristic);
        serial.printf("   - CCCD found\r\n");
        _descriptor_handle = event->descriptor.getAttributeHandle();
    }
}

/**
* If a CCCD has been found subscribe to server initiated events by writing
* its value.
*/
void when_descriptor_discovery_ends(const TerminationCallbackParams_t *event) {
    
    if (!_descriptor_handle) {
        serial.printf("  -- Warning: characteristic has notify/indicate without CCCD.\r\n");
    }
    else {
        uint16_t cccd_value = 
        (led_characteristic[0].getProperties().notify() << 0) | (led_characteristic[0].getProperties().indicate() << 1);
        ble_error_t error = BLE::Instance().gattClient().write(
                            GattClient::GATT_OP_WRITE_REQ,
                            led_characteristic[0].getConnectionHandle(),
                            _descriptor_handle,
                            sizeof(cccd_value),
                            reinterpret_cast<uint8_t*>(&cccd_value));

        if (error) {
            print_error(error, "Cannot initiate write of CCCD.");
        }
        else serial.printf("   - notify/indicate now enabled\r\n");

    }
    //Process_NextCharacteristic();
}

// This gets triggered once connected
void service_discovery(const DiscoveredService *service) {
    if (service->getUUID().shortOrLong() == UUID::UUID_TYPE_SHORT) {
        serial.printf("Service UUID-%x attrs[%u - %u]\r\n", service->getUUID().getShortUUID(), service->getStartHandle(), service->getEndHandle());
    } else {
        serial.printf("S UUID-");
        const uint8_t *longUUIDBytes = service->getUUID().getBaseUUID();
        for (unsigned i = 0; i < UUID::LENGTH_OF_LONG_UUID; i++) {
            serial.printf("%02x", longUUIDBytes[i]);
        }
        serial.printf(" attrs[%u %u]\r\n", service->getStartHandle(), service->getEndHandle());
    }
}

// This gets triggered once connected
void characteristic_discovery(const DiscoveredCharacteristic *characteristicP) {
    serial.printf("  Char UUID-%x has valueHandle[%u] declHandle[%u] connHandle[%u]\r\n", 
        characteristicP->getUUID().getShortUUID(),
        characteristicP->getValueHandle(), 
        characteristicP->getDeclHandle(),
        characteristicP->getConnectionHandle());

    if (characteristicP->getProperties().broadcast()) serial.printf("  - has Broadcast property\r\n");
    if (characteristicP->getProperties().indicate()) {
        serial.printf("  - has Indicate property\r\n");
    }
    if (characteristicP->getProperties().notify()) {
        serial.printf("  - has Notify property\r\n");
    }
    if (characteristicP->getProperties().read()) serial.printf("  - has Read property\r\n");
    if (characteristicP->getProperties().write()) serial.printf("  - has Write property\r\n");
    if (characteristicP->getProperties().writeWoResp()) serial.printf("  - has Write NoResp property\r\n");

    if (cIndex < 3 && (characteristicP->getUUID().getShortUUID() == PEER_CHAR_UUID)) {
        serial.printf("This UUID matches stored Characteristic. Saving details...\r\n");
        led_characteristic[cIndex] = *characteristicP;
        cIndex++;
    }
}

void discovery_termination(ble::connection_handle_t connectionHandle) {
    serial.printf("Terminated Discovery process for conn handle %u\r\n", connectionHandle);
    Process_NextCharacteristic();
}

void trigger_hvx(const GattHVXCallbackParams *response) {
    serial.printf("trigger_hvx: ");
    if ((response->handle == led_characteristic[0].getValueHandle()) && response->len == 1) {
        serial.printf("Button State is %u", response->data[0]);
        if (ColourState == 1) _red_led = response->data[0]; 
        else if (ColourState == 2) _green_led = response->data[0]; 
        else if (ColourState == 3) _blue_led = response->data[0]; 
    }
    else if ((response->handle == led_characteristic[1].getValueHandle()) && response->len == 1) {
        uint8_t BtnColour = response->data[0];
        serial.printf("Button Colour is %u", BtnColour);
        if (BtnColour == 1) {
            if (ColourState < 3) ColourState++;
            else ColourState = 1;

            if (ColourState == 1) {
                _red_led = 0;
                _green_led = 1;
                _blue_led = 1;
            }
            else if (ColourState == 2) {
                _red_led = 1;
                _green_led = 0;
                _blue_led = 1;
            }
            else if (ColourState == 3) {
                _red_led = 1;
                _green_led = 1;
                _blue_led = 0;
            }
        }
    }
    serial.printf(".\r\n");
}

void when_DataWritten(const GattWriteCallbackParams *response) {
        // should never happen
        if (!_descriptor_handle) {
            printf("\tError: received write response to unsolicited request.\r\n");
            return;
        }

        printf("\tCCCD at %u was written.\r\n", _descriptor_handle);
        _descriptor_handle = 0;
        Process_NextCharacteristic();
}

// Message will be printed when button released
void when_button_pressed(uint8_t Msg) {
    // this does not run in the ISR so we create a queue
    serial.printf("Button was %s!\r\n", Msg == 1 ? "Released":"Pressed");

    if (wIndex >=0) {
        led_characteristic[wIndex].write(1, &Msg);
    }
}



class LEDBlinkerDemo : ble::Gap::EventHandler {
public:
    LEDBlinkerDemo(BLE &ble, events::EventQueue &event_queue) :
        _ble(ble),
        _event_queue(event_queue),
        _button1(BUTTON1, PullUp),
        _is_connecting(false),
        _is_connected(false) { }

    ~LEDBlinkerDemo() { 
        stop();
    }

    void start() {
        _ble.gap().setEventHandler(this);

        _ble.init(this, &LEDBlinkerDemo::on_init_complete);

        _event_queue.call_every(500ms, this, &LEDBlinkerDemo::blink);

        _event_queue.dispatch_forever();
    }

    void stop() {
        _ble.gattClient().onHVX().detach(trigger_hvx);
        _ble.gattClient().onServiceDiscoveryTermination(NULL);
        memset(led_characteristic, 0, sizeof(DiscoveredCharacteristic)*3);
        cIndex = 0;
        ColourState = 1;
        wIndex = -1;
        _descriptor_handle = 0;
        _green_led = 1;
        _blue_led = 1;
    }
   
private:
    /** Callback triggered when the ble initialization process has finished */
    void on_init_complete(BLE::InitializationCompleteCallbackContext *params) {
        if (params->error != BLE_ERROR_NONE) {
            serial.printf("Ble initialization failed.");
            return;
        }        
        
        print_mac_address();

        ble::ScanParameters scan_params;
        _ble.gap().setScanParameters(scan_params);
        _ble.gap().startScan();
    }

    void blink() {
        if (!_is_connected) _red_led = !_red_led;
    }

    /* Event handler */
    void onDisconnectionComplete(const ble::DisconnectionCompleteEvent&) {
        serial.printf("Disconnected. Start scanning...\r\n");
        stop();
        _ble.gap().startScan();
        _is_connecting = false;
        _is_connected = false;
    }

    void onConnectionComplete(const ble::ConnectionCompleteEvent& event) {
        if (event.getOwnRole() == ble::connection_role_t::CENTRAL) {
            _ble.gattClient().onServiceDiscoveryTermination(discovery_termination);
            _ble.gattClient().launchServiceDiscovery(
                event.getConnectionHandle(),
                service_discovery,
                characteristic_discovery,
                PEER_UUID,
                PEER_CHAR_UUID
            );

            //_ble.gattClient().onDataRead(when_DataRead);
            _ble.gattClient().onDataWritten(when_DataWritten);
            _ble.gattClient().onHVX(trigger_hvx);

            serial.printf("Now Connected...\r\n");
            _is_connected = true;

            // Set up our interrupt callback functions
            _button1.fall(Callback<void()>(this, &LEDBlinkerDemo::button1_pressed));
            _button1.rise(Callback<void()>(this, &LEDBlinkerDemo::button1_released));


        } 
        
        else {
            _ble.gap().startScan();
        }
        _is_connecting = false;
    }

    void onAdvertisingReport(const ble::AdvertisingReportEvent &event) {
        /* don't bother with analysing scan result if we're already connecting */
        if (_is_connecting) {
            return;
        }

        ble::AdvertisingDataParser adv_data(event.getPayload());

        /* parse the advertising payload, looking for a discoverable device */
        while (adv_data.hasNext()) {
            ble::AdvertisingDataParser::element_t field = adv_data.next();

            if (field.type == ble::adv_data_type_t::INCOMPLETE_LIST_16BIT_SERVICE_IDS) {
                serial.printf("Found INCOMPLETE 16BIT...\r\n");
            }
            else if (field.type == ble::adv_data_type_t::COMPLETE_LIST_16BIT_SERVICE_IDS) {
                serial.printf("Found COMPLETE 16BIT");
                bool Found_PeerUUID = false;
                if ((field.value.size() > 1) && (field.value.size() < 256)) {
                    for (uint8_t xx = 0; xx < field.value.size(); xx +=2) {
                        serial.printf(" Service UUID: 0x%02X%02X ", field.value[xx+1], field.value[xx]);
                        if (PEER_UUID == (field.value[xx] | field.value[xx+1] << 8)) Found_PeerUUID = true;
                    }
                    serial.printf("\r\n");
                    if (Found_PeerUUID) serial.printf("This Service UUID matches stored UUID\r\n");
                }
            }
            else if (field.type == ble::adv_data_type_t::INCOMPLETE_LIST_128BIT_SERVICE_IDS) {
                serial.printf("Found INCOMPLETE 128BIT...\r\n");
            }
            else if (field.type == ble::adv_data_type_t::COMPLETE_LIST_128BIT_SERVICE_IDS) {
                serial.printf("Found COMPLETE 128BIT SERVIDS...\r\n");
            }

            /* connect to a discoverable device */
            if (field.type == ble::adv_data_type_t::COMPLETE_LOCAL_NAME &&
                field.value.size() == strlen(PEER_NAME) &&
                (memcmp(field.value.data(), PEER_NAME, field.value.size()) == 0)) {

                serial.printf("This MAC Adr matches stored local name: ");
                print_address(event.getPeerAddress().data());
                serial.printf(" rssi: %d, scan response: %u, connectable: %u\r\n",
                       event.getRssi(), event.getType().scan_response(), event.getType().connectable());

                ble_error_t error = _ble.gap().stopScan();

                if (error) {
                    print_error(error, "Error caused by Gap::stopScan");
                    return;
                }

                const ble::ConnectionParameters connection_params;

                error = _ble.gap().connect(
                    event.getPeerAddressType(),
                    event.getPeerAddress(),
                    connection_params
                );

                if (error) {
                    _ble.gap().startScan();
                    return;
                }

                /* we may have already scan events waiting
                 * to be processed so we need to remember
                 * that we are already connecting and ignore them */
                _is_connecting = true;
                // Set up the LED's
                _red_led = 0;
                _green_led = 1;
                _blue_led = 1;

                return;
            }
        }
    }
    
    void button1_pressed(void) {
        _event_queue.call(Callback<void(void)>(when_button_pressed, 0));
    }

    void button1_released(void) {
        _event_queue.call(Callback<void(void)>(when_button_pressed, 1));
    }


private:
    BLE &_ble;
    InterruptIn _button1;
    
    events::EventQueue &_event_queue;
    bool _is_connecting;
    bool _is_connected;
    
};

/** Schedule processing of events from the BLE middleware in the event queue. */
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
}

int main()
{
    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(schedule_ble_events);

    LEDBlinkerDemo demo(ble, event_queue);
    demo.start();

    return 0;
}
