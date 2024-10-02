#include <Arduino.h>

#include "hardware/timer.h"

#include "nthaka.h"
#include "nthaka/nxmc2.h"
#include "nthaka/orca.h"
#include "nthaka/pokecon.h"

#include "SwitchCommon.h"
#include "SwitchBluetooth.h"
#include "btstack.h"
#include "SwitchConsts.h"

static nxmc2_format_handler_t nxmc2;
static orca_format_handler_t orca;
static pokecon_format_handler_t pokecon;
static nthaka_format_handler_t *fmts[] = {(nthaka_format_handler_t *)&nxmc2, //
                                          (nthaka_format_handler_t *)&orca,  //
                                          (nthaka_format_handler_t *)&pokecon};
static nthaka_multi_format_handler_t fmt;
static nthaka_buffer_t buf;

static nthaka_gamepad_state_t out;
static char str[NTHAKA_GAMEPAD_STATE_STRING_SIZE_MAX];

SwitchCommon *switchCommon = new SwitchBluetooth();
static btstack_packet_callback_registration_t hci_event_callback_registration;

static void packet_handler_wrapper(uint8_t packet_type, uint16_t channel,
                                   uint8_t *packet, uint16_t packet_size) {
  packet_handler((SwitchBluetooth *)switchCommon, packet_type, packet);
}

static void hid_report_data_callback_wrapper(uint16_t cid,
                                             hid_report_type_t report_type,
                                             uint16_t report_id,
                                             int report_size, uint8_t *report) {
  // USB report callback includes 2 bytes excluded here, prepending 2 bytes to
  // keep alignment
  hid_report_data_callback(switchCommon, report_id, report - 1,
                           report_size + 1);
}

static int64_t _led_off(alarm_id_t id, void *user_data)
{
    digitalWriteFast(LED_BUILTIN, LOW);
    return 0;
}

static inline void async_led_on(uint32_t dur_ms)
{
    digitalWriteFast(LED_BUILTIN, HIGH);
    add_alarm_in_ms(dur_ms, _led_off, NULL, false);
}

static void update() {

    async_led_on(100);

    uint8_t but[3] = {0,0,0};
    uint16_t lx = 0x800;
    uint16_t ly = 0x800;
    uint16_t rx = 0x800;
    uint16_t ry = 0x800;
    if (CONTROLLER_TYPE == PRO_CON || CONTROLLER_TYPE == JOYCON_R) {
        if (out.y) but[0] |= 0x01;
        if (out.x) but[0] |= 0x02;
        if (out.b) but[0] |= 0x04;
        if (out.a) but[0] |= 0x08;
        if (CONTROLLER_TYPE == JOYCON_R) {
            if (out.l) but[0] |= 0x10; //SR
            if (out.zl) but[0] |= 0x20; //SL
        }
        if (out.r) but[0] |= 0x40;
        if (out.zr) but[0] |= 0x80;
        if (out.plus) but[1] |= 0x02;
        if (out.r_click) but[1] |= 0x04;
        if (out.home) but[1] |= 0x10;
        rx = (uint16_t)map(out.r_stick.x, 0, 255, 0, 0xFFF);
        ry = (uint16_t)map(out.r_stick.y, 0, 255, 0xFFF, 0);
    }
    if (CONTROLLER_TYPE == PRO_CON || CONTROLLER_TYPE == JOYCON_L) {
        if (out.minus) but[1] |= 0x01;
        if (out.l_click) but[1] |= 0x08;
        if (out.capture) but[1] |= 0x20;
        if (out.l) but[2] |= 0x40;
        if (out.zl) but[2] |= 0x80;
        if (CONTROLLER_TYPE == JOYCON_L) {
            if (out.r) but[2] |= 0x10; //SR
            if (out.zr) but[2] |= 0x20; //SL
        }
        switch (out.hat) {
            case NTHAKA_HAT_UP:
                but[2] = 0x02;
                break;
            case NTHAKA_HAT_UPRIGHT:
                but[2] = 0x06;
                break;
            case NTHAKA_HAT_RIGHT:
                but[2] = 0x04;
                break;
            case NTHAKA_HAT_DOWNRIGHT:
                but[2] = 0x05;
                break;
            case NTHAKA_HAT_DOWN:
                but[2] = 0x01;
                break;
            case NTHAKA_HAT_DOWNLEFT:
                but[2] = 0x09;
                break;
            case NTHAKA_HAT_LEFT:
                but[2] = 0x08;
                break;
            case NTHAKA_HAT_UPLEFT:
                but[2] = 0x0A;
                break;
            default:
            case NTHAKA_HAT_NEUTRAL:
                break;
        }
        lx = (uint16_t)map(out.l_stick.x, 0, 255, 0, 0xFFF);
        ly = (uint16_t)map(out.l_stick.y, 0, 255, 0xFFF, 0);
    }
    switchCommon->_switchReport.buttons[0] = but[0];
    switchCommon->_switchReport.buttons[1] = but[1];
    switchCommon->_switchReport.buttons[2] = but[2];
    switchCommon->_switchReport.l[0] = lx & 0xff;
    switchCommon->_switchReport.l[1] = ((ly & 0x0F) << 4) | ((lx & 0xF00) >> 8);
    switchCommon->_switchReport.l[2] = (ly & 0xFF0) >> 4;
    switchCommon->_switchReport.r[0] = rx & 0xff;
    switchCommon->_switchReport.r[1] = ((ry & 0x0F) << 4) | ((rx & 0xF00) >> 8);
    switchCommon->_switchReport.r[2] = (ry & 0xFF0) >> 4;

    //Serial1.println("--------------------");

    //size_t *i = nthaka_multi_format_handler_get_last_deserialized_index(&fmt);
    //sprintf(str, "index: %s", i == NULL ? "unknown" //
    //                          : *i == 0 ? "NXMC2"
    //                          : *i == 1 ? "ORCA"
    //                          : *i == 2 ? "PokeCon"
    //                                    : "unknown");
    //Serial1.println(str);

    //nthaka_gamepad_state_stringify(&out, str, NTHAKA_GAMEPAD_STATE_STRING_SIZE_MAX);
    //Serial1.println(str);
}

void setup()
{
    Serial.setTimeout(100);
    Serial.begin(9600);

    //Serial1.setTX(0);
    //Serial1.setRX(1);
    //Serial1.begin(115200);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    if (!(nxmc2_format_handler_init(&nxmc2) &&
          orca_format_handler_init(&orca) &&
          pokecon_format_handler_init(&pokecon) &&
          nthaka_multi_format_handler_init(&fmt, fmts, 3) &&
          nthaka_buffer_init(&buf, (nthaka_format_handler_t *)&fmt)))
    {
        digitalWrite(LED_BUILTIN, HIGH);
        while (true)
            ;
    }
}

void loop()
{
    uint8_t d;
    nthaka_buffer_state_t s;
    if (Serial.readBytes(&d, 1) != 1 ||
        (s = nthaka_buffer_append(&buf, d, &out)) == NTHAKA_BUFFER_REJECTED)
    {
        nthaka_buffer_clear(&buf);
        return;
    }
    else if (s == NTHAKA_BUFFER_PENDING)
    {
        return;
    }

    update();
    nthaka_buffer_clear(&buf);
}

void setup1()
{
  switchCommon->init();
  hci_event_callback_registration.callback = &packet_handler_wrapper;
  hci_add_event_handler(&hci_event_callback_registration);

  hid_device_register_packet_handler(&packet_handler_wrapper);
  hid_device_register_report_data_callback(&hid_report_data_callback_wrapper);

  // turn on!
  hci_power_control(HCI_POWER_ON);
  btstack_run_loop_execute();
}

void loop1()
{

}