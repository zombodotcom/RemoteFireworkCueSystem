#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/i2c_master.h"
#include "box_controller.h"
#include "board_config.h"
#include "expander_driver.h"
#include "arm_switch.h"
#include "status_leds.h"
#include "espnow_link.h"

static const char* TAG = "firing_box";

extern "C" void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());

    // I2C bus
    i2c_master_bus_config_t bc = {};
    bc.i2c_port = -1;                    // auto-select
    bc.sda_io_num = (gpio_num_t)board::I2C_SDA_GPIO;
    bc.scl_io_num = (gpio_num_t)board::I2C_SCL_GPIO;
    bc.clk_source = I2C_CLK_SRC_DEFAULT;
    bc.glitch_ignore_cnt = 7;
    i2c_master_bus_handle_t bus = nullptr;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bc, &bus));

    static ExpanderChannelDriver driver(bus, board::EXPANDER_I2C_ADDR, board::FIRE_LEVEL);
    // Boot to SAFE even if the expander can't be reached: the external pull
    // resistors hold every SSR input OFF, so an unreachable expander is the
    // safest state, not a reason to reboot-loop. Log the fault loudly.
    esp_err_t derr = driver.begin();     // writes the all-OFF word first
    if (derr != ESP_OK) {
        ESP_LOGE(TAG, "expander init FAILED (%s) — booting SAFE; outputs held off by pull resistors",
                 esp_err_to_name(derr));
    }

    fw::BoxConfig cfg; cfg.boxId = board::THIS_BOX_ID;
    static fw::BoxController box(driver, cfg);
    box.begin();                         // boot-safe: SAFE, all off

    static ArmSwitch armSwitch(board::ARM_SWITCH_GPIO); armSwitch.begin();
    static StatusLeds leds(board::STATUS_LED_GPIO, board::STATUS_LED_COUNT); leds.begin();

    static EspNowLink link;
    ESP_ERROR_CHECK(link.begin(board::CONTROLLER_MAC));

    ESP_LOGI(TAG, "firing box %u booted: SAFE, outputs off", cfg.boxId);

    uint32_t lastRxMs = 0;

    // SAFETY: the box never calls setSequenceRunning(true) — heartbeat dead-man always active.
    while (true) {
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

        // Drain received commands (single-threaded: no race with tick()).
        fw::CommandPacket pkt;
        while (link.receive(pkt)) {
            lastRxMs = now;
            fw::CommandResult r = box.onCommand(pkt, now);
            // ACK FIRED (1=IGNITED_OK) and DUPLICATE (2=ALREADY_FIRED) so that a lost
            // ACK self-heals on the controller's same-id retry.
            // Do NOT ACK REJECTED (genuinely not fired — controller must know).
            if (r == fw::CommandResult::FIRED || r == fw::CommandResult::DUPLICATE) {
                fw::AckPacket ack{};
                ack.type = (uint8_t)fw::MsgType::ACK;
                ack.responseToId = pkt.id;
                ack.deviceStatus = (r == fw::CommandResult::FIRED) ? 1 : 2; // 1=IGNITED_OK, 2=ALREADY_FIRED
                ack.timestamp = now;
                ack.crc = fw::computeCrc(ack);
                link.sendAck(ack);
            }
        }

        box.setPhysicalSwitch(armSwitch.isOn(), now);   // hardware-authoritative
        box.tick(now);
        bool linkAlive = (now - lastRxMs) < 2000;
        leds.show(box.state(), false /*estopped surfaced via BoxState*/, linkAlive, now);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
