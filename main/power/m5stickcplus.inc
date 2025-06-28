// M5StickCPlus implmentation - also uses AXP192 but configured
// differently from Jade v1.x
// NOTE: also handles M5StackCore2 as similar enough ...
//
#include "power/i2c.inc"

#define AXP192_ADDR 0x34
static i2c_master_dev_handle_t axp192 = NULL;

static esp_err_t _power_enable_coulomb_counter(void) { return _power_write_command(axp192, 0xb8, 0x80); }

esp_err_t power_init(void)
{
    I2C_CHECK_RET(_power_i2c_init());
    I2C_CHECK_RET(_power_i2c_attach_device(AXP192_ADDR, &axp192));

    _power_enable_coulomb_counter();

    return ESP_OK;
}

esp_err_t power_camera_on(void) { return ESP_OK; }
esp_err_t power_camera_off(void) { return ESP_OK; }

esp_err_t power_shutdown(void)
{
    JADE_SEMAPHORE_TAKE(i2c_mutex);
    const esp_err_t ret = _power_write_command(axp192, 0x32, 0x80);
    JADE_SEMAPHORE_GIVE(i2c_mutex);
    return ret;
}

esp_err_t power_screen_on(void)
{
    uint8_t buf1;
    I2C_LOG_ANY_ERROR(_power_master_read_slave(axp192, 0x12, &buf1, 1));
    vTaskDelay(20 / portTICK_PERIOD_MS);
    return _power_write_command(axp192, 0x12, buf1 | 0x08);
}

esp_err_t power_backlight_on(uint8_t brightness)
{
    uint8_t buf1;

    // dimming not supported - just full on
    I2C_LOG_ANY_ERROR(_power_master_read_slave(axp192, 0x12, &buf1, 1));
    vTaskDelay(20 / portTICK_PERIOD_MS);
    return _power_write_command(axp192, 0x12, buf1 | 0x04);
}

esp_err_t power_backlight_off(void)
{
    uint8_t buf1;
    I2C_LOG_ANY_ERROR(_power_master_read_slave(axp192, 0x12, &buf1, 1));
    vTaskDelay(20 / portTICK_PERIOD_MS);
    return _power_write_command(axp192, 0x12, buf1 & (~0x04));
}

uint16_t power_get_vbat(void)
{
    uint8_t buf1, buf2;
    JADE_SEMAPHORE_TAKE(i2c_mutex);
    I2C_LOG_ANY_ERROR(_power_master_read_slave(axp192, 0x78, &buf1, 1));
    I2C_LOG_ANY_ERROR(_power_master_read_slave(axp192, 0x79, &buf2, 1));
    JADE_SEMAPHORE_GIVE(i2c_mutex);

    const uint16_t vbat = ((buf1 << 4) + buf2) * 1.1;
    return vbat;
}

uint8_t power_get_battery_status(void)
{
    const uint16_t vbat = power_get_vbat();
    if (vbat > 4000) {
        return 5;
    } else if (vbat > 3850) {
        return 4;
    } else if (vbat > 3700) {
        return 3;
    } else if (vbat > 3550) {
        return 2;
    } else if (vbat > 3400) {
        return 1;
    }
    return 0;
}

uint16_t power_get_ibat_charge(void)
{
    uint8_t buf1, buf2;
    JADE_SEMAPHORE_TAKE(i2c_mutex);
    I2C_LOG_ANY_ERROR(_power_master_read_slave(axp192, 0x7A, &buf1, 1));
    I2C_LOG_ANY_ERROR(_power_master_read_slave(axp192, 0x7B, &buf2, 1));
    JADE_SEMAPHORE_GIVE(i2c_mutex);

    const uint16_t ibat = (buf1 << 5) + buf2;
    return ibat;
}

uint16_t power_get_ibat_discharge(void)
{
    uint8_t buf1, buf2;
    JADE_SEMAPHORE_TAKE(i2c_mutex);
    I2C_LOG_ANY_ERROR(_power_master_read_slave(axp192, 0x7C, &buf1, 1));
    I2C_LOG_ANY_ERROR(_power_master_read_slave(axp192, 0x7D, &buf2, 1));
    JADE_SEMAPHORE_GIVE(i2c_mutex);

    const uint16_t ibat = (buf1 << 5) + buf2;
    return ibat;
}

bool power_get_battery_charging(void)
{
    uint8_t buf;
    JADE_SEMAPHORE_TAKE(i2c_mutex);
    I2C_LOG_ANY_ERROR(_power_master_read_slave(axp192, 0x01, &buf, 1));
    JADE_SEMAPHORE_GIVE(i2c_mutex);

    const bool charging = (buf & 0b01000000) >> 6;
    return charging;
}

uint16_t power_get_vusb(void)
{
    uint8_t buf1, buf2;
    JADE_SEMAPHORE_TAKE(i2c_mutex);
    I2C_LOG_ANY_ERROR(_power_master_read_slave(axp192, 0x5a, &buf1, 1));
    vTaskDelay(20 / portTICK_PERIOD_MS);
    I2C_LOG_ANY_ERROR(_power_master_read_slave(axp192, 0x5b, &buf2, 1));
    JADE_SEMAPHORE_GIVE(i2c_mutex);

    const uint16_t vusb = ((buf1 << 4) + buf2) * 1.7;
    return vusb;
}

uint16_t power_get_iusb(void)
{
    uint8_t buf1, buf2;
    JADE_SEMAPHORE_TAKE(i2c_mutex);
    I2C_LOG_ANY_ERROR(_power_master_read_slave(axp192, 0x5c, &buf1, 1));
    vTaskDelay(20 / portTICK_PERIOD_MS);
    I2C_LOG_ANY_ERROR(_power_master_read_slave(axp192, 0x5d, &buf2, 1));
    JADE_SEMAPHORE_GIVE(i2c_mutex);

    const uint16_t iusb = ((buf1 << 4) + buf2) * 0.375;
    return iusb;
}

uint16_t power_get_temp(void)
{
    uint8_t buf1, buf2;
    JADE_SEMAPHORE_TAKE(i2c_mutex);
    I2C_LOG_ANY_ERROR(_power_master_read_slave(axp192, 0x5e, &buf1, 1));
    I2C_LOG_ANY_ERROR(_power_master_read_slave(axp192, 0x5f, &buf2, 1));
    JADE_SEMAPHORE_GIVE(i2c_mutex);

    const uint16_t temp = ((buf1 << 4) + buf2) * 0.1 - 144.7;
    return temp;
}

bool usb_connected(void)
{
    uint8_t buf;
    JADE_SEMAPHORE_TAKE(i2c_mutex);
    I2C_LOG_ANY_ERROR(_power_master_read_slave(axp192, 0x00, &buf, 1));
    JADE_SEMAPHORE_GIVE(i2c_mutex);
    const bool is_usb_connected = buf & 0b00100000;
    return is_usb_connected;
}
