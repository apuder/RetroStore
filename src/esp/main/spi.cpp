
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <driver/sdspi_host.h>
#include "config.h"
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_task.h"
#include "esp_event_loop.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "spi.h"

#define TAG "example"

#define MCP23S_CHIP_ADDRESS 0x40
#define MCP23S_WRITE 0x00
#define MCP23S_READ  0x01

#define MOUNT_POINT "/sdcard"

static spi_bus_config_t spi_bus;

static spi_device_interface_config_t spi_mcp23S17;
spi_device_handle_t spi_mcp23S17_h;

static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
  xQueueSendFromISR(gpio_evt_queue, NULL, NULL);
}

void writePortExpander(spi_device_handle_t dev, uint8_t cmd, uint8_t data)
{
  spi_transaction_t trans;

  memset(&trans, 0, sizeof(spi_transaction_t));
  trans.flags = SPI_TRANS_USE_TXDATA;
  trans.length = 3 * 8;   		// 3 bytes
  trans.tx_data[0] = MCP23S_CHIP_ADDRESS | MCP23S_WRITE;
  trans.tx_data[1] = cmd;
  trans.tx_data[2] = data;

  esp_err_t ret = spi_device_transmit(dev, &trans);
  ESP_ERROR_CHECK(ret);
}

uint8_t readPortExpander(spi_device_handle_t dev, uint8_t reg)
{
  spi_transaction_t trans;

  memset(&trans, 0, sizeof(spi_transaction_t));
  trans.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
  trans.length = 3 * 8;   		// 2 bytes
  trans.rxlength = 0;
  trans.tx_data[0] = MCP23S_CHIP_ADDRESS | MCP23S_READ;
  trans.tx_data[1] = reg;
  trans.tx_data[2] = 0;

  esp_err_t ret = spi_device_transmit(dev, &trans);
  ESP_ERROR_CHECK(ret);

  return trans.rx_data[2];
}


#if 1
void wire_test_port_expander()
{
#if 0
  while (1) {
    if(xQueueReceive(gpio_evt_queue, NULL, portMAX_DELAY)) {
      uint8_t data = readPortExpander2(MCP23S08_INTCAP);
      Serial.println(data);
    }
  }
#else

  // Writing
  uint8_t data = 0;
  writePortExpander(MCP23S17, MCP23S17_IODIRA, 0);
  writePortExpander(MCP23S17, MCP23S17_IODIRB, 0);

  while (1) {
    writePortExpander(MCP23S17, MCP23S17_GPIOA, data);
    writePortExpander(MCP23S17, MCP23S17_GPIOB, data);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    data ^= 0xff;
    writePortExpander(MCP23S17, MCP23S17_GPIOA, data);
    writePortExpander(MCP23S17, MCP23S17_GPIOB, data);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    data ^= 0xff;
  }
#endif
}

#endif


void init_spi()
{
      esp_err_t ret;

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    ESP_LOGI(TAG, "Initializing SD card");

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.
    ESP_LOGI(TAG, "Using SPI peripheral");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SPI_PIN_NUM_MOSI,
        .miso_io_num = SPI_PIN_NUM_MISO,
        .sclk_io_num = SPI_PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    //ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CHAN);
    ret = spi_bus_initialize(HSPI_HOST, &bus_cfg, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = GPIO_NUM_2;//PIN_NUM_CS;
    slot_config.host_id = HSPI_HOST;
    //slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

      // Configure SPI device for MCP23S17
  spi_mcp23S17.address_bits = 0;
  spi_mcp23S17.command_bits = 0;
  spi_mcp23S17.dummy_bits = 0;
  spi_mcp23S17.mode = 0;
  spi_mcp23S17.duty_cycle_pos = 0;
  spi_mcp23S17.cs_ena_posttrans = 0;
  spi_mcp23S17.cs_ena_pretrans = 0;
  spi_mcp23S17.clock_speed_hz = SPI_SPEED_MHZ * 1000 * 1000;
  spi_mcp23S17.spics_io_num = SPI_PIN_NUM_CS_MCP23S17;
  spi_mcp23S17.flags = 0;
  spi_mcp23S17.queue_size = 1;
  spi_mcp23S17.pre_cb = NULL;
  spi_mcp23S17.post_cb = NULL;
  ret = spi_bus_add_device(HSPI_HOST, &spi_mcp23S17, &spi_mcp23S17_h);
  ESP_ERROR_CHECK(ret);

  //wire_test_port_expander();

  /*
   * MCP23S17 configuration
   */
  // Ports A and B are connected to A0-A15. Configure as input. Disable pull-ups.
  // Disable interrupts
  writePortExpander(MCP23S17, MCP23S17_IODIRA, 0xff);
  writePortExpander(MCP23S17, MCP23S17_IODIRB, 0xff);
  writePortExpander(MCP23S17, MCP23S17_GPPUA, 0);
  writePortExpander(MCP23S17, MCP23S17_GPPUB, 0);
  writePortExpander(MCP23S17, MCP23S17_GPINTENA, 0);
  writePortExpander(MCP23S17, MCP23S17_GPINTENB, 0);
}

void init_spix()
{
  // Configure SPI bus
  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  #if 0
  spi_bus.flags = SPICOMMON_BUSFLAG_MASTER;
  spi_bus.sclk_io_num = SPI_PIN_NUM_CLK;
  spi_bus.mosi_io_num = SPI_PIN_NUM_MOSI;
  spi_bus.miso_io_num = SPI_PIN_NUM_MISO;
  spi_bus.quadwp_io_num = -1;
  spi_bus.quadhd_io_num = -1;
  spi_bus.max_transfer_sz = 4000;
  esp_err_t ret = spi_bus_initialize(HSPI_HOST, &spi_bus, 1);
  #endif
  spi_bus_config_t bus_cfg = {
        .mosi_io_num = SPI_PIN_NUM_MOSI,
        .miso_io_num = SPI_PIN_NUM_MISO,
        .sclk_io_num = SPI_PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
  esp_err_t ret = spi_bus_initialize(HSPI_HOST, &bus_cfg, 1);
  ESP_ERROR_CHECK(ret);

#if 0
  // Configure SPI device for MCP23S17
  spi_mcp23S17.address_bits = 0;
  spi_mcp23S17.command_bits = 0;
  spi_mcp23S17.dummy_bits = 0;
  spi_mcp23S17.mode = 0;
  spi_mcp23S17.duty_cycle_pos = 0;
  spi_mcp23S17.cs_ena_posttrans = 0;
  spi_mcp23S17.cs_ena_pretrans = 0;
  spi_mcp23S17.clock_speed_hz = SPI_SPEED_MHZ * 1000 * 1000;
  spi_mcp23S17.spics_io_num = SPI_PIN_NUM_CS_MCP23S17;
  spi_mcp23S17.flags = 0;
  spi_mcp23S17.queue_size = 1;
  spi_mcp23S17.pre_cb = NULL;
  spi_mcp23S17.post_cb = NULL;
  ret = spi_bus_add_device(HSPI_HOST, &spi_mcp23S17, &spi_mcp23S17_h);
  ESP_ERROR_CHECK(ret);

  //wire_test_port_expander();

  /*
   * MCP23S17 configuration
   */
  // Ports A and B are connected to A0-A15. Configure as input. Disable pull-ups.
  // Disable interrupts
  writePortExpander(MCP23S17, MCP23S17_IODIRA, 0xff);
  writePortExpander(MCP23S17, MCP23S17_IODIRB, 0xff);
  writePortExpander(MCP23S17, MCP23S17_GPPUA, 0);
  writePortExpander(MCP23S17, MCP23S17_GPPUB, 0);
  writePortExpander(MCP23S17, MCP23S17_GPINTENA, 0);
  writePortExpander(MCP23S17, MCP23S17_GPINTENB, 0);
#endif

     esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;

#if 0
    sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
    slot_config.gpio_cs = GPIO_NUM_2;
    slot_config.gpio_sck = SPI_PIN_NUM_CLK;
    slot_config.gpio_mosi = SPI_PIN_NUM_MOSI;
    slot_config.gpio_miso = SPI_PIN_NUM_MISO;
#else
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = GPIO_NUM_2;
    slot_config.host_id = HSPI_HOST;
#endif

    sdspi_dev_handle_t dev;
    //sdspi_host_init_device(&slot_config, &dev);
    spi_device_handle_t sd_dev;
    //spi_bus_add_device(HSPI_HOST, &dev, &sd_dev);
    ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);
}
