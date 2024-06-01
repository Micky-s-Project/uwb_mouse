#include "sc7i22.h"
#include "ingsoc.h"
#include "platform_api.h"
#include "iic.h"
#include <stddef.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define SENSOR_IRQ_PIN GIO_GPIO_6
#define IIC_SCL_PIN GIO_GPIO_7
#define IIC_SDA_PIN GIO_GPIO_8

#define I2C_PORT I2C_PORT_0
#define I2C_BASE(port) ((port) == I2C_PORT_0 ? APB_I2C0 : APB_I2C1)
#define SC7I22_I2C_ADDR 0x19

int16_t *acc, *gyr;
uint8_t sensor_data[12];
uint8_t sensor_is_inited = 0;
static void sensor_read(uint8_t reg, uint8_t *data, uint16_t len);
static uint8_t sensor_read_u8(uint8_t reg);

static void sensor_write(uint8_t reg, uint8_t *data, uint16_t len);
static void sensor_write_u8(uint8_t reg, uint8_t data);
static SemaphoreHandle_t sensor_xSemaphore = NULL;

uint32_t i2c_irq(void *p)
{
    uint32_t status = I2C_GetIntState(APB_I2C0);
    if (status & (1 << I2C_STATUS_CMPL))
    {
        I2C_ClearIntState(APB_I2C0, (1 << I2C_STATUS_CMPL));
        platform_printf("i2c_irq evt\r\n");
    }
    return 0;
}

uint32_t gpio_isr(void *user_data)
{
    uint32_t current = GIO_ReadAll();
    uint32_t status = GIO_GetAllIntStatus();
    static BaseType_t xHigherPriorityTaskWoken;
    GIO_ClearAllIntStatus();
    // platform_printf("gpio_isr evt\r\n");
    if (sensor_xSemaphore != NULL)
    {
        xSemaphoreGiveFromISR(sensor_xSemaphore, &xHigherPriorityTaskWoken);
    }
    return 0;
}

static void sensor_read(uint8_t reg, uint8_t *data, uint16_t len)
{
    i2c_read(I2C_PORT, SC7I22_I2C_ADDR, &reg, 1, data, len);
}

static uint8_t sensor_read_u8(uint8_t reg)
{
    uint8_t tmp[2] = {0};

    i2c_read(I2C_PORT, SC7I22_I2C_ADDR, &reg, 1, tmp, 1);
    return tmp[0];
}

static void sensor_write(uint8_t reg, uint8_t *data, uint16_t len)
{
    i2c_write(I2C_PORT, SC7I22_I2C_ADDR, data, len);
}

static void sensor_write_u8(uint8_t reg, uint8_t data)
{
    uint8_t i2c_send[2] = {reg, data};

    i2c_write(I2C_PORT, SC7I22_I2C_ADDR, i2c_send, 2);
}

static void sensor_updata_task(void *param)
{
    
    PINCTRL_SelI2cIn(I2C_PORT, IIC_SCL_PIN, IIC_SDA_PIN);
    I2C_Config(APB_I2C0, I2C_ROLE_MASTER, I2C_ADDRESSING_MODE_07BIT, SC7I22_I2C_ADDR);
    I2C_ConfigClkFrequency(APB_I2C0, I2C_CLOCKFREQUENY_STANDARD);
    // I2C_IntEnable(APB_I2C0, (1 << I2C_INT_CMPL));
    I2C_Enable(APB_I2C0, 1);

    i2c_init(I2C_PORT);

    uint8_t who_am_i = sensor_read_u8(0x01);
    platform_printf("who_am_i reg : 0x%02x\r\n", who_am_i);
    if (who_am_i != 0x6a)
    {
        platform_printf("sc7i22 not found!!!\r\n");
        // return;
    }

    sensor_write_u8(0x4A, 0xA5); // SOFT_RST
    vTaskDelay(pdMS_TO_TICKS(10));
    // uint8_t com_cfg = sensor_read_u8(0x05);
    // sensor_write_u8(0x05, com_cfg | 0x80);

    sensor_write_u8(0x04, 0x10); // OIS_CONF
    uint8_t int_cfg1 = sensor_read_u8(0x06);
    sensor_write_u8(0x06, 0b00000011); //  INT_CFG1  AOI1| AOI2 中断在 INT1 上
    platform_printf("INT_CFG1 :0x%02x  updata to 0x%02x\r\n", int_cfg1, sensor_read_u8(0x06));
    sensor_write_u8(0x40, 0b10001100); // ACC_CONF 1.6k
    sensor_write_u8(0x41, 0x03);       // ACC_RANGE 16g

    sensor_write_u8(0x42, 0b11001101); // GYR_CONF 3.2k
    sensor_write_u8(0x43, 0x08);       // GYR_RANGE 2000dps

    sensor_write_u8(0x7D, 0x0e); /* PWR_CTRL */
    sensor_is_inited = 1;

    vTaskDelay(pdMS_TO_TICKS(10));

    I2C_DmaEnable(APB_I2C0, 1);
    // I2C_IntEnable(APB_I2C0, (1 << I2C_INT_CMPL));
    // platform_set_irq_callback(PLATFORM_CB_IRQ_I2C0, i2c_irq, NULL);
    // sensor_read(0x0A, sensor_data, sizeof(sensor_data));
    while (1)
    {
        if (xSemaphoreTake(sensor_xSemaphore, portMAX_DELAY) == pdTRUE)
        {
            sensor_read(0x0C, sensor_data, sizeof(sensor_data));
            // uint8_t *data_heat = &sensor_data[3];
            acc = (int16_t *)sensor_data;
            gyr = (int16_t *)&sensor_data[3];
            // for (uint8_t i = 0; i < 3; i++)
            // {
            //     acc[i] = __REV16(acc[i]);
            //     gyr[i] = __REV16(gyr[i]);
            // }
            // platform_printf("acc_x=%d,acc_y=%d,acc_z=%d,gyr_x=%d,gyr_y=%d,gyr_z=%d\r\n",
            //                 acc[0], acc[1], acc[2], gyr[0], gyr[1], gyr[2]);
            // platform_printf("%02x,%02x,%02x,%02x,%02x,%02x,%02x\r\n", sensor_data[0], sensor_data[1], sensor_data[2], sensor_data[3], sensor_data[4], sensor_data[5], sensor_data[6]);
        }
        // vTaskDelay(pdMS_TO_TICKS(100));
        // platform_printf("sensor_updata_task running !!\r\n");
        // sensor_read(0x0A, sensor_data, sizeof(sensor_data));
        // platform_printf("%x %x %x %x %x %x %x \r\n", sensor_data[0], sensor_data[1], sensor_data[2], sensor_data[3], sensor_data[4], sensor_data[5], sensor_data[6]);
    }
}

TaskHandle_t sensor_task_handle = NULL;
void sc7122_init()
{
    vSemaphoreCreateBinary(sensor_xSemaphore);
    /* config gpio irq */
    // PINCTRL_Pull(SENSOR_IRQ_PIN, PINCTRL_PULL_UP);
    GIO_SetDirection(SENSOR_IRQ_PIN, GIO_DIR_INPUT);
    GIO_ConfigIntSource(SENSOR_IRQ_PIN, GIO_INT_EN_LOGIC_HIGH_OR_RISING_EDGE, GIO_INT_EDGE);
    platform_set_irq_callback(PLATFORM_CB_IRQ_GPIO, gpio_isr, NULL);

    xTaskCreate(sensor_updata_task, "sensor updata", 512 * 2, NULL, configMAX_PRIORITIES - 1 /* tskIDLE_PRIORITY */, &sensor_task_handle);
}
