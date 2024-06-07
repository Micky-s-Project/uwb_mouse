#include "sc7i22.h"
#include "ingsoc.h"
#include "platform_api.h"
#include "iic.h"
#include <stddef.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "algo_interfaces.h"

#define SENSOR_IRQ_PIN GIO_GPIO_6
#define IIC_SCL_PIN GIO_GPIO_7
#define IIC_SDA_PIN GIO_GPIO_8

#define I2C_PORT I2C_PORT_0
#define I2C_BASE(port) ((port) == I2C_PORT_0 ? APB_I2C0 : APB_I2C1)
#define SC7I22_I2C_ADDR 0x69

int16_t *imu_raw_data;
uint8_t sensor_data[12];
uint8_t sensor_is_inited = 0;
uint8_t sensor_is_send_seq = 0;
static void sensor_read(uint8_t reg, uint8_t *data, uint16_t len);
static uint8_t sensor_read_u8(uint8_t reg);

static void sensor_write(uint8_t reg, uint8_t *data, uint16_t len);
static void sensor_write_u8(uint8_t reg, uint8_t data);
static SemaphoreHandle_t sensor_xSemaphore = NULL;
static SemaphoreHandle_t imu_data_mutex = NULL;
SemaphoreHandle_t get_imu_data_mutex()
{
    return imu_data_mutex;
}
uint32_t i2c_irq(void *p)
{
    uint32_t status = I2C_GetIntState(APB_I2C0);

    if (status & (1 << I2C_STATUS_CMPL))
    {
        I2C_ClearIntState(APB_I2C0, (1 << I2C_STATUS_CMPL));

        // switch (sensor_is_send_seq)
        // {
        // case 0: // 发送传输的地址和寄存器
        //         //             I2C_DmaEnable(APB_I2C0, 1);
        //         //             I2C_CtrlUpdateDirection(APB_I2C0, I2C_TRANSACTION_SLAVE2MASTER);
        //         //             I2C_CtrlUpdateDataCnt(APB_I2C0, DATA_CNT);

        //     // #define I2C_DMA_RX_CHANNEL (0) // DMA channel 0
        //     //             peripherals_i2c_rxfifo_to_dma(I2C_DMA_RX_CHANNEL, read_data,
        //     //                                           sizeof(read_data));

        //     //             I2C_CommandWrite(APB_I2C0, I2C_COMMAND_ISSUE_DATA_TRANSACTION);
        //     //             sensor_is_send_seq = 1;
        //     break;
        // case 1: // 传输接收
        //     DMA_Descriptor descriptor __attribute__((aligned(8)));

        //     descriptor.Next = (DMA_Descriptor *)0;
        //     DMA_PreparePeripheral2Mem(&descriptor, dst, SYSCTRL_DMA_I2C0,
        //                               size, DMA_ADDRESS_INC, 0);

        //     DMA_EnableChannel(channel_id, &descriptor);
        //     sensor_is_send_seq = 2;
        //     break;
        // case 2: // 接收完成
        platform_printf("tran done\n");
        //     sensor_is_send_seq = 0;
        //     break;
        // default:
        //     break;
        // }
    }
    return 0;
}
uint8_t read_data[14] = {SC7I22_I2C_ADDR, 0x0c};

void peripherals_i2c_rxfifo_to_dma(int channel_id, void *dst, int size)
{
    DMA_Descriptor descriptor __attribute__((aligned(8)));

    descriptor.Next = (DMA_Descriptor *)0;
    DMA_PreparePeripheral2Mem(&descriptor, dst, SYSCTRL_DMA_I2C0,
                              size, DMA_ADDRESS_INC, 0);

    DMA_EnableChannel(channel_id, &descriptor);
}

uint32_t gpio0_isr(void *user_data)
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

    //     I2C_DmaEnable(APB_I2C0, 1);
    //     I2C_CtrlUpdateDirection(APB_I2C0, I2C_TRANSACTION_MASTER2SLAVE);
    //     I2C_CtrlUpdateDataCnt(APB_I2C0, 14);

    // #define I2C_DMA_RX_CHANNEL (0) // DMA channel 0
    //     peripherals_i2c_rxfifo_to_dma(I2C_DMA_RX_CHANNEL, read_data, sizeof(read_data));

    //     I2C_CommandWrite(APB_I2C0, I2C_COMMAND_ISSUE_DATA_TRANSACTION);
    //     sensor_is_send_seq = 1;

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
    I2C_ConfigClkFrequency(APB_I2C0, I2C_CLOCKFREQUENY_FASTMODE_PLUS);
    // I2C0
    PINCTRL_SelI2cIn(0, 7, 8);
    PINCTRL_SetPadMux(7, 106);
    PINCTRL_SetPadMux(8, 107);
    PINCTRL_SetPadMux(7, 24);
    PINCTRL_SetPadMux(8, 25);
    // I2C_IntEnable(APB_I2C0, (1 << I2C_INT_CMPL));
    I2C_Enable(APB_I2C0, 1);
    i2c_init(I2C_PORT);
    uint8_t who_am_i = 0;

    who_am_i = sensor_read_u8(0x75);
    platform_printf("who_am_i reg : 0x%02x\r\n", who_am_i);
    if (who_am_i != 0x6a)
    {
        platform_printf("sc7i22 not found!!!\r\n");
        // return;
    }
soft_res:

    // sensor_write_u8(0x4A, 0xA5); // SOFT_RST
    // // uint8_t com_cfg = sensor_read_u8(0x05);
    // // sensor_write_u8(0x05, com_cfg | 0x80);
    // sensor_write_u8(0x04, 0x10); // OIS_CONF
    // uint8_t int_cfg1 = sensor_read_u8(0x06);
    // sensor_write_u8(0x06, 0b00000011); //  INT_CFG1  AOI1| AOI2 中断在 INT1 上
    // platform_printf("INT_CFG1 :0x%02x  updata to 0x%02x\r\n", int_cfg1, sensor_read_u8(0x06));
    // sensor_write_u8(0x41, 0x03); // ACC_RANGE 16g
    // vTaskDelay(pdMS_TO_TICKS(10));
    // sensor_write_u8(0x41, 0x03);       // ACC_RANGE 16g
    // sensor_write_u8(0x40, 0b10001100); // ACC_CONF 1.6k
    // sensor_write_u8(0x42, 0b11001101); // GYR_CONF 3.2k
    // sensor_write_u8(0x43, 0x08);       // GYR_RANGE 2000dps
    // sensor_write_u8(0x7D, 0x0e); /* PWR_CTRL */

    // Init for ICM24688
    sensor_write_u8(0x76, 0x00);
    sensor_write_u8(0x11, 0x01);
    vTaskDelay(pdMS_TO_TICKS(10));
    sensor_write_u8(0x76, 0x01);
    sensor_write_u8(0x7A, 0x02);

    sensor_write_u8(0x76, 0x00);
    //    sensor_write_u8(0x16, 0x40);
    sensor_write_u8(0x14, 0x03);
    sensor_write_u8(0x65, 0x7f);
    sensor_write_u8(0x63, 0x20);
    sensor_write_u8(0x64, 0x60);
    sensor_write_u8(0x4C, 0x20);//little endian
    // sensor_write_u8(0x14, 0x00);
    sensor_write_u8(0x4E, 0x0F);

    sensor_write_u8(0x4F, 0x03); // gyro
    sensor_write_u8(0x50, 0x03); // acce
    vTaskDelay(pdMS_TO_TICKS(3));

    // uint8_t acc_range = sensor_read_u8(0x41);
    // uint8_t gyr_range = sensor_read_u8(0x43);
    // uint8_t acc_odr = sensor_read_u8(0x40);
    // uint8_t gyr_odr = sensor_read_u8(0x42);

    // platform_printf("ACC_RANGE :0x%02x GYR_RANGE :0x%02x \n", acc_range, gyr_range);
    // if ((acc_range != 0x03) || (gyr_range != 0x08) || (acc_odr != 140) || (gyr_odr != 205))
    // {
    //     goto soft_res;
    // }

    sensor_is_inited = 1;

    // I2C_DmaEnable(APB_I2C0, 1);
    // I2C_IntEnable(APB_I2C0, (1 << I2C_INT_CMPL));
    // platform_set_irq_callback(PLATFORM_CB_IRQ_I2C0, i2c_irq, NULL);
    // // init dma
    // SYSCTRL_ClearClkGateMulti(1 << SYSCTRL_ClkGate_APB_DMA);
    // DMA_Reset(1);
    // DMA_Reset(0);

    // sensor_read(0x0A, sensor_data, sizeof(sensor_data));
    while (1)
    {
        if (xSemaphoreTake(sensor_xSemaphore, portMAX_DELAY) == pdTRUE)
        {
            sensor_read(0x1f, sensor_data, sizeof(sensor_data));
            if (xSemaphoreTake(imu_data_mutex, 0) == pdTRUE)
            {
                imu_raw_data = (int16_t *)sensor_data;
                // for (uint8_t i = 0; i < 6; i++)
                // {
                //     imu_raw_data[i] = __REV16(imu_raw_data[i]);
                // }
                xSemaphoreGive(imu_data_mutex);
            }
            algo_imu_data_update_event_handler(&imu_raw_data[3], imu_raw_data);
            // platform_printf("acc_x=%d,acc_y=%d,acc_z=%d,gyr_x=%d,gyr_y=%d,gyr_z=%d\r\n",
            //                 imu_raw_data[0], imu_raw_data[1], imu_raw_data[2], imu_raw_data[3], imu_raw_data[4], imu_raw_data[5]);
        }
    }
}

TaskHandle_t sensor_task_handle = NULL;
void sc7122_init()
{
    vSemaphoreCreateBinary(sensor_xSemaphore);
    imu_data_mutex = xSemaphoreCreateMutex();
    /* config gpio irq */
    // PINCTRL_Pull(SENSOR_IRQ_PIN, PINCTRL_PULL_UP);

    //    GIO_SetDirection(SENSOR_IRQ_PIN, GIO_DIR_INPUT);
    //    GIO_ConfigIntSource(SENSOR_IRQ_PIN, GIO_INT_EN_LOGIC_HIGH_OR_RISING_EDGE, GIO_INT_EDGE);
    platform_set_irq_callback(PLATFORM_CB_IRQ_GPIO, gpio0_isr, NULL);

    xTaskCreate(sensor_updata_task, "sensor updata", 384, NULL, configMAX_PRIORITIES - 1 /* tskIDLE_PRIORITY */, &sensor_task_handle);
}
