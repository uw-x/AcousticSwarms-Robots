// bsp
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"

//// SPI
//#define SPI_SCK_PIN                      NRF_GPIO_PIN_MAP(0,4)
//#define SPI_MOSI_PIN                     NRF_GPIO_PIN_MAP(0,26)
//#define SPI_MISO_PIN                     NRF_GPIO_PIN_MAP(0,27)
//#define SPI_CS_PIN                       NRF_GPIO_PIN_MAP(0,28)

//// ACCEL
//#define ACCEL_EN_PIN                     NRF_GPIO_PIN_MAP(1, 4)
//#define ACCEL_INT1_PIN                   NRF_GPIO_PIN_MAP(1, 7)
//#define ACCEL_INT2_PIN                   NRF_GPIO_PIN_MAP(1, 6)

// TRACE / UART
//#define GPIO_1_PIN                       NRF_GPIO_PIN_MAP(0, 5)
//// #define GPIO_2_PIN                       NRF_GPIO_PIN_MAP(0, 6)
//#define GPIO_3_PIN                       NRF_GPIO_PIN_MAP(0, 7)
//#define GPIO_4_PIN                       NRF_GPIO_PIN_MAP(0, 8)

#define GPIO_1_PIN                       NRF_GPIO_PIN_MAP(1, 01)
// #define GPIO_2_PIN                       NRF_GPIO_PIN_MAP(0, 6)
#define GPIO_3_PIN                       NRF_GPIO_PIN_MAP(1, 04)
#define GPIO_4_PIN                       NRF_GPIO_PIN_MAP(1, 07)

// LED
//#define DEBUG_LED_PIN                    NRF_GPIO_PIN_MAP(0,12)                 // not functional on revA design
//#define BLE_LED_PIN                      NRF_GPIO_PIN_MAP(0,14)                 // 0 to turn on, 1 to turn off

#define DEBUG_LED_PIN                    NRF_GPIO_PIN_MAP(0,09)                 // not functional on revA design
#define BLE_LED_PIN                      NRF_GPIO_PIN_MAP(0,10)                 // 0 to turn on, 1 to turn off

// QSPI
// These are located in sdk_config.h. They're placed here just for reference.
// #define NRFX_QSPI_PIN_SCK                NRF_GPIO_PIN_MAP(0, 19)
// #define NRFX_QSPI_PIN_CSN                NRF_GPIO_PIN_MAP(0, 17)
// #define NRFX_QSPI_PIN_IO0                NRF_GPIO_PIN_MAP(0, 20)
// #define NRFX_QSPI_PIN_IO1                NRF_GPIO_PIN_MAP(0, 21)
// #define NRFX_QSPI_PIN_IO2                NRF_GPIO_PIN_MAP(0, 22)
// #define NRFX_QSPI_PIN_IO3                NRF_GPIO_PIN_MAP(0, 23)
//#define FLASH_EN_PIN                     NRF_GPIO_PIN_MAP(0,16)

// Wrapper
#define GPIO_INTERRUPT_CONFIG_RISING  GPIOTE_CONFIG_IN_SENSE_LOTOHI(true)
#define GPIO_INTERRUPT_CONFIG_FALLING GPIOTE_CONFIG_IN_SENSE_HITOLO(true)
#define GPIO_INTERRUPT_CONFIG_TOGGLE  GPIOTE_CONFIG_IN_SENSE_TOGGLE(true)

#define gpioPin_t                            nrfx_gpiote_pin_t
#define gpioOutput_t                         nrf_drv_gpiote_out_config_t

#define gpioInput_t                          nrf_drv_gpiote_in_config_t
#define gpioInputEnable(pin, config, handler)  nrf_drv_gpiote_in_init(pin, config, handler)
#define gpioInterruptEnable(pin)             nrf_drv_gpiote_in_event_enable(pin, true)
#define gpioInterruptDisable(pin)            nrf_drv_gpiote_in_event_disable(pin)
#define gpioRead(pin)                        nrf_gpio_pin_read(pin)

void gpioInit(void);
void gpioOutputEnable(gpioPin_t pin);
void gpioDisable(gpioPin_t pin);
void gpioWrite(gpioPin_t pin, uint8_t value);