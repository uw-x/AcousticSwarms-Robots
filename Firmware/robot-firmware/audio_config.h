// PDM
// NRF_DK
//#define MIC_EN_PIN                       NRF_GPIO_PIN_MAP(0, 15)
//#define PDM_CLK_PIN                      NRF_GPIO_PIN_MAP(1,8)
//#define PDM_DATA_PIN                     NRF_GPIO_PIN_MAP(1,9)

// Custom Board
#define MIC_EN_PIN                       NRF_GPIO_PIN_MAP(1, 13)
#define PDM_CLK_PIN                      NRF_GPIO_PIN_MAP(0, 14)
#define PDM_DATA_PIN                     NRF_GPIO_PIN_MAP(0, 16)

#define PDM_BUFFER_DURATION                 (20)

//#define PDM_MAX_BUFFER_LENGTH               (960) // 20ms @ 48kHz
#define PDM_MAX_BUFFER_LENGTH               (2 * (10 + (1240))) // 2channel 20ms @ 62.5kHz

#define AUDIO_QUEUE_MAX_SIZE (60) // 40 ms of audio data at 48kHz