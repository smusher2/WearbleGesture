#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "gesture.h"
#include <string.h>
#include <stdio.h>

char uart_buf[192];

void SystemClock_Config(void);
void Error_Handler(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_USART1_UART_Init();

    // Target >=100 Hz sampling for reliable dynamics
    const uint32_t SAMPLE_HZ = 100u;   // 10 ms
    const uint32_t SAMPLE_MS = 1000u / SAMPLE_HZ;

    snprintf(uart_buf, sizeof(uart_buf), "{status:ready, ms:%u}\r\n", (unsigned)HAL_GetTick());
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);


    int16_t ax, ay, az, gx, gy, gz;

    while (1) {
        // Replace with real sensor reads in production
        imu_generate_sim(&ax, &ay, &az, &gx, &gy, &gz);

        gesture_event_t evt = gesture_process(ax, ay, az, gx, gy, gz);

        if (strcmp(evt.gesture, "none") != 0) {

            snprintf(uart_buf, sizeof(uart_buf),
                     "{gesture:%s, conf:%d, gy:%d, ts:%lu}\r\n",
                     evt.gesture, evt.confidence, evt.gy, evt.timestamp);
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
        }


        HAL_Delay(SAMPLE_MS);
    }
}

void SystemClock_Config(void) {}
void Error_Handler(void) { while (1) {} }
