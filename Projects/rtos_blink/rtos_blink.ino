//#include <wirish/wirish.h>
//#include "libraries/FreeRTOS/MapleFreeRTOS.h"
#include "Arduino.h"
#include <MapleFreeRTOS900.h>



#define BOARD_LED_PIN PC13

static void vLEDFlashTask(void *pvParameters) {
    for (;;) {
        vTaskDelay(1000);
        digitalWrite(BOARD_LED_PIN, HIGH);
        vTaskDelay(500);
        digitalWrite(BOARD_LED_PIN, LOW);
    }
}

void setup() {
    // initialize the digital pin as an output:
    pinMode(BOARD_LED_PIN, OUTPUT);

    xTaskCreate(vLEDFlashTask, "Task1", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 2, NULL);
    vTaskStartScheduler();
}

void loop() {
    // Insert background code here
}


