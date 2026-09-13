#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/task.h"

#include "esp_log.h"

#include "libgnc/math/vector.hpp"
#include "libgnc/math/matrix.hpp"
#include "libgnc/navigation/kalman/kalman.hpp"

static const char *TAG = "GNC_LIB";

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Initializing C++ GNC lib debugging...");

    // gnc::navigation::KalmanFilter<float, 2, 1, 0> kalman;
    gnc::Vec3f vel;

    while (true) {
        vel[0] += 1.0f;

        vel.print();

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}