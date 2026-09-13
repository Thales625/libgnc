#pragma once

#ifdef ESP_PLATFORM
    #include <esp_log.h>
    #include <cstdlib>

    #define LIBGNC_ASSERT(cond) \
        do { \
            if (!(cond)) { \
                ESP_LOGE("LIBGNC", "Assertion failed: %s (%s:%d)", #cond, __FILE__, __LINE__); \
                abort(); \
            } \
        } while (0)

#else

    #include <cassert>
    #define LIBGNC_ASSERT(cond) assert(cond)

#endif