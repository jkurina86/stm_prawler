/** @file config.c @brief State for the shell/LI-COR branch. */
#include "config.h"

app_state_t g_app;

void config_init(void)
{
    g_app = (app_state_t){
        .sd_status = PERIPH_OFF,
        .rtc_status = PERIPH_OFF,
        .licor_status = PERIPH_OFF
    };
}
