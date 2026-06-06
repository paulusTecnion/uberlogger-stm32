/* app.h — the application state machine (IDLE/CONFIG/LOGGING/SINGLE_SHOT/
 * WAIT_FOR_TRIGGER), trigger debounce, and IDLE command dispatch. */
#ifndef _APP_H
#define _APP_H

void app_init(void);      /* one-time state init (called after MX_*_Init) */
void app_run_once(void);  /* one iteration of the former while(1) body */

#endif
