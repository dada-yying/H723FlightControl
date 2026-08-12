#ifndef APP_MAIN_H
#define APP_MAIN_H

/*
 * Application entry points for the template.
 * Add board drivers, modules, and FreeRTOS task creation from App_Main_Start().
 */

#ifdef __cplusplus
extern "C" {
#endif

void App_Main_Init(void);
void App_Main_Start(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_MAIN_H */
