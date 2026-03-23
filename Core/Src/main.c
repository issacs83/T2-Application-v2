/*
 * main.c : Application entry point (Core layer)
 *
 * Copyright (C) 2016-2026 Osstem Implant, Inc
 *
 * This file contains ONLY the main() function which calls
 * App_Init() and App_Run(). All application logic resides
 * in the App layer.
 *
 * STM32CubeIDE auto-generated content (clock config, etc.) is
 * handled by System_Init() called from App_Init().
 */

#include "stm32f2xx.h"
#include "app_main.h"

int main(void)
{
    App_Init();
    App_Run();

    /* Never reached */
    while (1) {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
    while (1) {
    }
}
#endif

void __assert_func(const char *file, int line, const char *func,
                    const char *failedexpr)
{
    (void)file;
    (void)line;
    (void)func;
    (void)failedexpr;
    while (1) {
    }
}

void __assert(const char *file, int line, const char *failedexpr)
{
    __assert_func(file, line, (void*)0, failedexpr);
}

#ifdef USE_SEE
#ifndef USE_DEFAULT_TIMEOUT_CALLBACK
uint32_t sEE_TIMEOUT_UserCallback(void)
{
    return 1; /* sEE_FAIL */
}
#endif
#endif
