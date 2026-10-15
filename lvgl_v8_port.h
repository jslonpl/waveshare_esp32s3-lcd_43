/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#pragma once

#include "sdkconfig.h"

#ifdef CONFIG_ARDUINO_RUNNING_CORE
#include <Arduino.h>
#endif

#include "esp_display_panel.hpp"
#include "lvgl.h"

// Okres aktualizacji zegara LVGL.
#define LVGL_PORT_TICK_PERIOD_MS                (2)

// Bufory używane, gdy ochrona przed tearingiem jest wyłączona.
#define LVGL_PORT_BUFFER_MALLOC_CAPS \
    (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)

#define LVGL_PORT_BUFFER_SIZE_HEIGHT            (20)
#define LVGL_PORT_BUFFER_NUM                    (2)

// Parametry zadania LVGL.
#define LVGL_PORT_TASK_MAX_DELAY_MS              (500)
#define LVGL_PORT_TASK_MIN_DELAY_MS              (2)
#define LVGL_PORT_TASK_STACK_SIZE                (6 * 1024)
#define LVGL_PORT_TASK_PRIORITY                  (2)

// W Arduino IDE ustaw: Arduino Runs On -> Core 1.
// Zadanie ADC jest przypisane osobno do rdzenia 0.
#ifdef ARDUINO_RUNNING_CORE
#define LVGL_PORT_TASK_CORE                     (ARDUINO_RUNNING_CORE)
#else
#define LVGL_PORT_TASK_CORE                     (1)
#endif

/*
 * Ochrona przed tearingiem:
 *
 * 0: wyłączona
 * 1: dwa bufory LCD, LVGL full-refresh
 * 2: trzy bufory LCD, LVGL full-refresh
 * 3: dwa bufory LCD, LVGL direct-mode
 *
 * Dotyczy interfejsu RGB; wymaga LVGL >= 8.3.9.
 */
#ifdef CONFIG_LVGL_PORT_AVOID_TEARING_MODE
#define LVGL_PORT_AVOID_TEARING_MODE \
    (CONFIG_LVGL_PORT_AVOID_TEARING_MODE)
#else
#define LVGL_PORT_AVOID_TEARING_MODE             (3)
#endif

#if LVGL_PORT_AVOID_TEARING_MODE != 0

/*
 * Przy włączonej ochronie przed tearingiem nie używaj
 * lv_disp_set_rotation(). Obrót ustawiany jest tutaj.
 */
#ifdef CONFIG_LVGL_PORT_ROTATION_DEGREE
#define LVGL_PORT_ROTATION_DEGREE \
    (CONFIG_LVGL_PORT_ROTATION_DEGREE)
#else
#define LVGL_PORT_ROTATION_DEGREE                (0)
#endif

#define LVGL_PORT_AVOID_TEAR                     (1)

// Konfiguracja buforów odpowiednia dla wybranego trybu.
#if LVGL_PORT_AVOID_TEARING_MODE == 1

#define LVGL_PORT_DISP_BUFFER_NUM                (2)
#define LVGL_PORT_FULL_REFRESH                   (1)

#elif LVGL_PORT_AVOID_TEARING_MODE == 2

#define LVGL_PORT_DISP_BUFFER_NUM                (3)
#define LVGL_PORT_FULL_REFRESH                   (1)

#elif LVGL_PORT_AVOID_TEARING_MODE == 3

#define LVGL_PORT_DISP_BUFFER_NUM                (2)
#define LVGL_PORT_DIRECT_MODE                    (1)

#else

#error "Nieprawidlowy LVGL_PORT_AVOID_TEARING_MODE"

#endif

// Weryfikacja kąta obrotu.
#if (LVGL_PORT_ROTATION_DEGREE != 0) && \
    (LVGL_PORT_ROTATION_DEGREE != 90) && \
    (LVGL_PORT_ROTATION_DEGREE != 180) && \
    (LVGL_PORT_ROTATION_DEGREE != 270)

#error "LVGL_PORT_ROTATION_DEGREE musi wynosic 0, 90, 180 lub 270"

#elif LVGL_PORT_ROTATION_DEGREE != 0

// Obrót przy włączonej ochronie przed tearingiem wymaga trzech buforów.
#undef LVGL_PORT_DISP_BUFFER_NUM
#define LVGL_PORT_DISP_BUFFER_NUM                (3)

#endif // Sprawdzenie kąta obrotu

#endif // LVGL_PORT_AVOID_TEARING_MODE != 0

// Funkcje zaimplementowane w lvgl_v8_port.cpp.
#ifdef __cplusplus
extern "C" {
#endif

/**
 * Inicjalizuje port LVGL.
 * lcd: wskaźnik do zainicjalizowanego wyświetlacza.
 * tp: wskaźnik do panelu dotykowego lub nullptr.
 */
bool lvgl_port_init(
    esp_panel::drivers::LCD *lcd,
    esp_panel::drivers::Touch *tp
);

/**
 * Zwalnia zasoby portu LVGL.
 */
bool lvgl_port_deinit(void);

/**
 * Blokuje mutex LVGL.
 * timeout_ms = -1 oznacza oczekiwanie bez limitu.
 */
bool lvgl_port_lock(int timeout_ms);

/**
 * Zwalnia mutex LVGL.
 */
bool lvgl_port_unlock(void);

/**
 * Budzi zadanie LVGL po zmianie obiektow przez zadanie aplikacji.
 * Uzywa osobnego semafora, poniewaz powiadomienia zadania obsluguja VSYNC.
 */
void lvgl_port_request_refresh(void);

#ifdef __cplusplus
}
#endif
