#pragma once

#include <lvgl.h>
#include <cmath>
#include <cstdio>
#include <cstring>

#if LVGL_VERSION_MAJOR != 8
#error "Ten projekt wymaga LVGL 8.4.0 z dolaczonej paczki."
#endif

// Widok pomiarowy. Geometria, czcionki i kolory zachowane z projektu uzytkownika.
namespace sine_view {
constexpr size_t kDisplayedPeriods = 3;
constexpr int kHorizontalDivisions = int(kDisplayedPeriods) * 2;
constexpr float kAxisLimitVolts = 400.0f;
constexpr int kScreenHeight = 480;
constexpr int kOuterMargin = 12;
constexpr int kGraphOffsetY = 22;
constexpr int kLeft = 80;
constexpr int kTop = 110 + kGraphOffsetY;
constexpr int kWidth = 680;
constexpr int kHeight = 280;
// Jeden punkt na piksel zapobiega widocznej redukcji rozdzielczosci wykresu.
// Wygladzenie pomiedzy probkami ADC wykonuje zadanie przetwarzania, poza LVGL.
constexpr int kPointStride = 1;
constexpr int kPointCount = kWidth / kPointStride + 1;
static_assert(kWidth % kPointStride == 0, "Curve width must match point stride");
constexpr int kHeaderX = 24;
constexpr int kHeaderY = 4;
constexpr int kHeaderWidth = 736;
constexpr int kHeaderHeight = 80;
constexpr int kHeaderColumnWidth = 245;
// Montserrat 26 ma dla wielkich liter i cyfr glif wysokosci 18 px,
// przesuniety o 6 px wzgledem gornej krawedzi etykiety. Te pozycje daja
// wizualnie 14 px nad tytulem, 15 px miedzy tytulem i wartoscia oraz 14 px
// pod cyframi (do kolorowej dolnej krawedzi panelu).
constexpr int kHeaderTitleY = 12;
constexpr int kHeaderValueY = 44;
static lv_obj_t *curve = nullptr;
constexpr size_t kMetricCharacterCount = 5;
struct MetricValue {
    lv_obj_t *characters[kMetricCharacterCount]{};
    size_t characterCount = 0;
};
static MetricValue rmsValue;
static MetricValue maximumValue;
static MetricValue frequencyValue;
static lv_obj_t *rmsLine = nullptr;
static lv_obj_t *rmsTag = nullptr;
static lv_obj_t *timeTicks[kHorizontalDivisions + 1]{};
static lv_point_t rmsLinePoints[2];
static lv_point_t points[kPointCount]; // LVGL zachowuje wskaznik: tablica musi zyc stale.
static float displayedAxisFrequencyHz = 0.0f;
constexpr int kRmsTagWidth = 64;
constexpr int kRmsTagHeight = 26;
constexpr int kRmsTagX = kLeft + kWidth - kRmsTagWidth;

inline int voltageToY(float volts) {
    return kTop + static_cast<int>(std::lround(
        (kAxisLimitVolts - volts) * kHeight / (2.0f * kAxisLimitVolts)));
}

inline lv_obj_t *label(lv_obj_t *parent, int x, int y, const char *text,
                  uint32_t color = 0xDCE6F0) {
    lv_obj_t *obj = lv_label_create(parent);
    lv_label_set_text(obj, text); // LVGL kopiuje tekst z bufora lokalnego.
    lv_obj_set_style_text_font(obj, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(obj, lv_color_hex(color), 0);
    lv_obj_set_pos(obj, x, y);
    return obj;
}

inline void setMetricValue(MetricValue &value, const char *text) {
    const size_t textLength = std::strlen(text);
    const size_t visibleLength = textLength < value.characterCount
        ? textLength
        : value.characterCount;
    const size_t sourceOffset = textLength - visibleLength;
    const size_t destinationOffset = value.characterCount - visibleLength;

    for (size_t i = 0; i < value.characterCount; ++i) {
        char character[2] = {' ', '\0'};
        if (i >= destinationOffset) {
            character[0] = text[sourceOffset + i - destinationOffset];
        }
        lv_label_set_text(value.characters[i], character);
    }
}

inline MetricValue metricValueLabel(
    lv_obj_t *parent,
    const char *title,
    int columnX,
    int columnWidth,
    int valueWidth,
    size_t characterCount,
    const char *unit,
    const lv_font_t *unitFont
) {
    constexpr int kUnitGap = 6;
    const int unitWidth = lv_txt_get_width(
        unit, std::strlen(unit), unitFont, 0,
        LV_TEXT_FLAG_NONE);
    const int groupWidth = valueWidth + kUnitGap + unitWidth;
    const int valueX = columnX + (columnWidth - groupWidth) / 2;
    const int unitX = valueX + valueWidth + kUnitGap;
    const int unitY = kHeaderValueY +
        (lv_font_montserrat_26.line_height - unitFont->line_height) / 2;
    lv_obj_t *titleLabel = label(
        parent, columnX + 14, kHeaderTitleY, title, 0x9CAEBF);
    lv_obj_t *unitLabel = label(
        parent, unitX, unitY, unit, 0x9CAEBF);
    MetricValue value;
    value.characterCount = characterCount;

    lv_obj_set_style_text_font(titleLabel, &lv_font_montserrat_26, 0);
    lv_obj_set_style_text_font(unitLabel, unitFont, 0);

    // Kazdy znak ma stale pole. Zmiana szerokosci glifu (np. 1 -> 8) nie
    // przesuwa pozostalych cyfr ani separatora dziesietnego.
    for (size_t i = 0; i < value.characterCount; ++i) {
        const int left = valueX + int(i) * valueWidth / int(value.characterCount);
        const int right = valueX + int(i + 1) * valueWidth / int(value.characterCount);
        value.characters[i] = label(
            parent, left, kHeaderValueY, " ", 0xF4F7FA);
        lv_obj_set_width(value.characters[i], right - left);
        lv_label_set_long_mode(value.characters[i], LV_LABEL_LONG_CLIP);
        lv_obj_set_style_text_font(value.characters[i], &lv_font_montserrat_26, 0);
        lv_obj_set_style_text_align(value.characters[i], LV_TEXT_ALIGN_CENTER, 0);
    }
    setMetricValue(value, "--");
    return value;
}

inline void setMetrics(
    const char *rmsText,
    const char *maximumText,
    const char *frequencyText
) {
    setMetricValue(rmsValue, rmsText);
    setMetricValue(maximumValue, maximumText);
    setMetricValue(frequencyValue, frequencyText);
}

// Wykres zawsze pokazuje trzy pelne okresy. Skala czasu wynika z aktualnie
// zmierzonej czestotliwosci; ograniczenie aktualizacji zapobiega migotaniu
// etykiet przy niewielkim szumie estymacji czestotliwosci.
inline void updateTimeAxis(float frequencyHz, bool force = false) {
    if (!(frequencyHz > 0.0f) || !std::isfinite(frequencyHz)) return;
    if (!force && std::fabs(frequencyHz - displayedAxisFrequencyHz) < 0.25f) {
        return;
    }

    const float periodMs = 1000.0f / frequencyHz;
    char text[16];
    for (int tick = 0; tick <= kHorizontalDivisions; ++tick) {
        const float timeMs = periodMs * float(tick) *
            float(kDisplayedPeriods) / float(kHorizontalDivisions);
        std::snprintf(text, sizeof(text), "%.1f", timeMs);
        lv_label_set_text(timeTicks[tick], text);
    }
    displayedAxisFrequencyHz = frequencyHz;
}

inline void bar(lv_obj_t *parent, int x, int y, int width, int height,
                uint32_t color) {
    lv_obj_t *obj = lv_obj_create(parent);
    lv_obj_remove_style_all(obj);
    lv_obj_set_pos(obj, x, y);
    lv_obj_set_size(obj, width, height);
    lv_obj_set_style_bg_color(obj, lv_color_hex(color), 0);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
}

// Wywolaj raz, po inicjalizacji LVGL, trzymajac mutex lvgl_port_lock().
inline void create() {
    lv_obj_t *screen = lv_scr_act();
    lv_obj_clean(screen);
    lv_obj_clear_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x101923), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(screen, 0, 0);

    char text[110];
    // Jeden panel porzadkuje pomiary w trzy stale kolumny. Caly wykres wraz
    // z etykietami jest przesuniety w dol o wspolny, staly offset.
    lv_obj_t *header = lv_obj_create(screen);
    lv_obj_remove_style_all(header);
    lv_obj_set_pos(header, kHeaderX, kHeaderY);
    lv_obj_set_size(header, kHeaderWidth, kHeaderHeight);
    lv_obj_set_style_bg_color(header, lv_color_hex(0x17232E), 0);
    lv_obj_set_style_bg_opa(header, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(header, lv_color_hex(0x344351), 0);
    lv_obj_set_style_border_width(header, 1, 0);
    lv_obj_set_style_radius(header, 6, 0);
    lv_obj_clear_flag(header, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    const int secondColumnX = kHeaderX + kHeaderColumnWidth;
    const int thirdColumnX = secondColumnX + kHeaderColumnWidth;
    bar(screen, secondColumnX, kHeaderY + 8, 1, kHeaderHeight - 16, 0x344351);
    bar(screen, thirdColumnX, kHeaderY + 8, 1, kHeaderHeight - 16, 0x344351);
    bar(screen, kHeaderX + 1, kHeaderY + kHeaderHeight - 3,
        kHeaderColumnWidth - 2, 2, 0xC4515D);
    bar(screen, secondColumnX + 1, kHeaderY + kHeaderHeight - 3,
        kHeaderColumnWidth - 2, 2, 0xFFD700);
    bar(screen, thirdColumnX + 1, kHeaderY + kHeaderHeight - 3,
        kHeaderWidth - 2 * kHeaderColumnWidth - 2, 2, 0x58B7C6);

    // Opis, stale pola znakow i jednostka sa osobnymi obiektami. Aktualizacja
    // cyfry nie zmienia polozenia pozostalych znakow, opisu ani jednostki.
    rmsValue = metricValueLabel(
        screen, "U RMS", kHeaderX, kHeaderColumnWidth, 85, 5,
        "V", &lv_font_montserrat_26);
    maximumValue = metricValueLabel(
        screen, "U max", secondColumnX, kHeaderColumnWidth, 85, 5,
        "V", &lv_font_montserrat_26);
    frequencyValue = metricValueLabel(
        screen, "FREQ", thirdColumnX,
        kHeaderWidth - 2 * kHeaderColumnWidth, 68, 4,
        "Hz", &lv_font_montserrat_26);
    label(screen, 24, 78 + kGraphOffsetY, "u [V]");

    // Siatka 100 V oraz 0,5 okresu. Linie osi sa jasniejsze i grubsze.
    for (int volts = -400; volts <= 400; volts += 100) {
        const int y = voltageToY(static_cast<float>(volts));
        bar(screen, kLeft, y, kWidth + 1, volts == 0 ? 2 : 1,
            volts == 0 ? 0xCDD9E5 : 0x344351);
        std::snprintf(text, sizeof(text), "%d", volts);
        label(screen, 27, y - 8, text);
    }
    for (int tick = 0; tick <= kHorizontalDivisions; ++tick) {
        const int x = kLeft + tick * kWidth / kHorizontalDivisions;
        bar(screen, x, kTop, tick == 0 ? 2 : 1, kHeight + 1,
            tick == 0 ? 0xCDD9E5 : 0x344351);
        timeTicks[tick] = label(screen, x - 16, kTop + kHeight + 12, "--");
    }
    const int timeAxisLabelY =
        kScreenHeight - kOuterMargin - lv_font_montserrat_16.line_height;
    label(screen, 704, timeAxisLabelY, "t [ms]");
    updateTimeAxis(50.0f, true);

    // Linia aktualnego U RMS lezy pod sinusoida. Dzieki kolejnosci tworzenia
    // zolta krzywa zachowuje swoj kolor w miejscach przeciecia.
    const lv_coord_t initialRmsY = static_cast<lv_coord_t>(voltageToY(0));
    rmsLinePoints[0] = {
        static_cast<lv_coord_t>(kLeft),
        initialRmsY
    };
    rmsLinePoints[1] = {
        static_cast<lv_coord_t>(kRmsTagX),
        initialRmsY
    };
    rmsLine = lv_line_create(screen);
    lv_obj_set_pos(rmsLine, 0, 0);
    lv_line_set_points(rmsLine, rmsLinePoints, 2);
    lv_obj_set_style_line_color(rmsLine, lv_color_hex(0xC4515D), 0);
    lv_obj_set_style_line_width(rmsLine, 1, 0);
    lv_obj_add_flag(rmsLine, LV_OBJ_FLAG_HIDDEN);

    // Stable storage for LVGL. Hidden until the first valid acquisition.
    for (int i = 0; i < kPointCount; ++i) {
        points[i].x = static_cast<lv_coord_t>(kLeft + i * kPointStride);
        points[i].y = static_cast<lv_coord_t>(voltageToY(0));
    }
    curve = lv_line_create(screen);
    lv_obj_set_pos(curve, 0, 0);
    lv_line_set_points(curve, points, kPointCount);
    lv_obj_set_style_line_color(curve, lv_color_hex(0xFFD700), 0);
    lv_obj_set_style_line_width(curve, 3, 0);
    // Zaokraglanie kazdego z setek polaczen jest kosztowne programowo, a przy
    // tej gestosci punktow nie daje widocznej korzysci na panelu 800x480.
    lv_obj_set_style_line_rounded(curve, true, 0);
    lv_obj_add_flag(curve, LV_OBJ_FLAG_HIDDEN);

    // Etykieta zamyka prawy koniec linii U RMS i porusza sie razem z nia.
    // Obiekt powstaje tylko raz; kolejne klatki zmieniaja jedynie jego Y.
    rmsTag = lv_obj_create(screen);
    lv_obj_remove_style_all(rmsTag);
    lv_obj_set_pos(
        rmsTag,
        kRmsTagX,
        static_cast<int>(initialRmsY) - kRmsTagHeight / 2
    );
    lv_obj_set_size(rmsTag, kRmsTagWidth, kRmsTagHeight);
    lv_obj_set_style_bg_color(rmsTag, lv_color_hex(0xC4515D), 0);
    lv_obj_set_style_bg_opa(rmsTag, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(rmsTag, 0, 0);
    lv_obj_set_style_radius(rmsTag, 0, 0);
    lv_obj_clear_flag(rmsTag, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *rmsTagLabel = lv_label_create(rmsTag);
    lv_label_set_text(rmsTagLabel, "U RMS");
    lv_obj_set_style_text_font(rmsTagLabel, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_color(rmsTagLabel, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(rmsTagLabel);
    lv_obj_add_flag(rmsTag, LV_OBJ_FLAG_HIDDEN);
}
// Call only while holding lvgl_port_lock(). Updates objects, never recreates axes.
inline void update(
    const float *volts,
    const char *rmsText,
    const char *maximumText,
    const char *frequencyText
) {
    lv_obj_invalidate(curve);
    for (int i=0; i<kPointCount; ++i) {
        const float clipped = fmaxf(-kAxisLimitVolts, fminf(kAxisLimitVolts, volts[i]));
        points[i].y = static_cast<lv_coord_t>(voltageToY(clipped));
    }
    lv_line_set_points(curve, points, kPointCount);
    lv_obj_clear_flag(curve, LV_OBJ_FLAG_HIDDEN);
    lv_obj_invalidate(curve);
    setMetrics(rmsText, maximumText, frequencyText);
}
// Call only while holding lvgl_port_lock(). Coordinates are prepared by the
// processing task, so the LVGL critical section performs only a bounded copy.
inline void updatePrepared(
    const int16_t *preparedY,
    int16_t rmsY,
    const char *rmsText,
    const char *maximumText,
    const char *frequencyText,
    float frequencyHz
) {
    lv_obj_invalidate(rmsLine);
    rmsLinePoints[0].y = static_cast<lv_coord_t>(rmsY);
    rmsLinePoints[1].y = static_cast<lv_coord_t>(rmsY);
    lv_line_set_points(rmsLine, rmsLinePoints, 2);
    lv_obj_clear_flag(rmsLine, LV_OBJ_FLAG_HIDDEN);
    lv_obj_invalidate(rmsLine);
    lv_obj_set_y(rmsTag, static_cast<int>(rmsY) - kRmsTagHeight / 2);
    lv_obj_clear_flag(rmsTag, LV_OBJ_FLAG_HIDDEN);
    lv_obj_invalidate(curve);
    for (int i=0; i<kPointCount; ++i) {
        points[i].y = static_cast<lv_coord_t>(preparedY[i]);
    }
    lv_line_set_points(curve, points, kPointCount);
    lv_obj_clear_flag(curve, LV_OBJ_FLAG_HIDDEN);
    lv_obj_invalidate(curve);
    setMetrics(rmsText, maximumText, frequencyText);
    updateTimeAxis(frequencyHz);
}
} // namespace sine_view
