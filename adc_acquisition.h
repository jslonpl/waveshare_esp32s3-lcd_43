#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include "adc_module.h"
#include "waveform_processing.h"

namespace acquisition {

constexpr BaseType_t kCore = 0;
constexpr UBaseType_t kPriority = 3;
constexpr uint32_t kStackBytes = 4096;
constexpr uint32_t kMaxGapUs = 5000;
constexpr unsigned kPoolSize = 3;

struct Diagnostics {
    uint32_t gapMaxUs = 0;
    uint32_t gapIndex = 0;
    uint32_t betweenReadsMaxUs = 0;

    uint32_t readMaxUs = 0;
    uint32_t writeMaxUs = 0;
    uint32_t pollMaxUs = 0;
    uint32_t resultMaxUs = 0;
    uint32_t waitMaxUs = 0;

    uint32_t elapsedUs = 0;
    uint32_t stackFreeBytes = 0;
    uint64_t gapTotalUs = 0;
    uint32_t gapCount = 0;
    uint32_t readyHighWater = 0;
    uint32_t errorCount = 0;
    uint32_t missedConversions = 0;

    void add(const AdcReadTiming &timing) {
        if (timing.totalUs > readMaxUs) {
            readMaxUs = timing.totalUs;
        }
        if (timing.writeUs > writeMaxUs) {
            writeMaxUs = timing.writeUs;
        }
        if (timing.pollMaxUs > pollMaxUs) {
            pollMaxUs = timing.pollMaxUs;
        }
        if (timing.resultUs > resultMaxUs) {
            resultMaxUs = timing.resultUs;
        }
        if (timing.waitUs > waitMaxUs) {
            waitMaxUs = timing.waitUs;
        }
        if (timing.notifications > 1) {
            missedConversions += timing.notifications - 1;
        }
    }
};

struct Block {
    waveform::Sample samples[adc_config::capacity];

    size_t count = 0;
    uint32_t sequence = 0;
    uint32_t droppedBefore = 0;
    uint32_t completedMs = 0;
    uint64_t completedUs = 0;

    esp_err_t error = ESP_OK;
    Diagnostics diagnostic;
};

// Obiekt Service musi mieć czas życia obejmujący całe działanie programu.
// W pliku .ino używamy: static acquisition::Service sampler;
class Service {
public:
    bool begin() {
        if (task_ != nullptr) {
            return false;
        }

        free_ = xQueueCreate(kPoolSize, sizeof(Block *));
        ready_ = xQueueCreate(kPoolSize - 1, sizeof(Block *));

        if (free_ == nullptr || ready_ == nullptr) {
            cleanup();
            return false;
        }

        for (auto &block : pool_) {
            Block *pointer = &block;

            if (xQueueSend(free_, &pointer, 0) != pdTRUE) {
                cleanup();
                return false;
            }
        }

        const BaseType_t result = xTaskCreatePinnedToCore(
            entry,
            "adc_capture",
            kStackBytes,
            this,
            kPriority,
            &task_,
            kCore
        );

        if (result != pdPASS) {
            cleanup();
            return false;
        }

        return true;
    }

    // Wywołuje tylko jeden odbiorca: Arduino loop().
    Block *take(TickType_t timeout) {
        Block *block = nullptr;

        if (xQueueReceive(ready_, &block, timeout) != pdTRUE) {
            return nullptr;
        }

        // Preferuj nowszy dostępny blok.
        return block;
    }

    void release(Block *block) {
        configASSERT(block != nullptr);

        const BaseType_t result = xQueueSend(free_, &block, 0);
        configASSERT(result == pdTRUE);
    }

private:
    Block pool_[kPoolSize];

    QueueHandle_t free_ = nullptr;
    QueueHandle_t ready_ = nullptr;
    TaskHandle_t task_ = nullptr;

    AdcModule adc_;

    uint32_t sequence_ = 0;
    uint32_t dropped_ = 0;
    uint32_t errorCount_ = 0;
    uint32_t readyHighWater_ = 0;

    // Używane wyłącznie przy nieudanej inicjalizacji.
    void cleanup() {
        if (free_ != nullptr) {
            vQueueDelete(free_);
        }

        if (ready_ != nullptr) {
            vQueueDelete(ready_);
        }

        free_ = nullptr;
        ready_ = nullptr;
    }

    static void entry(void *context) {
        static_cast<Service *>(context)->run();
    }

    void run() {
        bool adcReady = false;

        while (true) {
            Block *block = nullptr;

            if (xQueueReceive(free_, &block, 0) != pdTRUE) {
                // Odzyskaj starszy blok z kolejki.
                // Nie dotykamy bloku aktualnie używanego przez odbiorcę.
                if (xQueueReceive(ready_, &block, 0) == pdTRUE) {
                    ++dropped_;
                } else {
                    vTaskDelay(1);
                    continue;
                }
            }

            block->count = 0;
            block->error = ESP_OK;
            block->diagnostic = Diagnostics{};
            block->sequence = ++sequence_;

            const uint32_t start = micros();

            if (!adcReady) {
                if (!adc_.begin()) {
                    block->error = adc_.lastError();
                    ++errorCount_;
                } else {
                    adcReady = true;
                }
            }
            uint32_t previousEnd = 0;

            while (
                block->error == ESP_OK &&
                block->count < adc_config::capacity &&
                uint32_t(micros() - start) < adc_config::acquisitionChunkUs
            ) {
                const uint32_t before = micros();

                if (block->count > 0) {
                    const uint32_t idle =
                        uint32_t(before - previousEnd);

                    if (
                        idle >
                        block->diagnostic.betweenReadsMaxUs
                    ) {
                        block->diagnostic.betweenReadsMaxUs = idle;
                    }
                }

                waveform::Sample &sample =
                    block->samples[block->count];

                const bool ok = adc_.read(
                    sample.volts,
                    sample.us
                );

                previousEnd = micros();
                block->diagnostic.add(adc_.timing());

                if (!ok) {
                    block->error = adc_.lastError();
                    ++errorCount_;
                    adcReady = false;
                    break;
                }

                if (block->count > 0) {
                    const uint32_t gap = uint32_t(
                        sample.us -
                        block->samples[block->count - 1].us
                    );

                    if (gap > block->diagnostic.gapMaxUs) {
                        block->diagnostic.gapMaxUs = gap;
                        block->diagnostic.gapIndex =
                            static_cast<uint32_t>(block->count);
                    }
                    block->diagnostic.gapTotalUs += gap;
                    ++block->diagnostic.gapCount;
                }

                ++block->count;
            } // Koniec zbierania próbek do bloku.

            block->diagnostic.elapsedUs =
                uint32_t(micros() - start);

            block->diagnostic.stackFreeBytes =
                uxTaskGetStackHighWaterMark(nullptr);
            block->diagnostic.errorCount = errorCount_;

            block->droppedBefore = dropped_;
            block->completedMs = millis();
            block->completedUs = esp_timer_get_time();

            const uint32_t waitingReady = uxQueueMessagesWaiting(ready_);
            const uint32_t projectedReady = waitingReady < kPoolSize - 1
                ? waitingReady + 1
                : kPoolSize - 1;
            if (projectedReady > readyHighWater_) {
                readyHighWater_ = projectedReady;
            }
            block->diagnostic.readyHighWater = readyHighWater_;

            // Zapamiętaj przed przekazaniem własności bloku.
            const bool failed = block->error != ESP_OK;

            if (xQueueSend(ready_, &block, 0) != pdTRUE) {
                Block *obsolete = nullptr;
                if (xQueueReceive(ready_, &obsolete, 0) == pdTRUE) {
                    ++dropped_;
                    release(obsolete);
                }
                if (xQueueSend(ready_, &block, 0) != pdTRUE) {
                    ++dropped_;
                    release(block);
                }
            }

            // Od tego miejsca nie wolno odczytywać ani zmieniać bloku.
            // Odbiorca lub kolejka wolnych buforów przejęły jego własność.

            if (failed) {
                const TickType_t retryTicks = pdMS_TO_TICKS(250);
                vTaskDelay(retryTicks > 0 ? retryTicks : 1);
            } else {
                // Oddanie czasu procesora między blokami pomiarowymi.
            }
        } // Koniec while (true).
    } // Koniec run().
}; // Koniec klasy Service.

} // namespace acquisition
