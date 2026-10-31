#pragma once
#include <Arduino.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <freertos/queue.h>
#include "adc_config.h"

struct AdcReadTiming {
    uint32_t writeUs=0, pollMaxUs=0, resultUs=0;
    uint32_t waitUs=0, totalUs=0;
    uint32_t notifications=0;
};

// Sole owner: acquisition task. Reuses I2C host initialized by Board.
class AdcModule {
public:
    esp_err_t lastError() const { return error_; }
    const AdcReadTiming &timing() const { return timing_; }
    bool begin() {
        error_ = ESP_OK;

        if (cadenceQueue_ == nullptr) {
            cadenceQueue_ = xQueueCreateStatic(
                1,
                sizeof(CadenceEvent),
                cadenceQueueBuffer_,
                &cadenceQueueStorage_
            );
            if (cadenceQueue_ == nullptr) {
                error_ = ESP_ERR_NO_MEM;
                return false;
            }
        }

        if (timer_ != nullptr) {
            const esp_err_t stopped = esp_timer_stop(timer_);
            if (stopped != ESP_OK && stopped != ESP_ERR_INVALID_STATE) {
                error_ = stopped;
                return false;
            }
        }

        uint16_t detectedConfig;
        if (!readRegister(1, detectedConfig)) return false;

        // AIN0 to GND, +/-6.144 V, continuous mode, comparator disabled.
        const uint16_t config =
            0xC003 | (adc_config::ads1015 ? 0x00C0 : 0x00E0);
        if (!writeRegister(1, config)) return false;
        if (!setPointer(0)) return false;

        if (timer_ == nullptr) {
            const esp_timer_create_args_t args = {
                &AdcModule::timerCallback,
                this,
                ESP_TIMER_TASK,
                "adc_cadence",
                true
            };
            error_ = esp_timer_create(&args, &timer_);
            if (error_ != ESP_OK) return false;
        }

        xQueueReset(cadenceQueue_);
        lastReadSequence_ = cadenceSequence_;
        error_ = esp_timer_start_periodic(
            timer_,
            adc_config::conversionPeriodUs
        );
        return error_ == ESP_OK;
    }
    bool read(float &volts,uint32_t &timeUs) {
        timing_=AdcReadTiming{};
        const uint32_t callStart=micros();
        // Record elapsed wall time even on an early error return.
        struct Finish {
            uint32_t start;
            AdcReadTiming &timing;
            ~Finish() { timing.totalUs=uint32_t(micros()-start); }
        } finish{callStart,timing_};
        const uint32_t waitStart=micros();
        CadenceEvent event{};
        const bool cadenceReady = xQueueReceive(
            cadenceQueue_,
            &event,
            timeoutTicks()
        ) == pdTRUE;
        timing_.waitUs=uint32_t(micros()-waitStart);
        if(!cadenceReady) { error_=ESP_ERR_TIMEOUT; return false; }

        // The queue contains one coherent {timestamp, sequence} snapshot.
        // If the ADC task was delayed, the timer overwrites the stale event
        // with the newest one and the sequence delta exposes skipped periods.
        // This prevents a newer callback from overwriting the timestamp after
        // the wake-up has already been consumed.
        timing_.notifications = uint32_t(
            event.sequence - lastReadSequence_
        );
        lastReadSequence_ = event.sequence;

        // A long I2C transaction can span more than one ADC period. The
        // conversion register then contains the newest complete sample. Keep
        // it, preserve the newest timestamp, and expose the skipped periods in
        // diagnostics instead of turning a recoverable overrun into an error.
        const uint32_t conversionEndUs=event.timeUs;
        timeUs=conversionEndUs-adc_config::conversionPeriodUs/2;
        uint16_t raw;
        const uint32_t t=micros();
        const bool ok=readConversion(raw);
        timing_.resultUs=uint32_t(micros()-t);
        if(!ok) return false;
        int32_t code=(raw & 0x8000) ? int32_t(raw)-65536 : int32_t(raw);
        if(adc_config::ads1015) code/=16;
        volts=code*(adc_config::ads1015 ? 0.003f : 0.0001875f);
        return true;
    }
private:
    struct CadenceEvent {
        uint32_t timeUs;
        uint32_t sequence;
    };

    esp_err_t error_=ESP_OK;
    AdcReadTiming timing_;
    esp_timer_handle_t timer_=nullptr;
    QueueHandle_t cadenceQueue_=nullptr;
    StaticQueue_t cadenceQueueStorage_{};
    uint8_t cadenceQueueBuffer_[sizeof(CadenceEvent)]{};
    uint32_t cadenceSequence_=0;
    uint32_t lastReadSequence_=0;

    static void timerCallback(void *context) {
        auto *self=static_cast<AdcModule *>(context);
        if(self->cadenceQueue_==nullptr) return;
        const CadenceEvent event = {
            micros(),
            ++self->cadenceSequence_
        };
        xQueueOverwrite(self->cadenceQueue_, &event);
    }
    static TickType_t timeoutTicks() {
        const TickType_t ticks=pdMS_TO_TICKS(25);
        return ticks ? ticks : 1;
    }
    bool writeRegister(uint8_t reg,uint16_t value) {
        const uint8_t data[]={reg,uint8_t(value>>8),uint8_t(value)};
        error_=i2c_master_write_to_device(static_cast<i2c_port_t>(adc_config::i2cPort),
                 adc_config::address,data,sizeof(data),timeoutTicks());
        return error_==ESP_OK;
    }
    bool setPointer(uint8_t reg) {
        error_=i2c_master_write_to_device(static_cast<i2c_port_t>(adc_config::i2cPort),
                 adc_config::address,&reg,1,timeoutTicks());
        return error_==ESP_OK;
    }
    bool readConversion(uint16_t &value) {
        uint8_t data[2];
        error_=i2c_master_read_from_device(static_cast<i2c_port_t>(adc_config::i2cPort),
                 adc_config::address,data,2,timeoutTicks());
        if(error_!=ESP_OK) return false;
        value=(uint16_t(data[0])<<8)|data[1];
        return true;
    }
    bool readRegister(uint8_t reg,uint16_t &value) {
        uint8_t data[2];
        error_=i2c_master_write_read_device(static_cast<i2c_port_t>(adc_config::i2cPort),
                 adc_config::address,&reg,1,data,2,timeoutTicks());
        if(error_!=ESP_OK) return false;
        value=(uint16_t(data[0])<<8)|data[1];
        return true;
    }
};
