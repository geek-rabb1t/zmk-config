/*
 * 単4アルカリ乾電池の電圧を分圧抵抗経由で ADC ピンに入力して残量を測るドライバ。
 * ZMK 標準の zmk,battery-voltage-divider はリチウムイオン用の残量計算しか
 * 持たないため、アルカリ乾電池の放電カーブで % を計算する。
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT timothy_battery_alkaline

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(battery_alkaline, CONFIG_SENSOR_LOG_LEVEL);

struct alk_config {
    uint8_t channel;
    uint8_t cells;
    uint32_t output_ohm;
    uint32_t full_ohm;
};

struct alk_data {
    const struct device *adc;
    struct adc_channel_cfg acc;
    struct adc_sequence as;
    int16_t adc_raw;
    uint16_t millivolts;
    uint8_t state_of_charge;
};

/*
 * アルカリ乾電池 1 本あたりの電圧 [mV] と残量 [%] の対応 (軽負荷時の目安)。
 * 電圧の高い順に並べ、間は線形補間する。実機で測って合わない場合はここを調整する。
 */
static const struct {
    uint16_t mv;
    uint8_t pct;
} alkaline_curve[] = {
    {1550, 100}, {1450, 80}, {1380, 60}, {1320, 40}, {1250, 20}, {1150, 10}, {1000, 0},
};

static uint8_t alkaline_mv_to_pct(uint16_t cell_mv) {
    const size_t n = ARRAY_SIZE(alkaline_curve);

    if (cell_mv >= alkaline_curve[0].mv) {
        return 100;
    }
    if (cell_mv <= alkaline_curve[n - 1].mv) {
        return 0;
    }

    for (size_t i = 1; i < n; i++) {
        if (cell_mv >= alkaline_curve[i].mv) {
            uint16_t hi_mv = alkaline_curve[i - 1].mv, lo_mv = alkaline_curve[i].mv;
            uint8_t hi_pct = alkaline_curve[i - 1].pct, lo_pct = alkaline_curve[i].pct;
            return lo_pct + (cell_mv - lo_mv) * (hi_pct - lo_pct) / (hi_mv - lo_mv);
        }
    }
    return 0;
}

static int alk_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct alk_data *data = dev->data;
    const struct alk_config *cfg = dev->config;

    if (chan != SENSOR_CHAN_GAUGE_VOLTAGE && chan != SENSOR_CHAN_GAUGE_STATE_OF_CHARGE &&
        chan != SENSOR_CHAN_ALL) {
        return -ENOTSUP;
    }

    int rc = adc_read(data->adc, &data->as);
    data->as.calibrate = false;
    if (rc != 0) {
        LOG_DBG("Failed to read ADC: %d", rc);
        return rc;
    }

    int32_t mv = MAX(data->adc_raw, 0);
    adc_raw_to_millivolts(adc_ref_internal(data->adc), data->acc.gain, data->as.resolution, &mv);

    /* 分圧前の電池電圧に戻す */
    uint32_t bat_mv = (uint64_t)mv * cfg->full_ohm / cfg->output_ohm;

    data->millivolts = bat_mv;
    data->state_of_charge = alkaline_mv_to_pct(bat_mv / cfg->cells);
    LOG_DBG("ADC raw %d ~ %d mV => battery %u mV => %d%%", data->adc_raw, mv, bat_mv,
            data->state_of_charge);

    return 0;
}

static int alk_channel_get(const struct device *dev, enum sensor_channel chan,
                           struct sensor_value *val) {
    struct alk_data *data = dev->data;

    switch (chan) {
    case SENSOR_CHAN_GAUGE_VOLTAGE:
        val->val1 = data->millivolts / 1000;
        val->val2 = (data->millivolts % 1000) * 1000U;
        return 0;
    case SENSOR_CHAN_GAUGE_STATE_OF_CHARGE:
        val->val1 = data->state_of_charge;
        val->val2 = 0;
        return 0;
    default:
        return -ENOTSUP;
    }
}

static const struct sensor_driver_api alk_api = {
    .sample_fetch = alk_sample_fetch,
    .channel_get = alk_channel_get,
};

static int alk_init(const struct device *dev) {
    struct alk_data *data = dev->data;
    const struct alk_config *cfg = dev->config;

    if (!device_is_ready(data->adc)) {
        LOG_ERR("ADC device not ready");
        return -ENODEV;
    }

    data->as = (struct adc_sequence){
        .channels = BIT(0),
        .buffer = &data->adc_raw,
        .buffer_size = sizeof(data->adc_raw),
        .resolution = 12,
        .oversampling = 4,
        .calibrate = true,
    };

    /* 内部基準 0.6V × ゲイン 1/6 → フルスケール 3.6V (VDD 以下の入力なら飽和しない) */
    data->acc = (struct adc_channel_cfg){
        .gain = ADC_GAIN_1_6,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
        .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0 + cfg->channel,
    };

    int rc = adc_channel_setup(data->adc, &data->acc);
    LOG_DBG("AIN%u setup returned %d", cfg->channel, rc);
    return rc;
}

#define ALK_INIT(n)                                                                                \
    static struct alk_data alk_data_##n = {                                                        \
        .adc = DEVICE_DT_GET(DT_INST_IO_CHANNELS_CTLR(n)),                                         \
    };                                                                                             \
    static const struct alk_config alk_config_##n = {                                              \
        .channel = DT_INST_IO_CHANNELS_INPUT(n),                                                   \
        .cells = DT_INST_PROP(n, cells),                                                           \
        .output_ohm = DT_INST_PROP(n, output_ohms),                                                \
        .full_ohm = DT_INST_PROP_OR(n, full_ohms, DT_INST_PROP(n, output_ohms)),                   \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, alk_init, NULL, &alk_data_##n, &alk_config_##n, POST_KERNEL,          \
                          CONFIG_SENSOR_INIT_PRIORITY, &alk_api);

DT_INST_FOREACH_STATUS_OKAY(ALK_INIT)
