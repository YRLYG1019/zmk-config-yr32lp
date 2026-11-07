/*
 * Pimoroni PIM447 Trackball (I2C) input driver for ZMK / Zephyr
 *
 * QMK の drivers/sensors/pimoroni_trackball.c と同じ計算方法で移動量を作る:
 *   - 移動量 = scale * offset * offset  (2乗加速。linear 指定時は scale * offset)
 *   - ボールを押している間は移動を止め、離した直後も少しの間止める(誤移動防止)
 *   - rotation は QMK の POINTING_DEVICE_ROTATION_xx と同じ向きの定義
 * QMK と違い、I2C エラーが続いても読み取りを止めず、再初期化を繰り返す。
 * AZ1UBALL(PIM447 互換)でも使える。az1uball-accel でボール側の加速モードを ON にできる。
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT pimoroni_pim447

#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pim447, CONFIG_INPUT_LOG_LEVEL);

#define PIM447_REG_LED_RED 0x00
#define PIM447_REG_LEFT 0x04
#define PIM447_REG_CHIP_ID_L 0xFA
#define PIM447_CHIP_ID 0xBA11
#define PIM447_SWITCH_STATE 0x80
/* AZ1UBALL: カウント方式の切替コマンド (0x90 = 通常 / 0x91 = 加速モード) */
#define AZ1UBALL_CMD_ACCEL 0x91

#define PIM447_ERROR_LIMIT 10
#define PIM447_RETRY_MS 500
#define PIM447_STARTUP_MS 300

struct pim447_config {
    struct i2c_dt_spec i2c;
    uint16_t poll_ms;
    uint16_t scale;
    uint16_t rotation;
    bool invert_x;
    bool invert_y;
    bool linear;
    bool az1uball_accel;
    uint8_t debounce_polls;
    uint8_t rgbw[4];
};

struct pim447_data {
    const struct device *dev;
    struct k_work_delayable work;
    bool ready;
    bool pressed;
    uint8_t debounce;
    uint8_t errors;
};

static int pim447_setup(const struct device *dev) {
    const struct pim447_config *cfg = dev->config;
    uint8_t id[2] = {0};
    int ret;

    ret = i2c_burst_read_dt(&cfg->i2c, PIM447_REG_CHIP_ID_L, id, sizeof(id));
    if (ret < 0) {
        return ret;
    }
    uint16_t chip_id = ((uint16_t)id[1] << 8) | id[0];
    if (chip_id != PIM447_CHIP_ID) {
        /* 互換品などで ID が違っても動作は続ける */
        LOG_WRN("unexpected chip id 0x%04x", chip_id);
    }

    uint8_t led[5] = {PIM447_REG_LED_RED, cfg->rgbw[0], cfg->rgbw[1], cfg->rgbw[2],
                      cfg->rgbw[3]};
    ret = i2c_write_dt(&cfg->i2c, led, sizeof(led));
    if (ret < 0) {
        return ret;
    }

    /* AZ1UBALL の加速モード(速く回したときだけ 1 回の検知で 2〜3 カウント)。
     * ボール側は先頭 1 バイトだけを見る。再初期化のたびに送り直す */
    if (cfg->az1uball_accel) {
        uint8_t cmd[2] = {AZ1UBALL_CMD_ACCEL, 0x00};
        ret = i2c_write_dt(&cfg->i2c, cmd, sizeof(cmd));
        if (ret < 0) {
            return ret;
        }
    }

    /* 起動前にたまったカウントを読み捨てる */
    uint8_t dummy[5];
    return i2c_burst_read_dt(&cfg->i2c, PIM447_REG_LEFT, dummy, sizeof(dummy));
}

/* QMK pimoroni_trackball_get_offsets() と同じ符号・計算 */
static int32_t pim447_offset(uint8_t negative_dir, uint8_t positive_dir, uint16_t scale,
                             bool linear) {
    int32_t offset = (int32_t)positive_dir - (int32_t)negative_dir;
    int32_t mag = abs(offset);
    mag = linear ? (scale * mag) : (scale * mag * mag);
    return offset < 0 ? -mag : mag;
}

static int16_t clamp16(int32_t v) {
    if (v > INT16_MAX) {
        return INT16_MAX;
    }
    if (v < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)v;
}

static void pim447_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct pim447_data *data = CONTAINER_OF(dwork, struct pim447_data, work);
    const struct device *dev = data->dev;
    const struct pim447_config *cfg = dev->config;

    if (!data->ready) {
        if (pim447_setup(dev) == 0) {
            data->ready = true;
            data->errors = 0;
            LOG_INF("PIM447 ready");
        } else {
            k_work_schedule(&data->work, K_MSEC(PIM447_RETRY_MS));
            return;
        }
    }

    /* left, right, up, down, switch */
    uint8_t buf[5];
    if (i2c_burst_read_dt(&cfg->i2c, PIM447_REG_LEFT, buf, sizeof(buf)) < 0) {
        if (++data->errors >= PIM447_ERROR_LIMIT) {
            LOG_WRN("PIM447 I2C error, re-initializing");
            data->ready = false;
            k_work_schedule(&data->work, K_MSEC(PIM447_RETRY_MS));
            return;
        }
        k_work_schedule(&data->work, K_MSEC(cfg->poll_ms));
        return;
    }
    data->errors = 0;

    bool pressed = (buf[4] & PIM447_SWITCH_STATE) != 0;
    if (pressed != data->pressed) {
        data->pressed = pressed;
        input_report_key(dev, INPUT_BTN_0, pressed ? 1 : 0, true, K_FOREVER);
    }

    if (pressed) {
        data->debounce = cfg->debounce_polls;
    } else if (data->debounce > 0) {
        data->debounce--;
    } else {
        /* QMK: x = offsets(right, left), y = offsets(down, up) */
        int32_t qx = pim447_offset(buf[1], buf[0], cfg->scale, cfg->linear);
        int32_t qy = pim447_offset(buf[3], buf[2], cfg->scale, cfg->linear);
        int32_t x, y;

        switch (cfg->rotation) {
        case 90:
            x = qy;
            y = -qx;
            break;
        case 180:
            x = -qx;
            y = -qy;
            break;
        case 270:
            x = -qy;
            y = qx;
            break;
        default:
            x = qx;
            y = qy;
            break;
        }
        if (cfg->invert_x) {
            x = -x;
        }
        if (cfg->invert_y) {
            y = -y;
        }

        if (x != 0 || y != 0) {
            input_report_rel(dev, INPUT_REL_X, clamp16(x), false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_Y, clamp16(y), true, K_FOREVER);
        }
    }

    k_work_schedule(&data->work, K_MSEC(cfg->poll_ms));
}

static int pim447_init(const struct device *dev) {
    const struct pim447_config *cfg = dev->config;
    struct pim447_data *data = dev->data;

    if (!i2c_is_ready_dt(&cfg->i2c)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }

    data->dev = dev;
    k_work_init_delayable(&data->work, pim447_work_cb);
    /* トラックボール側マイコンの起動を待ってから初期化する */
    k_work_schedule(&data->work, K_MSEC(PIM447_STARTUP_MS));
    return 0;
}

#define PIM447_DEFINE(n)                                                                           \
    static struct pim447_data pim447_data_##n;                                                     \
    static const struct pim447_config pim447_config_##n = {                                        \
        .i2c = I2C_DT_SPEC_INST_GET(n),                                                            \
        .poll_ms = DT_INST_PROP(n, poll_interval_ms),                                              \
        .scale = DT_INST_PROP(n, scale),                                                           \
        .rotation = DT_INST_PROP(n, rotation),                                                     \
        .invert_x = DT_INST_PROP(n, invert_x),                                                     \
        .invert_y = DT_INST_PROP(n, invert_y),                                                     \
        .linear = DT_INST_PROP(n, linear),                                                         \
        .az1uball_accel = DT_INST_PROP(n, az1uball_accel),                                         \
        .debounce_polls = DT_INST_PROP(n, debounce_polls),                                         \
        .rgbw = DT_INST_PROP(n, led_rgbw),                                                         \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, pim447_init, NULL, &pim447_data_##n, &pim447_config_##n,              \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(PIM447_DEFINE)
