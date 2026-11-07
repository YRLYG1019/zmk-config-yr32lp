/*
 * Pimoroni PIM447 Trackball (I2C) input driver for ZMK / Zephyr
 *
 * QMK の drivers/sensors/pimoroni_trackball.c と同じ計算方法で移動量を作る:
 *   - 移動量 = scale * offset * offset  (2乗加速。linear 指定時は scale * offset)
 *   - ボールを押している間は移動を止め、離した直後も少しの間止める(誤移動防止)
 *   - rotation は QMK の POINTING_DEVICE_ROTATION_xx と同じ向きの定義
 * QMK と違い、I2C エラーが続いても読み取りを止めず、再初期化を繰り返す。
 * AZ1UBALL(PIM447 互換)でも使える。az1uball を指定すると起動時に通常モードのコマンドを、
 * az1uball-accel を指定するとボール側の加速モードのコマンドを送る。
 * accel-window-polls を指定すると、上下・左右を差し引いたあとの速さで加速する
 * (反対側センサーのノイズは打ち消されてから加速されるので、逆向きに大きく動かない)。
 * reject-reverse-ms を指定すると、一方向に動いている最中の逆向きのカウント(ノイズ)を捨てる。
 * smoothing を 2 以上にすると、1 回分の移動を数回のポーリングに分けて出す(なめらか処理)。
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT pimoroni_pim447

#include <stdlib.h>
#include <string.h>
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
#define AZ1UBALL_CMD_NORMAL 0x90
#define AZ1UBALL_CMD_ACCEL 0x91

/* 加速の速さを測る区間の最大ポーリング回数 */
#define PIM447_ACCEL_WINDOW_MAX 16

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
    bool az1uball;
    bool az1uball_accel;
    uint8_t debounce_polls;
    uint8_t smoothing;
    uint8_t accel_window;
    uint16_t accel_step;
    uint16_t accel_max;
    uint16_t reject_reverse_ms;
    uint8_t reverse_counts;
    uint8_t rgbw[4];
};

/* 逆向きノイズ除去の状態(1 軸分): 最後に動いた向き・時刻と、保留中の逆向きカウント */
struct pim447_dir {
    int8_t dir;
    int64_t ms;
    int32_t held;
};

struct pim447_data {
    const struct device *dev;
    struct k_work_delayable work;
    bool ready;
    bool pressed;
    uint8_t debounce;
    uint8_t errors;
    /* なめらか処理でまだ出していない移動量 */
    int32_t pending_x;
    int32_t pending_y;
    /* 加速用: 直近のカウント(差し引き後)とその合計 */
    int16_t hist_x[PIM447_ACCEL_WINDOW_MAX];
    int16_t hist_y[PIM447_ACCEL_WINDOW_MAX];
    uint8_t hist_pos;
    int32_t sum_x;
    int32_t sum_y;
    /* 逆向きノイズ除去用 */
    struct pim447_dir dir_x;
    struct pim447_dir dir_y;
    /* 調整用の記録の通し番号(抜けがあれば記録が欠けている) */
    uint32_t log_seq;
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

    /* AZ1UBALL のカウント方式。加速モード(速く回したときだけ 1 回の検知で 2〜3 カウント)は
     * ボール側が電源を切るまで覚えているので、使わないときも通常モードを送る。
     * ボール側は先頭 1 バイトだけを見る。本物の PIM447 には送らない。再初期化のたびに送り直す */
    if (cfg->az1uball || cfg->az1uball_accel) {
        uint8_t cmd[2] = {cfg->az1uball_accel ? AZ1UBALL_CMD_ACCEL : AZ1UBALL_CMD_NORMAL, 0x00};
        ret = i2c_write_dt(&cfg->i2c, cmd, sizeof(cmd));
        if (ret < 0) {
            return ret;
        }
    }

    /* 起動前にたまったカウントを読み捨てる */
    uint8_t dummy[5];
    return i2c_burst_read_dt(&cfg->i2c, PIM447_REG_LEFT, dummy, sizeof(dummy));
}

/* カウント(差し引き後)→ 移動量。QMK pimoroni_trackball_get_offsets() と同じ符号・計算
 * (2乗加速。linear 指定時は比例)に、加速の倍率(percent)を掛ける */
static int32_t pim447_move(int32_t count, uint16_t scale, bool linear, uint16_t percent) {
    int32_t mag = abs(count);
    mag = linear ? (scale * mag) : (scale * mag * mag);
    mag = mag * percent / 100;
    return count < 0 ? -mag : mag;
}

/* 加速の倍率: 直近 accel-window-polls 回の移動(上下・左右を差し引いた後)のカウント数 s が
 * 1 以下なら 100%、それより多ければ 1 カウントごとに accel-step-percent ずつ上げる
 * (上限 accel-max-percent)。差し引き後で測るので、反対側センサーのノイズは速さに入らない */
static uint16_t pim447_accel_percent(const struct pim447_config *cfg, struct pim447_data *data,
                                     int32_t cx, int32_t cy) {
    if (cfg->accel_window == 0) {
        return 100;
    }
    uint8_t n = MIN(cfg->accel_window, PIM447_ACCEL_WINDOW_MAX);
    uint8_t i = data->hist_pos;
    data->sum_x += cx - data->hist_x[i];
    data->sum_y += cy - data->hist_y[i];
    data->hist_x[i] = (int16_t)cx;
    data->hist_y[i] = (int16_t)cy;
    data->hist_pos = (i + 1) % n;

    int32_t s = abs(data->sum_x) + abs(data->sum_y);
    if (s <= 1) {
        return 100;
    }
    int32_t pct = 100 + (int32_t)cfg->accel_step * (s - 1);
    return (uint16_t)MIN(pct, (int32_t)cfg->accel_max);
}

/* 逆向きノイズの除去(1 軸分)。
 * 最後に動いた向きと反対のカウントが、最後の移動から hold_ms 以内に来たら保留する。
 *   - 保留が min_counts に達したら本当に向きを変えたとみなし、保留分もまとめて通す
 *   - その前に元の向きのカウントが来たら、保留分はノイズとして捨てる
 *   - hold_ms 以上止まってからの逆向きは、すぐ通す
 * ゆっくり転がしている間は本当のカウントで hold が延び続けるので、
 * 反対側センサーのノイズ(1〜2 カウント)は通らない */
static int32_t pim447_reject_reverse(int32_t c, struct pim447_dir *d, int64_t now,
                                     uint16_t hold_ms, uint8_t min_counts) {
    if (c == 0) {
        if (d->held != 0 && now - d->ms >= hold_ms) {
            d->held = 0; /* 元の向きが止まったあとの単発ノイズは捨てる */
        }
        return 0;
    }
    int8_t dir = c > 0 ? 1 : -1;
    if (d->dir == 0 || dir == d->dir) {
        d->dir = dir;
        d->ms = now;
        d->held = 0;
        return c;
    }
    if (now - d->ms >= hold_ms) {
        d->dir = dir;
        d->ms = now;
        d->held = 0;
        return c;
    }
    d->held += c;
    if (abs(d->held) >= min_counts) {
        int32_t out = d->held;
        d->dir = dir;
        d->ms = now;
        d->held = 0;
        return out;
    }
    return 0;
}

static void pim447_accel_reset(struct pim447_data *data) {
    memset(data->hist_x, 0, sizeof(data->hist_x));
    memset(data->hist_y, 0, sizeof(data->hist_y));
    data->hist_pos = 0;
    data->sum_x = data->sum_y = 0;
}

/* なめらか処理: たまっている移動量の 1/n (切り上げ) を取り出す。
 * 1 カウント分の移動を一度に出さず、数回のポーリングに分けて出す */
static int32_t pim447_take(int32_t *pending, uint8_t n) {
    int32_t p = *pending;
    int32_t out;
    if (n <= 1) {
        out = p;
    } else if (p > 0) {
        out = (p + n - 1) / n;
    } else {
        out = -((-p + n - 1) / n);
    }
    *pending = p - out;
    return out;
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
#if IS_ENABLED(CONFIG_INPUT_PIM447_LOG_RAW)
        LOG_INF("SW,%u,%u,%d", data->log_seq++, k_uptime_get_32(), pressed ? 1 : 0);
#endif
    }

    if (pressed) {
        data->debounce = cfg->debounce_polls;
        data->pending_x = data->pending_y = 0;
        pim447_accel_reset(data);
    } else if (data->debounce > 0) {
        data->debounce--;
        data->pending_x = data->pending_y = 0;
        pim447_accel_reset(data);
    } else {
        /* QMK: x = offsets(right, left), y = offsets(down, up)。ここではまだカウントのまま */
        int32_t qx = (int32_t)buf[0] - (int32_t)buf[1];
        int32_t qy = (int32_t)buf[2] - (int32_t)buf[3];
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
        int32_t raw_x = x, raw_y = y;

        if (cfg->reject_reverse_ms > 0) {
            int64_t now = k_uptime_get();
            x = pim447_reject_reverse(x, &data->dir_x, now, cfg->reject_reverse_ms,
                                      cfg->reverse_counts);
            y = pim447_reject_reverse(y, &data->dir_y, now, cfg->reject_reverse_ms,
                                      cfg->reverse_counts);
        }
        int32_t filt_x = x, filt_y = y;
        uint16_t pct = pim447_accel_percent(cfg, data, x, y);
        x = pim447_move(x, cfg->scale, cfg->linear, pct);
        y = pim447_move(y, cfg->scale, cfg->linear, pct);

        data->pending_x += x;
        data->pending_y += y;
        x = pim447_take(&data->pending_x, cfg->smoothing);
        y = pim447_take(&data->pending_y, cfg->smoothing);

#if IS_ENABLED(CONFIG_INPUT_PIM447_LOG_RAW)
        /* 調整用の記録: 生カウント(左,右,上,下)→ 回転後 → ノイズ除去後 → 倍率 → 出力 */
        if (buf[0] || buf[1] || buf[2] || buf[3] || x || y) {
            LOG_INF("RAW,%u,%u,%u,%u,%u,%u,%d,%d,%d,%d,%u,%d,%d", data->log_seq++,
                    k_uptime_get_32(), buf[0], buf[1], buf[2], buf[3], raw_x, raw_y, filt_x,
                    filt_y, pct, x, y);
        }
#else
        ARG_UNUSED(raw_x);
        ARG_UNUSED(raw_y);
        ARG_UNUSED(filt_x);
        ARG_UNUSED(filt_y);
#endif

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
        .az1uball = DT_INST_PROP(n, az1uball),                                                     \
        .az1uball_accel = DT_INST_PROP(n, az1uball_accel),                                         \
        .debounce_polls = DT_INST_PROP(n, debounce_polls),                                         \
        .smoothing = DT_INST_PROP(n, smoothing),                                                   \
        .accel_window = DT_INST_PROP(n, accel_window_polls),                                       \
        .accel_step = DT_INST_PROP(n, accel_step_percent),                                         \
        .accel_max = DT_INST_PROP(n, accel_max_percent),                                           \
        .reject_reverse_ms = DT_INST_PROP(n, reject_reverse_ms),                                   \
        .reverse_counts = DT_INST_PROP(n, reverse_counts),                                         \
        .rgbw = DT_INST_PROP(n, led_rgbw),                                                         \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, pim447_init, NULL, &pim447_data_##n, &pim447_config_##n,              \
                          POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(PIM447_DEFINE)
