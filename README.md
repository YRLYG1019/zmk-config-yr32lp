# yr32lp ZMK ファームウェア

QMK版 (`../qmk用`) を ZMK に移植したものです。

- マイコン: Waveshare RP2040-Zero(USB 有線)
- トラックボール: Pimoroni PIM447(I2C1 SDA=GP2 / SCL=GP3)
- LED: WS2812 × 30(GP0)

## できあがった uf2

`firmware/yr32lp_rp2040_zero.uf2`

### 書き込み方法
1. RP2040-Zero の **BOOT** ボタンを押しながら USB ケーブルを挿す
   (ZMK 書き込み後は、C か D を長押し + 左親指の内側キーでも入れます)
2. PC に `RPI-RP2` ドライブが出るので、uf2 をドラッグ&ドロップ
3. 自動で再起動したら完了

書込モードに入ったあと書き込まずに戻したいときは、USB ケーブルを抜き差ししてください。

## キーマップの書き換え(ZMK Studio / USB)

ZMK Studio を USB で使えるようにしてあります(ロック解除キーは不要)。

- YRキーマップエディター: 「🔌 USB接続」→ 一覧から yr32lp の COM ポートを選ぶ → 編集 →「📤 書き込み」
- 公式の ZMK Studio (https://zmk.studio) でも接続できます

## フォルダ構成

| パス | 内容 |
|---|---|
| `config/boards/shields/yr32lp/yr32lp.overlay` | ピン配置・マトリクス・トラックボール・LED |
| `config/boards/shields/yr32lp/yr32lp.keymap` | キーマップ(5 レイヤー + コンボ 12 個) |
| `yr32lp_keymap.pdf` | キーマップの図(A4横) |
| `config/boards/shields/yr32lp/yr32lp.conf` | 機能設定(ポインティング・RGB) |
| `config/west.yml` | ZMK のバージョン(main のコミットに固定) |
| `build.yaml` | ビルド対象 `rp2040_zero//zmk` + `yr32lp` |
| `drivers/input/input_pim447.c` ほか | PIM447 ドライバ(ZMK 本体に無いので自作。QMK と同じ計算方法) |
| `.github/workflows/build.yml` | GitHub Actions でビルドする場合の設定 |

## トラックボールの調整(yr32lp.overlay)

```dts
trackball: trackball@a {
    compatible = "pimoroni,pim447";
    reg = <0x0a>;
    scale = <12>;         // 速度(上限なし / QMK版の上限=8 / QMK版と同じ速さ=2)
    rotation = <270>;     // QMK の POINTING_DEVICE_ROTATION_270
    poll-interval-ms = <10>;
    // invert-x; invert-y;  // 向きを反転したいとき
    // linear;              // 2乗加速をやめたいとき
    // led-rgbw = [00 00 00 10];  // ボールの LED(R G B W)
};
```

### 横方向の速さ(yr32lp.keymap)

カーソルの横方向だけ 1.5 倍にしています。`&trackball_listener` の `<&zip_x_scaler 3 2>`(= 3/2)で調整します(2 倍なら `2 1`)。

### スクロール(yr32lp.keymap)

Fn レイヤー中(C か D を長押し)は、トラックボールで画面スクロールします(ボールを上 = 上にスクロール)。
速さは `&trackball_listener` の `<&zip_scroll_scaler 1 48>` で調整します(速すぎるときは 48 を大きく、遅すぎるときは小さく)。
※ overlay の scale を変えるとスクロールの速さも同じ割合で変わります。

## QMK版との違い

- `MT(MO(1),KC_V)` などは QMK では Ctrl/Shift のモッドタップになってしまう書き方のため、
  意図どおり **レイヤータップ**(`&lt 1 V` など)にしています。
- RGB は ZMK に RGB Matrix が無いため **アンダーグロー**(全体の色・効果)です。
- Fn レイヤー(C か D を長押し)に次のキーを追加しています。
  - 左親指の内側キー: ブートローダー(書き込みモード)
  - 右下段 G / D / M / J の位置: LED ON/OFF・効果切替・明るさ+・明るさ−
- PIM447 の通信エラーが続いても止まらず、自動で再接続します(QMK は 10 回で停止)。

## 自分でビルドする方法

### GitHub Actions
このフォルダを GitHub リポジトリにして push すると、Actions で uf2 が作られます。

### Docker(ローカル)
```bash
docker run --rm -it -v zmk-ws:/ws -v "<このフォルダ>:/src" zmkfirmware/zmk-build-arm:stable bash
# コンテナ内
mkdir -p /ws/w && cp -r /src/config /ws/w/ && cd /ws/w
west init -l config && west update --narrow -o=--depth=1 && west zephyr-export
west build -p -s zmk/app -d build -b rp2040_zero//zmk -- -DSHIELD=yr32lp -DZMK_CONFIG=/ws/w/config -DZMK_EXTRA_MODULES=/src
```
