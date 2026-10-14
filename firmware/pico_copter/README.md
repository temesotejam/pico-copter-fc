# Pico Copter firmware

このフォルダが、Raspberry Pi Picoへ書き込むファームウェアの独立したプロジェクトです。共有するときは`firmware/pico_copter`フォルダをそのまま取り出せます。

`.cpp`、`.hpp`、`.c`、`.h`は`github.dev`または通常のローカル開発環境で編集できます。変更履歴はGitで管理され、GitHub Actionsが変更後のファームウェアをコンパイルします。

## 必要なもの

- Raspberry Pi Pico SDK 2.3.0
- ARM GCC
- CMake 3.13以上
- Eigen 3.4.0

Eigenの場所を指定しない場合、CMakeが公式Eigenの固定コミットを自動取得します。

## ビルド例

```bash
cmake -S . -B build \
  -DPICO_SDK_PATH=/path/to/pico-sdk \
  -DPICO_BOARD=pico \
  -DCMAKE_BUILD_TYPE=Release

cmake --build build --parallel
```

生成物は`build/pico_copter.uf2`です。

既にEigenを持っている場合は、次を追加できます。

```bash
-DEIGEN3_INCLUDE_DIR=/path/to/eigen
```

## i-26 からの移植について

このファームウェアは `pico-copter-uf2-builder` を土台に、`i-26` の制御・PWM・SBUS等の変更を移植しています。**モータミキシングでは `R_com`（Yaw補正）の符号だけ元のPico Copterを維持し、それ以外の係数・出力倍率 `0.9` はi-26に合わせています。**

### PWM GPIO（現行基板：i-26 実装に準拠）

| Output | GPIO | Pico physical pin |
|---|---:|---:|
| RL | 2 | 4 |
| RR | 3 | 5 |
| FL | **14** | **19** |
| FR | **15** | **20** |
| Servo | 6 | 9 |

PWMピンを変更する場合は `pwm.hpp` の `PWM_PIN_*` を変更します。メインループは `PWM_MAINLOOP_PIN` のPWMラップ割り込み（400Hz）に同期します。

### 実機に書き込む前の注意

- 現行基板では FL=GPIO14 / FR=GPIO15 を使用します。i-26 のREADMEの GPIO4 / GPIO5 は旧配線を示しており、現在の `pwm.hpp` には適合しません。
- i-26 に合わせてPIDパラメータとSBUS範囲が変更されているため、送信機のキャリブレーションやゲインの再確認が必要です。
- モータミキシングは i-26 の係数・倍率（4モータとも `*0.9`）を使用し、`R_com` の符号だけ Pico Copter のオリジナルを維持しています。
- プロペラを外して各軸の応答・回転方向・フェイルセーフを確認してください。

## CH5操作による100 Hzリングバッファ記録

飛行中のログは**CH5をONにした間だけ**記録します。アームするだけでは記録せず、CH5をOFFにするとその時点の直近約10秒分をRAMに保持します。

| 操作・状況 | 記録の動作 |
|---|---|
| 電源投入・アーム | 記録しない。アーム後にCH5のOFF状態を確認してからONを受け付ける |
| 飛行中にCH5をON | 新しい記録として開始（以前のデータを置き換える） |
| CH5をONのまま | 100 Hzで38項目を1行として連続記録し、満杯なら最古の行を上書き |
| CH5をOFF | 次の100 Hz処理で記録を凍結し、データを消去しない（停止遅延は通常10 ms以内） |
| ディスアーム・通信断 | 記録していた場合は凍結。自動再記録しない |
| 停止後に再びCH5をON | 前の記録を消去して新しい記録を開始 |
| ディスアーム中の既存ログ出力操作 | RAMの実データだけ古い順にUSBシリアル出力。出力後も保持 |
| 再アーム（CH5はOFF） | 以前の記録を保持。新たにCH5をONにするまで上書きしない |

- **記録周期:** 公称100 Hz。リアルタイムの制御・推定を優先し、リング記録以外の処理周期は変更しません。
- **記録内容:** 既存の38項目をそのまま維持します。
- **容量:** 1,000行×38個のfloat＝**152,000バイトの固定RAM**。公称10秒分。
- **CH5の判定:** 現行のCH5中間値より上でON、下側でOFF。
- **出力時刻:** 記録開始からの公称時間で、リングが一周しても出力行の時刻は連続します。実測タイムスタンプではありません。
- **負荷対策:** CH5 OFF時は38項目の行生成・RAM書込みを行わず、CH5 ON時だけ実行します。RAM上で完結し、記録中のシリアル出力やフラッシュ書込みは行いません。

**注意:** RAMのため電源OFFでデータは消失します。新しいCH5 ONでも前の記録を上書きするため、必要なデータは再記録の前にPCへ取り出してください。実機での動作確認は必ずプロペラを外してください。

## DShot300 motor output (RP2040 PIO)

Motor outputs have been changed from analog PWM to **one-way DShot300**.
The GPIO assignments, PID/EKF computations, radio receiver, CH5 logger, and
400 Hz / 100 Hz control rates remain unchanged.

| Signal | GPIO | Pico physical pin | Interface |
|---|---:|---:|---|
| Rear Left (RL) | 2 | 4 | PIO0 DShot300 |
| Rear Right (RR) | 3 | 5 | PIO0 DShot300 |
| Front Left (FL) | 14 | 19 | PIO0 DShot300 |
| Front Right (FR) | 15 | 20 | PIO0 DShot300 |
| Servo | 6 | 9 | hardware PWM, unchanged |

- `pwm.cpp` keeps the PWM **slice 7** wrap IRQ solely as the **2.5 ms/400 Hz**
  scheduler. GPIO14 is configured for PIO rather than PWM but its internal
  PWM slice continues to run. Servo PWM remains on slice 3.
- PIO0 uses four state machines (one per motor), at 2.4 MHz PIO execution clock,
  eight PIO cycles per DShot bit. Each full 16-bit frame takes about 53.3 µs.
  No software bit-banging, DMA, or new CPU interrupt is required.
- The existing `set_duty_fr/fl/rr/rl(float)` entry points are preserved.
  `motor_stop()` now sends DShot throttle **0** on all four pins; stop frames
  continue at 400 Hz while disarmed / in radio failsafe.
- To approximate the previous ESC PWM pulse span of **1100–1900 µs**, a
  positive `duty` of 0.0–1.0 maps to a DShot normalized 10%–90% command,
  namely `48 + round(1999 * (0.10 + 0.80 * duty))`. Duty 0,
  negative values and non-finite values send DShot stop code 0.
  The 1–47 special-command range is never generated.
- DShot's bit-level timing and CRC are verified by a host-side test. This
  firmware uses **standard unidirectional** (non-inverted) CRC with the
  telemetry-request bit always zero. Bidirectional DShot / RPM telemetry
  are not yet implemented.
- The four GPIOs have **not** been reassigned; I-26 board wiring is preserved.
  However, this is a **protocol change**: ESCs must actually be configured
  for DShot300, and their response to DShot throttle values may differ from
  the previous analog PWM calibration. No motor speed equivalence is assumed.
- This implementation relies on the 400 Hz controller to refresh all four
  outputs, including DShot stop. It is not an independent hardware watchdog:
  a stalled controller cannot be guaranteed to send further stop frames.

**Before operation:** remove propellers, check each pin with a logic analyzer
(DShot300 bit 3.33 µs, '0' high 1.25 µs, '1' high 2.50 µs),
verify all four motors respond to the correct output and STOP=0, and then
recheck low/high throttle behavior. Never use this new UF2 for a propeller-on
flight before these checks. For a quick rollback, restore the prior commit
`b0c9826b3bca7c423c13f6b8884542189809b513` (legacy PWM firmware).

## SBUS communication-loss protection

- `sbus_parser.hpp`: 完全な25バイトフレームのみ解析し、フレームロスト/フェイルセーフフラグを確認。
- `radio.cpp`: 正常なフレームの最終受信時刻を記録し、100 msの通信監視を実施。
- `control.cpp`: 異常検出時にアームを解除、モータに最小ESC指令を送信。通信復帰後はスロットル低・アーム操作解除を200 ms維持し、通常の再アーム操作を必須にする。
- `tests/test_sbus.cpp`: フレーム解析、通信タイムアウト、復帰条件の単体テスト。

**注意:** PWM最小指令は電源遮断ではありません。受信機がフェイルセーフフラグなしで古いチャンネル値を送信し続ける場合は通信断を検出できません。プロペラを外して実機確認してください。
