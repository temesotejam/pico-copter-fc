# Pico Copter FC

Raspberry Pi Pico用のPico Copterファームウェアを、GitHub Actionsでコンパイルし、最新のUF2をGitHub Pagesからダウンロードするためのリポジトリです。


## 派生元と移植範囲

- ベース: [pico-copter-uf2-builder](https://github.com/temesotejam/pico-copter-uf2-builder)
- 変更の移植元: [i-26](https://github.com/temesotejam/i-26)
- **唯一の制御上の例外: モータミキシングの `R_com`（Yaw補正）符号だけは元の Pico Copter に合わせています。** PID・各軸の補正量・ミキシング係数・最終倍率 `0.9` は i-26 に合わせています。
- 移植する変更: PIDパラメータ、EKF呼び出し、PWM GPIO設定・IRQ処理・出力競合検査、SBUS入力範囲など。
- `i26-pico-firmware` は今回の移植元ではありません。ToF、RGB LEDなどの追加機能は含みません。

### 現行基板の配線（i-26 準拠）

| 出力 | 元のPico Copter | このリポジトリ（i-26準拠） |
|---|---:|---:|
| Rear Left | GPIO 2 | GPIO 2 |
| Rear Right | GPIO 3 | GPIO 3 |
| Front Left | GPIO 4 | **GPIO 14** |
| Front Right | GPIO 5 | **GPIO 15** |
| Servo | GPIO 6 | GPIO 6 |

**現行基板は i-26 の実装ピン配置です。** FL/FR は GPIO 14/15 を使用し、旧基板の GPIO 4/5 には戻しません。i-26 側の README に書かれた FL/FR=GPIO 4/5 は実装と一致していません。

ミキシングの `P_com` / `Q_com` の符号・構造は i-26 と同じで、`R_com` の符号のみ Pico Copter のものです。4モータとも計算式の最終倍率は **`0.9`** で、旧版の `*0.25` と `*0.0901` は使用しません。

プロペラを外して初期動作を確認してください。コードの移植は飛行安全性やビルドの成功を保証するものではありません。

## RC link failsafe

SBUS受信安全処理（`firmware/pico_copter/sbus_parser.hpp`）を追加しています。

- 有効なSBUSフレームが100 ms届かない場合、最小ESC指令へ移行してアーム解除
- SBUSのフェイルセーフフラグを検出した場合、直ちに安全状態に移行
- 単発のフレームロストは有効受信に数えず、100 msタイムアウトで判定
- 通信復帰後はスロットル低・アーム操作解除を200 ms継続し、改めて通常のアーム操作を必要とする
- 起動直後も受信確認まではアームできない
- CIでSBUSフレーム解析と復帰判定のテストを実施

**制限:** 停止指令はESCへの最小PWMであり、モータの電源遮断ではありません。受信機が古いスティック値を正常なSBUSフレームとして送信し続け、フェイルセーフを通知しない設定の場合、通信断を判別できません。受信機の設定を確認し、プロペラを外して検証してください。

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

## ファームウェアとビルダーの分離

マイコンへ書き込むプログラムは、次の1フォルダへまとめています。

```text
firmware/pico_copter/
```

他人と共有するときは、このフォルダをそのまま取り出せます。フォルダ内には、プログラム本体、独立ビルド用`CMakeLists.txt`、ビルド手順が入っています。

```text
firmware/pico_copter/
├─ pico_copter.cpp / .hpp
├─ control.cpp / .hpp
├─ ekf.cpp / .hpp
├─ sensor.cpp / .hpp
├─ pwm.cpp / .hpp
├─ radio.cpp / .hpp
├─ lsm9ds1_reg.c / .h
├─ CMakeLists.txt
└─ README.md
```

GitHub Actions、キャッシュ、Web配布画面などは、ファームウェアフォルダの外側に分離しています。

## github.devでファームウェアを編集する

通常のGitHubで編集したいファイルを開き、キーボードの`.`キーを押すと、同じリポジトリを`github.dev`で開けます。

主な編集対象は次のフォルダです。

```text
firmware/pico_copter/
```

編集後は、左側のソース管理から変更内容を入力して`Commit & Push`を実行します。`main`へ反映されるとGitHub Actionsが自動的にコンパイルし、成功したUF2をPagesへ公開します。Pull Requestではコンパイル確認だけを行います。

ファームウェアの変更履歴はGitで管理されるため、以前の状態はGitHubのコミット履歴から確認・復元できます。

## リポジトリ構成

```text
pico-copter-fc/
├─ firmware/
│  └─ pico_copter/              共有・持ち出し可能なファームウェア
├─ web/                         UF2配布ページ
├─ .github/workflows/
│  └─ build-and-deploy.yml      クラウドビルドとPages公開
├─ .gitignore
└─ README.md
```

元のPico Copterリポジトリとのサブモジュール連携や自動同期はありません。このリポジトリ内のファイルだけで管理します。

## GitHubでUF2を生成する

1. `Settings`を開く
2. `Pages` → `Build and deployment`を開く
3. `Source`を`GitHub Actions`に設定する
4. `Actions`を開く
5. `Build Pico Copter UF2 and deploy Pages`を選ぶ
6. `Run workflow`を押す
7. 成功後、`Settings` → `Pages`に表示される公開先を開く

`main`のファームウェアやWorkflowを変更した場合も自動実行されます。Pull Requestではコンパイル確認だけを行い、Pagesへの公開は行いません。

## ファームウェアフォルダ単体でビルドする

`firmware/pico_copter`へ移動し、Pico SDKを指定してビルドします。

```bash
cd firmware/pico_copter

cmake -S . -B build \
  -DPICO_SDK_PATH=/path/to/pico-sdk \
  -DPICO_BOARD=pico \
  -DCMAKE_BUILD_TYPE=Release

cmake --build build --parallel
```

生成物は次です。

```text
firmware/pico_copter/build/pico_copter.uf2
```

## 依存関係

| 項目 | 固定値 |
|---|---|
| 対象ボード | `pico` |
| Pico SDK | `2.3.0` |
| Eigen | `3.4.0` |
| Eigenコミット | `3147391d946bb4b6c68edd901f2add6ac1f31f8c` |
| C++ | C++17 |

ファームウェアフォルダのCMakeは、`EIGEN3_INCLUDE_DIR`が指定されていない場合、公式Eigenの固定コミットを自動取得します。既にEigenを持っている場合は、次のように指定できます。

```bash
-DEIGEN3_INCLUDE_DIR=/path/to/eigen
```

GitHub ActionsではEigenを別に取得してキャッシュし、そのパスをファームウェア側のCMakeへ渡します。

## 高速化

GitHub Actionsでは次をキャッシュします。

```text
Pico SDK
Eigen 3.4.0
ccacheによるC/C++のコンパイル結果
```

初回、キャッシュ削除後、依存バージョン変更後は通常どおり時間がかかります。

## ビルド結果

成功時は次を生成します。

```text
pico_copter.uf2
pico_copter.uf2.sha256
size.txt
manifest.json
configure.log
build.log
```

通常Artifactと診断ログの保持期間は1日です。GitHub Pagesには最後に成功したUF2が公開されます。

## コンパイルエラーについて

編集した`.cpp/.hpp/.c/.h`にコンパイルエラーやリンクエラーがある場合、Workflowのログへそのまま表示されます。Workflowがプログラム本体を自動修正することはありません。

## UF2の書き込み

1. Pagesから`pico_copter-pico.uf2`をダウンロードする
2. Picoの`BOOTSEL`ボタンを押したままUSB接続する
3. 表示された`RPI-RP2`ドライブへUF2をコピーする
4. コピー後、Picoが自動的に再起動する
