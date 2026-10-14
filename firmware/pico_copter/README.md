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

## SBUS communication-loss protection

- `sbus_parser.hpp`: 完全な25バイトフレームのみ解析し、フレームロスト/フェイルセーフフラグを確認。
- `radio.cpp`: 正常なフレームの最終受信時刻を記録し、100 msの通信監視を実施。
- `control.cpp`: 異常検出時にアームを解除、モータに最小ESC指令を送信。通信復帰後はスロットル低・アーム操作解除を200 ms維持し、通常の再アーム操作を必須にする。
- `tests/test_sbus.cpp`: フレーム解析、通信タイムアウト、復帰条件の単体テスト。

**注意:** PWM最小指令は電源遮断ではありません。受信機がフェイルセーフフラグなしで古いチャンネル値を送信し続ける場合は通信断を検出できません。プロペラを外して実機確認してください。
