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

このファームウェアは `pico-copter-uf2-builder` を土台に、`i-26` の制御・PWM・SBUS等の変更を移植しています。ただし**4モータのミキシング式はベースのPico Copterから変更していません**（Yaw符号も出力倍率も元のまま）。

### PWM GPIO（i-26 実装に準拠）

| Output | GPIO | Pico physical pin |
|---|---:|---:|
| RL | 2 | 4 |
| RR | 3 | 5 |
| FL | **14** | **19** |
| FR | **15** | **20** |
| Servo | 6 | 9 |

PWMピンを変更する場合は `pwm.hpp` の `PWM_PIN_*` を変更します。メインループは `PWM_MAINLOOP_PIN` のPWMラップ割り込み（400Hz）に同期します。

### 実機に書き込む前の注意

- i-26 のREADMEの FL=GPIO4 / FR=GPIO5 という表記と実装は異なり、実装では14/15を使用します。
- i-26 に合わせてPIDパラメータとSBUS範囲が変更されているため、送信機のキャリブレーションやゲインの再確認が必要です。
- モータ出力計算（`control.cpp` 内の4本の `*_duty`）はPico Copterオリジナルを保持しています。
- プロペラを外して各軸の応答・回転方向・フェイルセーフを確認してください。
