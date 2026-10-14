# Pico Copter FC

Raspberry Pi PicoおよびPico 2用のPico Copterファームウェアを、GitHub Actionsでコンパイルし、最新のUF2をGitHub Pagesからダウンロードするためのリポジトリです。


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

## Raspberry Pi Pico 2（RP2350）対応

GitHub Actionsでは**初代Pico (RP2040) と Pico 2 (RP2350)**を同じソースコードから**別々にビルド**します。
Pico 2に初代Pico用UF2を書き込まず、必ず対象基板に合ったファイルを使ってください。

| 基板 | CMakeの指定 | 配布ファイル |
|---|---|---|
| Raspberry Pi Pico | `-DPICO_BOARD=pico` | `pico_copter.uf2` |
| **Raspberry Pi Pico 2** | **`-DPICO_BOARD=pico2`** | **`pico_copter_pico2.uf2`** |

GitHub Pagesで対象基板を選択できます。互換性のため初代Pico用の
`firmware/manifest.json` と `pico_copter.uf2` は維持し、
Pico 2用には `manifest-pico2.json` を追加しています。

- GPIO2/3/14/15の双方向DShot300、GPIO6サーボPWM、PWMスライス7の制御周期、
  400 Hz角速度制御、100 Hz姿勢制御、SBUSフェイルセーフ、CH5ログ、eRPM監視の仕様は共通です。
- 初代Picoのシステムクロックは標準125 MHz、Pico 2は標準150 MHz。
  **PWM分周比を起動時にクロックから計算**するため、どちらも
  制御周期とサーボPWM周期を400 Hz（2.5 ms）に維持します。
  PIOの双方向DShot300送受信クロックも従来どおり動的に計算します。
- PID/EKF、モータミキシング、eRPMの用途（USB診断だけ）、ログの38項目に変更はありません。
- Pico 2でのコンパイルとUF2生成はGitHub Actionsで検証できますが、
  実機での通信成功率・制御周期のジッタは未検証です。
  プロペラを外してモータ停止・各GPIOの信号・eRPM受信を確認してください。

## 双方向DShot300：4モータのeRPM監視

RP2040/RP2350のPIO0（4ステートマシン）で**双方向DShot300**を送受信します。配線は以前と同じです。

| モータ | GPIO | Pico物理ピン |
|---|---:|---:|
| FR（前右） | 15 | 20 |
| FL（前左） | 14 | 19 |
| RR（後右） | 3 | 5 |
| RL（後左） | 2 | 4 |

GPIO6のサーボはPWMのままです。GPIO14に対応するPWMスライス7は、
ピンへPWMを出さず**400 Hzの制御周期生成のみ**を継続します。

- モータ指令の計算（PID、ミキシング、停止条件）と100 HzのEKF・CH5ログは**変更しません**。
- 制御周期末に4本のDShot300を送信。**ESC応答を待たずに**処理を終了し、
  **次の400 Hz周期で前回の受信結果を確認**します。受信失敗してもモータ指令は送信します。
- 送信フォーマットは双方向用の反転波形・反転CRC（テレメトリ要求ビットは0）。
  応答の21ビットNRZI/GCRとCRCを検証し、eRPMを復号します。
- 4基のeRPMと受信成功率・受信失敗数を、飛行中にUSBシリアル接続がある場合のみ
  **約1秒に1回**表示します（例：`#DShot300 FR eRPM=50000 valid=1 rx=400/400 (100%) ...`）。
  250 msを超えて正常なeRPMが得られなければ `eRPM=-1 valid=0` と表示します。
  `eRPM=0 valid=1` はモータ停止を意味する正常データです。
- **eRPMは監視専用**です。PID/姿勢制御/モータ指令/CH5の38列リングバッファに入力しません。
- eRPMは電気的回転数です。機械的RPMはモータの極対数で除して計算します。
- 診断情報の表示はCore 0の通常のメインループで実行します。
  **400 Hz制御割り込み内ではUSB出力・eRPM待機を行いません**。
- 送信・受信PIO実装は [pio-dshot](https://github.com/simonwunderlich/pio-dshot) を参考にした
  MITライセンスのコードです。ライセンス表記は `DSHOT_UPSTREAM_LICENSE.txt` に保存しています。
- 元のPWM制御版はコミット `b0c9826b3bca7c423c13f6b8884542189809b513` です。
  先の一方向DShot300版は `01cad80ac3fd60f4d3a2e7cc3f981279db3e04e0` です。

**実機試験ではプロペラを取り外してください。** ESCが双方向DShot300に設定されていること、
停止指令0・モータ4基の対応・反転信号の論理レベルを確認してください。
受信処理にタイムアウトを設けた構造ですが、400 Hz制御の周期・ジッタと
実際のeRPM受信成功率は未測定です。ビルド成功は実機動作を保証しません。

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
firmware/pico_copter/build/pico_copter.uf2（設定した基板向け）
```

## 依存関係

| 項目 | 固定値 |
|---|---|
| 対象ボード | `pico` / `pico2` |
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

1. Pagesで対象基板を選び、Pico 2なら`pico_copter-pico2.uf2`をダウンロードする
2. Picoの`BOOTSEL`ボタンを押したままUSB接続する
3. 表示された`RPI-RP2`ドライブへUF2をコピーする
4. コピー後、Picoが自動的に再起動する
