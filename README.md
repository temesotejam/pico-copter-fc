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
