# チームメンバー向けセットアップ・実行手順

このrepoは `ET-` 自体を `spike-rt` ルートとして使います。
PC上ではビルドを行い、実際の実行は生成された `asp.bin` を SPIKE Hub に書き込んで行います。

## 先に用意するもの

- Git
- make
- Python 3
- gcc-arm-none-eabi-10.3-2021.10
- SPIKE Hub とデータ通信対応USBケーブル

`gcc-arm-none-eabi-10.3-2021.10` は、SPIKE Hub向けの実行ファイルを作るためのARM用ツールチェーンです。
このフォルダの中に `arm-none-eabi-g++` などが入っています。

## 初回セットアップ

repoをcloneします。

```sh
git clone https://github.com/nakamoto007/ET-.git
cd ET-
```

必要なsubmoduleを準備します。

```sh
make setup
```

このrepoでは次のコマンドは使わないでください。

```sh
git submodule update --init --recursive
```

`--recursive` を使うと `external/libpybricks/micropython/lib/btstack` まで初期化され、SPIKE-RT側のbtstackと混ざってビルドが失敗することがあります。
すでに実行してしまった場合も、repo直下で次を実行すれば戻せます。

```sh
make setup
```

## ツールチェーンの置き方

`gcc-arm-none-eabi-10.3-2021.10` は、次のどちらかに置くと `make` が自動で見つけます。

```text
<repo-root>/gcc-arm-none-eabi-10.3-2021.10
<repo-root>の1つ上/gcc-arm-none-eabi-10.3-2021.10
```

別の場所に置いた場合は、コマンド実行時だけ場所を指定します。
`/actual/path/...` は実際に置いた場所へ置き換えてください。

```sh
ETROBO_TARGET_GCC=/actual/path/gcc-arm-none-eabi-10.3-2021.10 make
```

`ETROBO_TARGET_GCC` は `gcc-arm-none-eabi-10.3-2021.10` のフォルダ本体を指定するのが基本です。
`bin` フォルダを直接指定しても動きます。

## 環境チェック

セットアップできているか確認します。

```sh
make doctor
```

`arm-none-eabi-g++` やsubmoduleが `ok` ならビルドできます。
`pyusb` が `ng` の場合でも `make` は通りますが、Hubへの書き込みには追加セットアップが必要です。

## ビルド

repo直下で実行します。

```sh
make
```

成功すると最後に次の表示が出ます。

```text
configuration check passed
```

主な生成物は次です。

```text
sdk/workspace/asp.bin
sdk/workspace/appdir
sdk/workspace/etrobo_app_0/build/
build/
```

Hubに書き込む本体は `sdk/workspace/asp.bin` です。
これらは生成物なのでGitHubには上げません。

## Hubに書き込んで実行する

初回だけ、PythonのUSB書き込み依存を入れます。

```sh
make setup-upload
```

SPIKE HubをDFUモードにします。

1. USBケーブルを抜く
2. HubのBluetoothボタンを押したままにする
3. ボタンを押したままUSBケーブルをPCに接続する
4. DFUモードになったらボタンを離す

`spike` コマンドが使える環境では、DFUモードを確認できます。

```sh
make device
```

ビルドしてから書き込む場合:

```sh
make up
```

すでにビルド済みの `asp.bin` だけを書き込む場合:

```sh
make upload
```

## よくあるエラー

### `arm-none-eabi-g++ was not found`

ツールチェーンが見つかっていません。
`gcc-arm-none-eabi-10.3-2021.10` をrepo直下かrepoの1つ上に置くか、次のように場所を指定します。

```sh
ETROBO_TARGET_GCC=/actual/path/gcc-arm-none-eabi-10.3-2021.10 make
```

### submoduleやbtstack関連で失敗する

repo直下で次を実行します。

```sh
make setup
```

`git submodule update --init --recursive` は使わないでください。

### `pyusb` がない

Hubへの書き込み用Python依存がありません。

```sh
make setup-upload
```

### `No DFU device found`

ビルドは成功していて、Hub書き込みだけ失敗しています。
HubがDFUモードか、USBケーブルがデータ通信対応か、USBハブ経由ではなくPCへ直接接続しているかを確認してください。

### `asp.bin was not found`

まだビルドしていません。

```sh
make
```

その後、書き込みだけ行います。

```sh
make upload
```

## クリーン

通常の生成物を消します。

```sh
make clean
```

カーネル側の生成物も含めて消します。

```sh
make realclean
```