# ET- team setup

このリポジトリは `spike-rt` をルートとして使う想定です。
チームメンバーは clone 後、repo 直下または `sdk/workspace/etrobo_app_0` からビルド/書き込みできます。

## 初回セットアップ

```sh
git clone --recurse-submodules https://github.com/nakamoto007/ET-.git
cd ET-
```

すでに clone 済みで submodule が空の場合:

```sh
git submodule update --init --recursive
```

Python の USB 書き込み依存を入れる場合:

```sh
python3 -m venv tools/python
tools/python/bin/pip install -r tools/requirements.txt
```

`arm-none-eabi-g++` は `PATH` に入れるか、次のどちらかに置きます。

```text
../gcc-arm-none-eabi-10.3-2021.10/bin
./gcc-arm-none-eabi-10.3-2021.10/bin
```

## ビルド

repo 直下:

```sh
make
```

アプリ直下:

```sh
cd sdk/workspace/etrobo_app_0
make
```

成功の目印:

```text
configuration check passed
```

## 書き込み

Hub を DFU モードにしてから実行します。

repo 直下:

```sh
make up
```

ビルド済みの `asp.bin` だけを書き込む場合:

```sh
make upload
```

アプリ直下でも同じです。

```sh
cd sdk/workspace/etrobo_app_0
make up
make upload
```

## DFU モード

1. USB ケーブルを抜く
2. Hub の Bluetooth ボタンを押したままにする
3. ボタンを押したまま USB ケーブルを PC に接続する
4. DFU モードになったらボタンを離す

`spike` コマンドが使える環境では確認できます。

```sh
make device
```
