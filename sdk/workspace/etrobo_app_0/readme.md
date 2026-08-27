# etrobo_app_0 build/upload memo

## チームrepoから使う場合

`ET-` を `spike-rt` ルートとして clone した場合は、repo直下からそのまま実行できる。

```sh
cd <repo-root>
make
make up
```

`make up` はビルドしてから `asp.bin` を書き込む。ビルド済みのものだけ書き込む場合は次を使う。

```sh
make upload
```

## ビルド

おすすめ:

```sh
cd <etrobo-root>/workspace/etrobo_app_0
make
```

SDKのworkspaceから直接実行する場合:

```sh
cd <etrobo-root>/workspace
make -C etrobo_app_0 build
```

ビルド成功の目印:

```text
configuration check passed
```

生成物:

```text
<etrobo-root>/spike-rt/sdk/workspace/asp.bin
```

`Makefile` は自分の配置場所から `<etrobo-root>` とアプリ名を自動で計算するので、別PCへ移動しても中身を書き換える必要はない。

## SPIKE HubをDFUモードにする

1. USBケーブルを一度抜く
2. HubのBluetoothボタンを押したままにする
3. Bluetoothボタンを押したままUSBケーブルをPCに接続する
4. DFUモードになったらボタンを離す

DFUモード確認:

```sh
cd <etrobo-root>/workspace/etrobo_app_0
make device
```

期待する表示:

```text
DFU
```

## 書き込み

HubがDFUモードになっていることを確認してから実行する。

おすすめ:

```sh
cd <etrobo-root>/workspace/etrobo_app_0
make up
```

`make up` は先に `etrobo_app_0` をビルドしてから書き込む。

SDKのworkspaceから直接実行する場合は、初回だけでなく毎回ビルドも一緒に実行する。

```sh
cd <etrobo-root>/workspace
make -C etrobo_app_0 up
```

## ビルド+書き込み

Hubを先にDFUモードにしてから実行する。

```sh
cd <etrobo-root>/workspace/etrobo_app_0
make up
```

## `cat: appdir: No such file or directory` が出たとき

`workspace` で `make upload` だけ実行していて、まだ `make img=etrobo_app_0` が実行されていない状態。
`appdir` は `make img=etrobo_app_0` が作るファイルなので、先にビルドする。

```text
cat: appdir: No such file or directory
Makefile:15: /Makefile.inc: No such file or directory
make: *** No rule to make target `/Makefile.inc'.  Stop.
```

直し方:

```sh
cd <etrobo-root>/workspace/etrobo_app_0
make up
```

または:

```sh
cd <etrobo-root>/workspace
make -C etrobo_app_0 up
```

## `No DFU device found` が出たとき

ビルドは成功していて、アップロードだけ失敗している状態。

```text
ValueError: No DFU device found
make: *** [upload] Error 1
```

確認すること:

- HubがDFUモードになっているか
- `spike device` が `DFU` を返すか
- USBケーブルがデータ通信対応か
- USBハブ経由ではなくPCへ直接接続しているか
- いったんUSBを抜いて、Bluetoothボタンを押したまま接続し直したか

切り分けの基本:

```sh
cd <etrobo-root>/workspace/etrobo_app_0
make
make device
make upload
```

## 別PCへ移動した後のIDE設定

`build/` と `compile_commands.json` は生成物なので、別PCへコピーしなくてよい。
`compile_commands.json` はPCごとの絶対パスを含むため、移動先PCで必要になったときだけ再生成する。
(VS Codeの認識範囲の問題で発生している警告用なので、`compile_commands.json` がなくてもコンパイル自体はできます)

```sh
cd <etrobo-root>/workspace/etrobo_app_0
bash gen_compile_commands.sh
```

VS Codeで `workspace/etrobo_app_0` を開く場合は `.vscode/c_cpp_properties.json` を使う。
`compilerPath` は `arm-none-eabi-g++` にしてあるので、IntelliSenseでコンパイラが見つからない場合は、先に次を実行してからVS Codeを起動する。

```sh
cd <etrobo-root>/workspace
code etrobo_app_0
```
