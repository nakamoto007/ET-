# etrobo_nao build/upload memo

## チームrepoから使う場合

`ET-` を `spike-rt` ルートとして clone した場合は、repo直下からそのまま実行できる。

```sh
cd <repo-root>
make setup
make
make up
```

`make up` はビルドしてから `asp.bin` を書き込む。ビルド済みのものだけ書き込む場合は次を使う。

```sh
make upload
```

## ビルド

初回clone後、またはsubmoduleが空の場合はrepo直下で先に実行する。

```sh
make setup
```

このrepoでは `git submodule update --init --recursive` を使わない。
`external/libpybricks/micropython/lib/btstack` まで初期化されると、SPIKE-RT側のbtstackと混ざってビルドが失敗する。
すでに実行した場合も `make setup` で戻せる。

おすすめ:

```sh
cd <repo-root>/sdk/workspace/etrobo_nao
make
```

SDKのworkspaceから直接実行する場合:

```sh
cd <repo-root>/sdk/workspace
make -C etrobo_nao build
```

`arm-none-eabi-g++ was not found` が出る場合は、ツールチェーンの場所をそのコマンドだけに指定する。
`ETROBO_TARGET_GCC` には `gcc-arm-none-eabi-10.3-2021.10` のフォルダ本体を指定する。
`bin` 直指定でも動くが、フォルダ本体指定を基本にする。
`/path/to/...` は仮の例なので、そのまま実行せず、実際に展開した場所に置き換える。

```sh
ETROBO_TARGET_GCC=/path/to/gcc-arm-none-eabi-10.3-2021.10 make
```

例: `ET-` と同じ `GitHub` フォルダに置いた場合:

```sh
ETROBO_TARGET_GCC="$HOME/GitHub/gcc-arm-none-eabi-10.3-2021.10" make
```

環境確認だけしたい場合:

```sh
make doctor
```

書き込みまで使うPCでは、PythonのUSB依存も入れる。

```sh
make setup-upload
```

ビルド成功の目印:

```text
configuration check passed
```

生成物:

```text
<repo-root>/sdk/workspace/asp.bin
```

`Makefile` は自分の配置場所からrepo rootとアプリ名を自動で計算するので、別PCへ移動しても中身を書き換える必要はない。

## SPIKE HubをDFUモードにする

1. USBケーブルを一度抜く
2. HubのBluetoothボタンを押したままにする
3. Bluetoothボタンを押したままUSBケーブルをPCに接続する
4. DFUモードになったらボタンを離す

DFUモード確認:

```sh
cd <repo-root>/sdk/workspace/etrobo_nao
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
cd <repo-root>/sdk/workspace/etrobo_nao
make up
```

`make up` は先に `etrobo_nao` をビルドしてから書き込む。

SDKのworkspaceから直接実行する場合は、初回だけでなく毎回ビルドも一緒に実行する。

```sh
cd <repo-root>/sdk/workspace
make -C etrobo_nao up
```

## ビルド+書き込み

Hubを先にDFUモードにしてから実行する。

```sh
cd <repo-root>/sdk/workspace/etrobo_nao
make up
```

## `cat: appdir: No such file or directory` が出たとき

`workspace` で `make upload` だけ実行していて、まだ `make img=etrobo_nao` が実行されていない状態。
`appdir` は `make img=etrobo_nao` が作るファイルなので、先にビルドする。

```text
cat: appdir: No such file or directory
Makefile:15: /Makefile.inc: No such file or directory
make: *** No rule to make target `/Makefile.inc'.  Stop.
```

直し方:

```sh
cd <repo-root>/sdk/workspace/etrobo_nao
make up
```

または:

```sh
cd <repo-root>/sdk/workspace
make -C etrobo_nao up
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
cd <repo-root>/sdk/workspace/etrobo_nao
make
make device
make upload
```

## 別PCへ移動した後のIDE設定

`build/` と `compile_commands.json` は生成物なので、別PCへコピーしなくてよい。
`compile_commands.json` はPCごとの絶対パスを含むため、移動先PCで必要になったときだけ再生成する。
(VS Codeの認識範囲の問題で発生している警告用なので、`compile_commands.json` がなくてもコンパイル自体はできます)

```sh
cd <repo-root>/sdk/workspace/etrobo_nao
bash gen_compile_commands.sh
```

VS Codeで `sdk/workspace/etrobo_nao` を開く場合は、C/C++拡張に `compile_commands.json` を参照させる。
IntelliSenseでコンパイラが見つからない場合は、先にツールチェーンを指定して `compile_commands.json` を再生成する。
`/path/to/...` は実際に展開した場所に置き換える。

```sh
cd <repo-root>/sdk/workspace/etrobo_nao
ETROBO_TARGET_GCC=/path/to/gcc-arm-none-eabi-10.3-2021.10 bash gen_compile_commands.sh
code .
```
