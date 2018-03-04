# Python materials for CentSDR

python関連のファイルです。

## DSPデザインファイル

フィルタ係数の設計に使用したJupyter notebookです。

- [SSB-Filter-Design.ipynb](SSB-Filter-Design.ipynb) : SSBモード用の1300Hz LPF IIRフィルタ設計
- [CW-Filter-Design.ipynb](CW-Filter-Design.ipynb) : CWモード用の150Hz LPF IIRフィルタ設計
- [TLV320AIC3204-1st-IIR-HPF.ipynb](TLV320AIC3204-1st-IIR-HPF.ipynb) : Codec DSP用DCリジェクトHPFフィルタ

## centsdr.py

CentSDRをスクリプトやコマンドラインから制御するための、モジュール兼コマンドです。
シリアル(USB-CDC)のコマンドを、Pythonスクリプトで行うものです。内部バッファの波形をプロットすることもできます。

## Requirement

- python 2.7
- pyserial

波形表示をするにはオプションで下記が必要です

- numpy
- matplotlib

## デバイスの指定

シリアルポートを指定する必要があります。環境変数`CENTSDR_DEVICE`、または`-d`オプションで指定します。スクリプトの冒頭の`DEFAULT_DEVICE`を修正してもOKです。

- `$ export CENTSDR_DEVICE=/dev/ttyACM1`
- `$ ./centsdr.py -d /dev/cu.usbmodem401 -p 0`
- `$ CENTSDR_DEVICE=/dev/cu.usbmodem401 ./centsdr.py -p 0`

## Usage

```
$ ./centsdr.py -h
Usage: centsdr.py [options]

Options:
  -h, --help            show this help message and exit
  -d DEV, --dev=DEV     device node (default from env var CENTSDR_DEVICE)
  -F FREQ, --freqeucy=FREQ
                        set tuning frequency
  -G GAIN, --gain=GAIN  gain (0-95)
  -V VOLUME, --volume=VOLUME
                        set volume
  -A AGC, --agc=AGC     set agc
  -M MODE, --mode=MODE  set modulation
  -C MODE, --channel=MODE
                        set channel
  -P, --power           show power
  -s, --show            show current setting
  -S SHOWARG, --show-arg=SHOWARG
                        show specific setting
  -p BUFFER, --plot=BUFFER
                        plot buffer
  -l, --loop            loop continuously
```

## 制御

コマンドラインからの制御例を示します。オプションは複数同時に指定できます。

### 周波数を設定する

```
$ ./centsdr.py -F 27500000
```

### 復調モードを設定する

```
$ ./centsdr.py -M fm
```

### ボリュームを設定する

```
$ ./centsdr.py -V 10
```

### AGCを設定する

```
$ ./centsdr.py -A mid
```

### RFゲインを設定する

```
$ ./centsdr.py -G 40
```

## データ取得

### 現在状態を取得する

```
$ ./centsdr.py -s
tune: 27500300
volume: 10
mode: fm
gain: 60
channel: 8
agc: manual
```

### 電力を取得する

```
$ ./centsdr.py -P
-73.1
```

### 波形を表示する

波形データが保存されているバッファ番号を指定します。

- 0: Capture buffer
- 1: Audio play buffer
- 2: intermediate buffer 1
- 3: intermediate buffer 2

```
$ ./centsdr.py -p 0
```

クローズは、ウィンドウのクローズボタンです。Ctrl-Cでは消えません。

`-l`を指定すると連続して表示を行います。

```
$ ./centsdr.py -p 0 -l
```

こちらの停止はCtrl-Cです。クローズボタンでは消えません。

<div align="center">
<img src="/doc/plot-waveform.png" width="480px">
</div>

## スクリプトからの使用例

モジュールとしてimportしてスクリプトで使用することができます。

```
from centsdr import CentSDR
sdr = CentSDR()
sdr.set_tune(27500000)
sdr.set_mode('fm')
sdr.set_volume(20)
```
