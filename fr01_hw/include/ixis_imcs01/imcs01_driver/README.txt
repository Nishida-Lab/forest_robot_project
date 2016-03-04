Linux kernel 3.2 用 iMCs01 USB ドライバ 2012年9月14日版
株式会社イクシスリサーチ

***ファイルについて***

Linux kernel 2.6 用 iMCs01 USB ドライバソース urbtc2.6ディレクトリ には以下のファ
イルが含まれています。


-imcs01_driver
  README.txt			//現在開いているファイル
  Makefile			//サンプルプログラム用Makefile
  h8test.c
  uread.c
  urobotc_ad.c
  urobotc_open_loop.c
  urobotm.c
  urobot_1ch_enc_pos.c
  urobot_enc_pos.c
─driver
    Makefile		//ビルド用の Makefile
    urbtc.c		//ドライバの本体
    urbtc.h		//ioctl 用のヘッダ
    urobotc.h		//read/write するデータを定義するヘッダ

***ビルドおよび動作環境***

Ubuntu1204で、ビルドおよび動作確認をしています。

ビルドには、uname -r でわかる現在動作中の kernel に対応する、
kernel-devel パッケージをインストールしておく必要があります。

$ uname -r
3.2.0-29-generic-pae

となるような環境では、linux-source-3.2.0 パッケージをあらかじめインストールしておいてください。
$ sudo apt-get install linux-source-3.2.0

カーネルソースをダウンロード後、以下を実施し、必要なファイル類の解凍を行います。
$ cd /usr/src
$ sudo tar xf linux-source-3.2.0.tar.bz2

/boot以下の現在のカーネル環境の.configを取得します。
$ cd linux-source-3.2.0
$ sudo cp /boot/confing-3.2.0-30-generic-pae .config

その後、makeを実施します。
$ sudo make oldconfig
$ sudo make

***ビルド方法***

ビルドはドライバのソースを展開したディレクトリに移動し、

$ make

とします。うまくコンパイルが成功するとドライバモジュールファイル
urbtc.ko が作成されます。

適切な kernel-devel パッケージがインストールされていない場合、以下の
ように表示されて make が失敗します。

$ make
make -C /lib/modules/2.6.15-1.2054_FC5/build M=/disk/tonki/d50/proj/ixs-usb/work/mrt/urbtc-2.6 modules
make: *** /lib/modules/2.6.15-1.2054_FC5/build: No such file or directory.  Stop.
make: *** [default] Error 2


***動作確認***

kernel 2.4 用ドライバのソースに附属の、以下のプログラムについて動作
確認しています。

h8test
uread
urobot_1ch_enc_pos
urobot_enc_pos
urobotc_ad
urobotc_open_loop
urobotm

環境は Ubuntu12.04です。

手順は
$ sudo insmod urbtc.ko
としてドライバをロードしたのちに、デバイスを USB コネクタに差します。

/dev/urbtc0が作られることを確認します。
$ ls -l /dev/urbtc*

アクセス権を変更します。
$ sudo chmod 777 /dev/urbtc0

上記のプログラムを順次起動しています。
