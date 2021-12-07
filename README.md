# TinyUSBのGR-ROSE対応のアプリをWindows上でデバッグ

TinyUSBでマウスとUSB-LANアダプタの複合デバイスを作っています。

<https://github.com/h7ga40/azure_iot_hub_rose>

しかし、Windowsに複合デバイスとして認識はされるものの両方動いていません。

そこで、USBデバイスの動作をデバッグするため、Visual Studioの環境を作ってみました。
GR-ROSEのIOレジスタを仮想メモリで疑似動作することで、Windows上でデバッグするものです。

ソフトの構成は、下記のようになっています。

- ``usb_device``がデバッグ対象のソフトで、dllとなっています。
- ``win_gr_rose``が上記dllを起動するexeです。
- ``usb_host``は``usb_device``と通信するためのUSBホストで、dllとなっています。

``usb_device``にUSBホストの機能が簡単な実装で入っています。
このUSBホストの機能を充実させるため``usb_host``と通信させようと考えています。

現状は、TinyUSBのRX65N用の実装にUSBホスト機能が無かったので作成中です。

## ライセンス

このソフトは、下記のソフトを含んでいます。ライセンスについてはそれぞれに従ってください。

|Folder|Software|
|-|-|
|softgun|[softgun](http://softgun.sourceforge.net/)|
|tinyusb|[tinyusb](https://www.tinyusb.org/)|
|tinyusb/lwip|[lwip](https://github.com/lwip-tcpip/lwip)|
