# smartmotion_exaple_algo

## install

以下をインストールする

- **FTDI driver**
    - ダウンロードリンク：https://ftdichip.com/drivers/vcp-drivers/
- **Microchip Studio**
    - ボードの書き込み用
    - ダウンロードリンク：https://www.microchip.com/en-us/tools-resources/develop/microchip-studio
- **git bash**
    - ダウンロードリンク：https://git-scm.com/downloads
      - https://github.com/git-for-windows/git/releases/download/v2.45.2.windows.1/Git-2.45.2-64-bit.exe
- **smart motion sample code**
    - ダウンロードリンク：https://invensense.tdk.com/products/dk-42688-p/
        - [DK-42688-P SmartMotion eMD](https://invensense.tdk.com/developers/download/dk-42688-p-smartmotion-emd-2-0-3/?wpdmdl=38397)をダウンロードする

## 利用方法

1. Git bashを起動する

2. 本リポジトリをcloneする
```
git clone https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware.git
```
3. ダウンロードしたDK-42688-P SmartMotion eMDを展開する

  展開後，以下のようなファイル構成になっている
  ![image](https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware/assets/53966346/70850185-8ef4-4afa-8214-9640c5c512a8)

3. `sources/examples/example-algo`以下のファイル群を本リポジトリの`/DK-42688-P`以下にコピーする
```
cp -r Downloads/smartmotion-dkexamples-atmel-cm4-fpu-2.0.9/* SUSTAINA-OP2-Firmware/DK-42688-P/
```

4. git bashを操作し、`/DK-42688-P`まで移動する
```
cd SUSTAINA-OP2-Firmware/DK-42688-P/
```
7. `apply_patch.bash`を実行する
```
bash apply_patch.bash
```
8. microchip-studioを起動し、File -> Open -> project/Solutionから、先ほどコピーした`sources/examples/example-algo/example-algo.cproj`を開く

![image](https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware/assets/53361179/0593d849-9f44-45db-a394-ea8430699dff)

9. Build -> Build Solutionからビルドを行う

![image](https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware/assets/53361179/c2f9bde5-16ff-43d7-8df1-b146fa21027f)

10. 後述する Setting of jumper pins on the board に従ってボードのピン配置を設定する
    
11. 画像のように，「FTDI USB」と「EDBG USB」のUSB ポートを使用し，PCと接続する．

<img src="https://github.com/citbrains/citbrains_humanoid/assets/53966346/5cdc0fe0-7457-4877-913e-01b1b06a02cc" width="50%" />

12. Tools -> Device Programming -> Applyから書き込む
![image](https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware/assets/53361179/ec3f219e-8db8-41e4-98e7-7e61c7c76af0)

![image](https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware/assets/53966346/2eb03aa2-3aa6-46a4-aeb3-46a63ea4b4d9)

![image](https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware/assets/53966346/1cb0359f-7555-42b1-8417-9f6f9816cf48)



# Setting of jumper pins on the board

Refer to section 3.1.3 of [SmartMotion_ICM42688P_Software_Guide.pdf](https://invensense.tdk.com/wp-content/uploads/2020/05/SmartMotion_ICM42688P_Software_Guide.pdf) and install the jumpers to be in the platform configuration when communicating with ICM through SPI.

> 3.1.3. Illustration of the configuration for SPI and I2C <br>
> Platform configuration when communicating with ICM through SPI: <br>
> <img src="https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware/assets/53966390/e629ce14-9455-4f89-a479-35e42c1d39ed" width="320px">

In SUSTAINA-OP2&trade;, solder is used to short-circuit as follows to prevent communication from being disrupted by vibrations.

<img src="https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware/assets/53966390/6ea3fcf4-6370-4871-98a5-26017b1bbd32" width="320px">
<img src="https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware/assets/53966390/b5d7650c-390e-45f6-8f39-34996e999aac" width="320px">
<img src="https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware/assets/53966390/e030bbc9-09a2-4851-8d67-acee906f8360" width="320px">
