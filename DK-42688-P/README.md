# smartmotion_exaple_algo
## 利用方法
1. microchip studioをインストール
2. DK-42688-P用のソフトウェアサンプルをダウンロード https://invensense.tdk.com/products/dk-42688-p/#documentation
3. その中の、sources/examples/example-algoに、このリポジトリの中身をコピー
4. microchip studioで、プロジェクトを開いてビルド、書き込み

# Setting of jumper pins on the board

Refer to section 3.1.3 of [SmartMotion_ICM42688P_Software_Guide.pdf](https://invensense.tdk.com/wp-content/uploads/2020/05/SmartMotion_ICM42688P_Software_Guide.pdf) and install the jumpers to be in the platform configuration when communicating with ICM through SPI.

> 3.1.3. Illustration of the configuration for SPI and I2C <br>
> Platform configuration when communicating with ICM through SPI: <br>
> <img src="https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware/assets/53966390/e629ce14-9455-4f89-a479-35e42c1d39ed" width="320px">

In SUSTAINA-OP2&trade;, solder is used to short-circuit as follows to prevent communication from being disrupted by vibrations.

<img src="https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware/assets/53966390/6ea3fcf4-6370-4871-98a5-26017b1bbd32" width="320px">
<img src="https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware/assets/53966390/b5d7650c-390e-45f6-8f39-34996e999aac" width="320px">
<img src="https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware/assets/53966390/e030bbc9-09a2-4851-8d67-acee906f8360" width="320px">
