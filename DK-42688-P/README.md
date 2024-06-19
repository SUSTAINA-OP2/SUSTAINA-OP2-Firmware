# smartmotion_exaple_algo
## 利用方法

1. git bashをインストールする
2. smatrmotionのソースコードをダウンロードする。
3. 本リポジトリ/DK-42688-Pに、上記ソースコード直下に入っているフォルダ群（画像以下）をコピーする
![image](https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware/assets/53361179/cc427450-6193-4239-9406-c468a534f6cd)

4. git bashを操作し、/DK-42688-Pまで移動する
5. bash apply_patch.bashを実行する
6. microchip-studioをインストールする
7. microchip-studioを起動し、File -> Open -> project/Solutionから、先ほどコピーしたsources/examples/example-algo/example-algo.cprojを開く
![image](https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware/assets/53361179/0593d849-9f44-45db-a394-ea8430699dff)
8. Build -> Build Solutionからビルドを行う
![image](https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware/assets/53361179/c2f9bde5-16ff-43d7-8df1-b146fa21027f)
9. Tools -> Device Programming -> Applyから書き込む
![image](https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware/assets/53361179/ec3f219e-8db8-41e4-98e7-7e61c7c76af0)






# Setting of jumper pins on the board

Refer to section 3.1.3 of [SmartMotion_ICM42688P_Software_Guide.pdf](https://invensense.tdk.com/wp-content/uploads/2020/05/SmartMotion_ICM42688P_Software_Guide.pdf) and install the jumpers to be in the platform configuration when communicating with ICM through SPI.

> 3.1.3. Illustration of the configuration for SPI and I2C <br>
> Platform configuration when communicating with ICM through SPI: <br>
> <img src="https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware/assets/53966390/e629ce14-9455-4f89-a479-35e42c1d39ed" width="320px">

In SUSTAINA-OP2&trade;, solder is used to short-circuit as follows to prevent communication from being disrupted by vibrations.

<img src="https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware/assets/53966390/6ea3fcf4-6370-4871-98a5-26017b1bbd32" width="320px">
<img src="https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware/assets/53966390/b5d7650c-390e-45f6-8f39-34996e999aac" width="320px">
<img src="https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware/assets/53966390/e030bbc9-09a2-4851-8d67-acee906f8360" width="320px">
