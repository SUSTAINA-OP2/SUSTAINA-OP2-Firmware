<h1 align="center">
  EEPROM for USB to Quad RS-485 Conv. Module
</h1>

> [!NOTE]
> This directory provides EEPROM for the [USB-to-Quad-RS-485-Conv-Module](https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Electronics) to be installed in SUSTAINA-OP2&trade;.<br>
> EEPROM is written to the FT4232H (Quad High Speed USB to Multipurpose UART/MPSSE IC), using Windows OS specific software "FT_PROG".

## How to write EEPROM

### 0. Download
FT_PROG is available for download by clicking [here](https://ftdichip.com/wp-content/uploads/2024/06/FT_Prog_v3.12.54.665-Installer.zip).<br>
>FT_PROGは[こちら](https://ftdichip.com/wp-content/uploads/2024/06/FT_Prog_v3.12.54.665-Installer.zip)からダウンロードできます。

### 1. Connection
Connect the USB-to-Quad-RS-485-Conv-Module to USB and press "Scan and Parse".<br>
> USB-to-Quad-RS-485-Conv-ModuleをUSBに接続し、"Scan and Parse" を押す。<br>

<img src="https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware/assets/53966390/96ce1149-3986-4197-860e-82bbe84f72a5" width="320">

### 2. Loading
Select the configuration file (.xml) to be written from "Apply Template".<br>
> "Apply Template" から書き込む設定ファイル（.xml）を選択します。<br>

<img src="https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware/assets/53966390/9104fd29-58db-423c-af17-9357b8405289" width="320">
<img src="https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware/assets/53966390/58f684aa-0dbe-4e08-ade5-3854abab127c" width="320">

If the file is successfully loaded, the following notification will appear.<br>
> ファイルの読み込みに成功すると、以下の通知が表示されます。<br>

<img src="https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware/assets/53966390/21d84b86-34ec-4262-b393-404c68d00d71" width="320">

### 3. Write
Press "Program Devices" to enter the phase of writing files.<br>
> Program Devices" を押して、ファイルを書き込む段階に入る。<br>

<img src="https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware/assets/53966390/0387f058-0896-4429-b17d-79b36227dacd" width="320">

Press "Program" to write the configuration file. If "Programming Successfull" -> "Ready" is displayed, the file is successfully written.<br>
> "Program" を押して設定ファイルを書き込みます。Programming Successfull" -> "Ready" と表示されれば、ファイルの書き込みは成功です。<br>

<img src="https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware/assets/53966390/714ab833-5fd2-4bd1-8405-f4843e39869b" width="320">
<img src="https://github.com/SUSTAINA-OP/SUSTAINA-OP2-Firmware/assets/53966390/ede4dc5f-350e-46f5-8aea-0a4ff9b8b911" width="320">
