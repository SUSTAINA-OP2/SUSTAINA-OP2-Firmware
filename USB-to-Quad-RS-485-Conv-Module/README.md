## Flash FT4232H EEPROM

### Setting emviroment

setting on jetson
``` sudo bash jetson_setting_lib.sh ```  

setting on linux x64
``` sudo bash x64_setting_lib.sh ```

compile code
``` bash compile.sh ```


### How to execute code

First unload Serial USB Drivers
```sudo modprobe -r ftdi_sio ``` 

Writing EEPROM ```sudo ./write```

Reading setting from EEPROM  ```sudo ./read```

After write eeprom, reload Drivers:
```sudo modprobe ftdi_sio```

## How to write EEPROM

1. ``` lsusb -t ``` を行いFT4232Hのバスの場所を確認する.
2.  AnkerのUSBハブの場合、ケーブルを左側、USBポートを上向きにした時、ポート番号は左から4,3,2,1である.ここで右半身はポート3、左半身はポート4である.
3.  ``` cd /sys/bus/usb/drivers/usb/ ``` に移動.
4.  ``` sudo echo "1-2.4.1" > unbind ``` と ``` sudo echo "1-2.4.2" > unbind ``` 他のバスをアンバインドする.
5.  ``` sudo echo  "1-2.4.3" > unbind ``` を行い、右半身のFT4232Hのバスをアンバインドする.
6.  ``` sudo ./read <番号> ``` のコマンドで左半身のFT4232Hのポート番号を確認する.ここでオープン成功した番号が左半身のFT4232Hのバスである.
7.  ``` sudo ./writefor20x_left <上で調べた番号> ``` を行い左半身のFT4232HのEEPROMに書き込む.
8.  ``` sudo echo "1-2.4.4" > bind ``` を行い、左半身のFT4232Hのバスをアンバインドする.
9.  ``` sudo echo "1-2.4.3" > bind ``` を行い、右半身のFT4232Hのバスをバインドする.
10. ``` sudo ./read <番号> ``` のコマンドで右半身のFT4232Hのポート番号を確認する.ここでオープン成功した番号が右半身のFT4232Hのバスである.
11. ``` sudo ./writefor20x_right <上で調べた番号> ``` を行い右半身のFT4232HのEEPROMに書き込む.
12. ``` sudo echo "1-2.4.1" > bind ``` と ``` sudo echo "1-2.4.2" > bind ``` と``` sudo echo "1-2.4.4" > bind ``` を行い、他のバスをバインドしなおす。
