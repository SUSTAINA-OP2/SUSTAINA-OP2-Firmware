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
