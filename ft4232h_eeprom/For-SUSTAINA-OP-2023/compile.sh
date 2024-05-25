gcc -o read read_setting_from_eeprom.c -L. -lftd2xx -Wl,-rpath,/usr/local/lib
gcc -o write write_setting_into_eeprom.c -L. -lftd2xx -Wl,-rpath,/usr/local/lib