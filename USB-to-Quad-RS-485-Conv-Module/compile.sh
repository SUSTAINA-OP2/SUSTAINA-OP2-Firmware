gcc -o read read_setting_from_eeprom.c -L. -lftd2xx -Wl,-rpath,/usr/local/lib
gcc -o write write_setting_into_eeprom.c -L. -lftd2xx -Wl,-rpath,/usr/local/lib
gcc -o writefor20x_right write_setting_for20x_right.c -L. -lftd2xx -Wl,-rpath,/usr/local/lib
gcc -o writefor20x_left  write_setting_for20x_left.c -L. -lftd2xx -Wl,-rpath,/usr/local/lib
gcc -o write_setting_for_sensor_modules write_setting_for_sensor_modules.c -L. -lftd2xx -Wl,-rpath,/usr/local/lib
gcc -o reset_port  reset_port.c -L. -lftd2xx -Wl,-rpath,/usr/local/lib
gcc -o erase_eeprom  erase_eeprom.c -L. -lftd2xx -Wl,-rpath,/usr/local/lib
