gcc -o read read_setting_from_eeprom.c -L. -lftd2xx -Wl,-rpath,/usr/local/lib
gcc -o write write_setting_into_eeprom.c -L. -lftd2xx -Wl,-rpath,/usr/local/lib
gcc -o writefor20x_right write_setting_for20x_right.c -L. -lftd2xx -Wl,-rpath,/usr/local/lib
gcc -o writefor20x_left  write_setting_for20x_left.c -L. -lftd2xx -Wl,-rpath,/usr/local/lib
