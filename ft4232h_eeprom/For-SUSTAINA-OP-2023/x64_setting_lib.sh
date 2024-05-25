wget https://ftdichip.com/wp-content/uploads/2022/07/libftd2xx-x86_64-1.4.27.tgz
tar -xvf libftd2xx-x86_64-1.4.27.tgz
sudo cp -r release/build/lib* /usr/local/lib
sudo ln -s /usr/local/lib/libftd2xx.so.1.4.27 /usr/local/lib/libftd2xx.so
sudo chmod 0755 /usr/local/lib/libftd2xx.so.1.4.27
cp release/ftd2xx.h .
cp release/WinTypes.h . 
