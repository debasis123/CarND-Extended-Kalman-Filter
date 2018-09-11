#! /bin/bash
sudo unlink /usr/lib/libuWS.so
sudo rm /usr/lib64/libuWS.so
sudo apt-get update
#sudo apt-get install git libuv1-dev libssl-dev gcc g++ cmake make zlib1g-dev
git clone https://github.com/uNetworking/uWebSockets 
cd uWebSockets
git checkout e94b6e1
mkdir build
cd build
cmake ..
make 
sudo make install
cd ../..
sudo ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
sudo rm -r uWebSockets
