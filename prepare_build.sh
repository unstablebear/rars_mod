#!/bin/sh
apt-get install libqt3-headers libqt3-compat-headers libqt3-mt libqt3-mt-dev

apt-get remove gcc-4.2
apt-get install gcc-3.3
ln -s /usr/bin/gcc-3.3 /usr/bin/gcc

rm /usr/bin/cpp /lib/cpp
ln -s /usr/bin/cpp-3.3 /usr/bin/cpp
ln -s /usr/bin/cpp-3.3 /lib/cpp

apt-get install g++-3.3
ln -s /usr/bin/g++-3.3 /usr/bin/g++

apt-get install kde-devel

