#!/bin/sh
cd rars

./rars 1 /tmp/rars.log -nd 11 8 -d id_1 SmoothB2 120000 2000 id_2 WappuCar 140000 4000 id_3 Mafanja 130000 3000 id_4 Bulle 110000 10 id_5 OscCar2 140000 4000 id_6 DougE1 130000 3000 id_7 O1 110000 10 id_8 Apex8 120000 2000 id_9 Apex1 130000 300 id_10 Felix16 100000 4000 id_12 Bulle2 140000 4000 
# id_11 Dodger6 110000 10 
# id_13 Jocosa83 110000 10 id_14 J++ 120000 2000 id_15 Sparky5 140000 4000

#./rars 1 /tmp/rars.log -nd 4 8 -d id_2 SmoothB2 120000 2000 id_4 WappuCar 140000 4000 id_3 Mafanja 130000 
#3000 id_1 Human 100000 0 
# вылетает при старте
# SmoothB2 - номер начинается с 2-ки, нужно проверять на начальные символы имя машины в командной строке, не учитывая номер, или скорее считать, что SmoothB = SmoothB2

#./rars 1 /tmp/rars.log -nd 3 8 -d id_1 Bulle 110000 10  id_4 OscCar2 140000 4000 id_3 DougE1 130000 3000 
# id_2 Vector 120000 2000 
# выдает : Wappu car error 1

#./rars 1 /tmp/rars.log -nd 4 8 -d id_1 O1 110000 10 id_2 Apex8 120000 2000 id_3 Apex1 130000 300 id_4 Felix16 100000 4000

#./rars 1 /tmp/rars.log -nd 2 8 -d id_1 Dodger6 110000 10 id_3 Bulle2 140000 4000
# id_2 K2001 120000 2000
# id_4 K1999 130000 3000 
# K1999 Control Problem

# ./rars 1 /tmp/rars.log -nd 3 8 -d id_1 Jocosa83 110000 10 id_2 J++ 120000 2000  id_4 Sparky5 140000 4000
# id_3 Djoefe 130000 3000
# виснет со штуками дрюками
cd ..














