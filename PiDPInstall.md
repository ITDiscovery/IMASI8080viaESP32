From new RPi Installation command line:

```
cd /home/pi
git clone https://github.com/open-simh/simh.git
cd simh
make
```

Lots of waiting...the executables are built in /home/pi/simh/BIN. Make a "systems" directory and download the dsk files via:
```
mkdir systems
cd systems
wget https://deramp.com/downloads/mfe_archive/010-S100%20Computers%20and%20Boards/00-MITS/40-Software/CPM%202-2%20for%20Altair/Binaries/cpm2.dsk
wget https://deramp.com/downloads/mfe_archive/010-S100%20Computers%20and%20Boards/00-MITS/40-Software/CPM%202-2%20for%20Altair/Binaries/app.dsk
```
Now launch the altairz80 simulator via:
```
/home/pi/simh/BIN/altairz80
```
Now at the "sim>" prompt load up those two disks in the floppy drives and setup the simulated environment:
```
d tracks[0-7] 254
attach dsk0 cpm2.dsk
attach dsk1 app.dsk
set cpu 64k
set cpu noitrap
set cpu z80
set cpu altairrom
set cpu nonbanked
reset cpu
set sio ansi
set sio nosleep
boot dsk
```
Now on to installing the Blinkenboard Server
```
cd /home/pi/simh
git clone https://github.com/j-hoppe/BlinkenBone.git
sudo apt-get install ant



```
