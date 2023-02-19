# Introduction
Kill the Bit game by Dean McDaniel, May 15, 1975

Object: Kill the rotating bit. If you miss the lit bit, another bit turns on leaving two bits to destroy. Quickly
toggle the switch, don't leave the switch in the up position. Before starting, make sure all the switches are in the down position.

## Assembly

```
.originate 0x0000



```


## Hand Assembly

```
0000
0000 21 00 00   lxi h,0
0003 16 80      mvi h,80
0005 01 0E 00   lxi b,0E
0008 1A         ldax d
0009 1A         ldax d
000A 1A         ldax d
000B 1A         ldax d
000C 09         dad b
000D D2 08 00   jnc 0x0008     
0010 DB FF      in 0xff
0012 AA         xra d
0013 0F         rrc
0014 57         mov d,a
0015 C3 08 00   jmp 0x0000
```



byte killbits[] = {
       0x21,0x00,0x00,
       0x16,0x80,
       0x01,0x0E,0x00,
       0x1A,
       0x1A,
       0x1A,
       0x1A,
       0x09,
       0xD2,0x08,0x00,
       0xDB,0xFF,
       0xAA,
       0x0F,
       0x57,
       0xC3,0x08,0x00 };
       loadData(killbits,sizeof(killbits),0);
       examine(0);
```
