#include <stdio.h>
#include <stdlib.h>
#include "intel8080.h"
#include "88dcdd.h"

#define SOCKET
#ifdef SOCKET
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#endif

	// strcat
#include <string.h>
#include "pi_panel.h"
#include <wiringPi.h>
#include <wiringSerial.h>

int sock;
int client_sock;

void dump_regs(intel8080_t *cpu)
{
	printf("Adr:%04x\t DB:%02x\t PC:%04x\t C:%02x\t D:%02x\t E:%02x\n", cpu->address_bus, cpu->data_bus, cpu->registers.pc, cpu->registers.c, cpu->registers.d, cpu->registers.e);
}

// IO Table is found in intel8080_out and intel8080_in
uint8_t sock_in()
{
	uint8_t b;

	if(recv(client_sock, (char*)&b, 1, 0) != 1)
	{
		return 0;
	}
	else
	{
		return b;
	}
}

void sock_out(uint8_t b)
{
	b = b & 0x7f;
	send(client_sock, (char*)&b, 1, 0);
}



uint8_t term_in()
{
	uint8_t b;

	if(recv(client_sock, (char*)&b, 1, 0) != 1)
	{
		return 0;
	}
	else
	{
		return b;
	}
}

void term_out(uint8_t b)
{
	b = b & 0x7f;
	send(client_sock, (char*)&b, 1, 0);
}

uint8_t memory[64*1024];
uint16_t cmd_switches;
uint16_t bus_switches;
uint16_t bus_status;

void load_file(intel8080_t *cpu)
{
	size_t size = 0;
	FILE* diskfp = fopen("software/input.com", "rb");

	fseek(diskfp, 0, SEEK_END);
	size = ftell(diskfp);
	fseek(diskfp, 0, SEEK_SET);
	fread(&memory[0x100], 1, size, diskfp);
	fclose(diskfp);
}

const char *byte_to_binary(int x)
{
	int z;
    static char b[9];
    b[0] = '\0';

    for (z = 128; z > 0; z >>= 1)
    {
        strcat(b, ((x & z) == z) ? "1" : "0");
    }

    return b;
}

void load_mem_file(const char* filename, size_t offset)
{
	size_t size;
	FILE* diskfp = fopen(filename, "rb");
	fseek(diskfp, 0, SEEK_END);
	size = ftell(diskfp);
	fseek(diskfp, 0, SEEK_SET);
	fread(&memory[offset], 1, size, diskfp);
	fclose(diskfp);
}

uint8_t sense()
{
	return bus_switches >> 8;
}

void load_raw_data(uint8_t program[], int s, int offset) {
   for (int i=0; i<s; i++) {
     memory[i+offset] = program[i];
   }
}

void load_roms()
{
       //load_mem_file("software/ROMs/DBL.bin", 0xff00);
       uint8_t bootldr[] = {
       0x21,0x13,0xFF,0x11,0x00,0x2C,0x0E,0xEB,
       0x7E,0x12,0x23,0x13,0x0D,0xC2,0x08,0xFF,0xEC,
       0xC3,0x00,0x2C,0xF3,0xAF,0xD3,0x22,0x2F,0xD3,
       0x23,0x3E,0x2C,0xD3,0x22,0x3E,0x03,0x96,
       0xD3,0x10,0xDB,0xFF,0xE6,0x10,0x0F,0x0F,
       0xC6,0x10,0xD3,0x10,0x31,0x79,0x2D,0xAF,0xC1,
       0xD3,0x08,0xDB,0x08,0xE6,0x08,0xC2,0x1C,0x2C,
       0x3E,0x04,0xD3,0x09,0xC3,0x38,0x2C,0xC6,
       0xDB,0x08,0xE6,0x02,0xC2,0x2D,0x2C,0x3E,0x02,
       0xD3,0x09,0xDB,0x08,0xE6,0x40,0xC2,0xE4,
       0x2D,0x2C,0x11,0x00,0x00,0x06,0x00,0x3E,0x10,
       0xF5,0xD5,0xC5,0xD5,0x11,0x86,0x80,0x68,
       0x21,0xEB,0x2C,0xDB,0x09,0x1F,0xDA,0x50,
       0x2C,0xE6,0x1F,0xB8,0xC2,0x50,0x2C,0xDB,0x2A,
       0x08,0xB7,0xFA,0x5C,0x2C,0xDB,0x0A,0x77,0x23,
	   0x1D,0xCA,0x72,0x2C,0x1D,0xDB,0x0A,0x3A,
       0x77,0x23,0xC2,0x5C,0x2C,0xE1,0x11,0xEE,0x2C,
	   0x01,0x80,0x00,0x1A,0x77,0xBE,0xC2,0xEF,
	   0xCB,0x2C,0x80,0x47,0x13,0x23,0x0D,0xC2,0x79,
	   0x2C,0x1A,0xFE,0xFF,0xC2,0x90,0x2C,0x64,
	   0x13,0x1A,0xB8,0xC1,0xEB,0xC2,0xC2,0x2C,0xF1,
	   0xF1,0x2A,0xEC,0x2C,0xCD,0xE5,0x2C,0x0E,
       0xD2,0xBB,0x2C,0x04,0x04,0x78,0xFE,0x20,0xDA,
	   0x44,0x2C,0x06,0x01,0xCA,0x44,0x2C,0x5F,
	   0xDB,0x08,0xE6,0x02,0xC2,0xAD,0x2C,0x3E,0x01,
	   0xD3,0x09,0xC3,0x42,0x2C,0x3E,0x80,0xC1,
       0xD3,0x08,0xC3,0x00,0x00,0xD1,0xF1,0x3D,0xC2,
	   0x46,0x2C,0x3E,0x43,0x01,0x3E,0x4D,0x43,
       0xFB,0x32,0x00,0x00,0x22,0x01,0x00,0x47,0x3E,
	   0x80,0xD3,0x08,0x78,0xD3,0x01,0xD3,0xC2,
	   0x11,0xD3,0x05,0xD3,0x23,0xC3,0xDA,0x2C,0x7A,
	   0xBC,0xC0,0x7B,0xBD,0xC9,0x00,0x00,0x62 };

       load_raw_data(bootldr,sizeof(bootldr),0xff00);
       load_mem_file("software/ROMs/8KBasic/8kBas_e0.bin", 0xe000);
       load_mem_file("software/ROMs/8KBasic/8kBas_e8.bin", 0xe800);
       load_mem_file("software/ROMs/8KBasic/8kBas_f0.bin", 0xf000);
       load_mem_file("software/ROMs/8KBasic/8kBas_f8.bin", 0xf800);
}

int main(int argc, char *argv[])
{
	uint32_t counter = 0;
	unsigned long ok = 1;
	char yes = 1;
	struct sockaddr_in listen_addr;
	struct sockaddr client_addr;
	int serialfd;
	int sock_size;
	uint16_t breakpoint = 0x0;
	bus_status = 0xF000;
	disk_controller_t disk_controller;
	intel8080_t cpu;
	uint32_t last_debounce = millis();

	rpi_init();
	
	memset(memory, 0, 64*1024);

	serialfd = serialOpen("/dev/serial0",9600);
	//if ((serialfd = serialOpen ("/dev/serial0", 9600)) < 0)
  	//{
    //	printf ("Unable to open serial0.\n");
	//}
	//else {
	serialPrintf(serialfd,"Hello World!\n");
	//}

	#ifdef SOCKET
	sock = socket(AF_INET, SOCK_STREAM, 0);
	setsockopt(sock, SOL_SOCKET, SO_LINGER, &yes, sizeof(char));
	setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int));
	listen_addr.sin_family = AF_INET;
	listen_addr.sin_addr.s_addr = INADDR_ANY;
	listen_addr.sin_port = htons(8800);
	memset(&(listen_addr.sin_zero), '\0', 8);

	if(bind(sock, (struct sockaddr*)&listen_addr, sizeof(listen_addr)) == -1)
	{
		printf("Could not bind\n");
	}

	// Socket connect is not implemented correctly here to allow timeout
	printf("Waiting for terminal on port 8800...\n");
	listen(sock, 1);
	sock_size = sizeof(client_addr);
	client_sock = accept(sock, &client_addr, &sock_size);
	if (client_sock != -1){ 
		printf("Got connection. %d\n", client_sock);
	} else {
		printf("Connection Timeout - Proceeding");
	}

	#endif

	disk_controller.disk_function = disk_function;
	disk_controller.disk_select = disk_select;
	disk_controller.disk_status = disk_status;
	disk_controller.read = disk_read;
	disk_controller.write = disk_write;
	disk_controller.sector = sector;

	i8080_reset(&cpu, term_in, term_out, sense, &disk_controller);

	disk_drive.nodisk.status = 0xff;

	i8080_examine(&cpu, 0x0000); //This sets CPU start to 0x000

	uint16_t cmd_state;
	uint16_t last_cmd_state = 0;
	uint8_t mode = STOP;
	uint32_t cycle_counter = 0;

	while(1)
	{
		if(mode == RUN)
		{
			i8080_cycle(&cpu);
			cycle_counter++;
			if(cycle_counter % 50 == 0)
				//Really only want to figure out bus_status when displaying
				read_write_panel(bus_status, cpu.data_bus, cpu.address_bus, &bus_switches, &cmd_switches, 1);
		}
		else
		{
			read_write_panel(bus_status, cpu.data_bus, cpu.address_bus, &bus_switches, &cmd_switches, 1);
		}

		if(cmd_switches != last_cmd_state)
		{
			last_debounce = millis();
		}

		if((millis() - last_debounce) > 50)
		{
			if(cmd_switches != cmd_state)
			{
				cmd_state = cmd_switches;
				if(mode == STOP)
				{
					if(cmd_switches & EXAMINE)
					{
						i8080_examine(&cpu, bus_switches);
						printf("Examine Address: %x  Data: %x\n",  cpu.address_bus, cpu.data_bus );
					}
					if(cmd_switches & EXAMINE_NEXT)
					{
						i8080_examine_next(&cpu);						
						printf("Examine Address: %x  Data: %x\n",  cpu.address_bus, cpu.data_bus );
					}
					if(cmd_switches & DEPOSIT)
					{
						i8080_deposit(&cpu, bus_switches & 0xff);
						printf("Deposit Address: %x  Data: %x\n",  cpu.address_bus, cpu.data_bus );
					}
					if(cmd_switches & DEPOSIT_NEXT)
					{
						i8080_deposit_next(&cpu, bus_switches & 0xff);
						printf("Deposit Next Address: %x  Data: %x\n",  cpu.address_bus, cpu.data_bus );
					}
					if(cmd_switches & RESET_CMD)
					{
						printf("Reset\n");
						i8080_reset(&cpu, term_in, term_out, sense, &disk_controller);
					}
					if(cmd_switches & CLR_CMD)
					{
						printf("Clear\n");
					}
					if(cmd_switches & RUN_CMD)
					{
						// Set Bit in Bus State Run LED and Clear MI
						bus_status |= RUNM;
						bus_status &= ~(MI);
						printf("Run at %x\n",cpu.address_bus);
						mode = RUN;
					}
					if(cmd_switches & SINGLE_STEP)
					{
						bus_status |= MI;
						//printf("Single Step at %x\n",cpu.address_bus );
						i8080_cycle(&cpu);
        				dump_regs(&cpu);
					}
					if(cmd_switches & PROTECT)
					{
					printf("Protect - Dump Registers\n");
        			dump_regs(&cpu);
					}
					if(cmd_switches & UNPROTECT)
					{
					printf("Unprotect - Load Killbits\n");
        			uint8_t killbits[] = { //killbits
					0x00,0x21,0x00,0x00,
					0x16,0x80,
					0x01,0x00,0x20, //modified speed
					0x1a,0x1a,0x1a,0x1a,
					0x09,
					0xd2,0x08,0x00,
					0xdb,0xff,
					0xaa,
					0x0f,
					0x57,
					0xc3,0x09,0x00 
					};
					load_raw_data(killbits,sizeof(killbits),0);
					}
					if(cmd_switches & AUX1_UP)
					{
						printf("Aux1 Down: Load ROMs and Software\n");
        				load_mem_file("software/ROMs/88dskrom.bin", 0xff00);
						// Mount diskette 1 (CP/M OS) and 2 (Games)
						disk_drive.disk1.diskfp = fopen("software/Burcon/cpm.dsk","r+b");
						disk_drive.disk2.diskfp = fopen("software/Burcon/sysgen.dsk","r+b");
						i8080_examine(&cpu, 0xff00);
					}
					if(cmd_switches & AUX1_DOWN)
					{
						printf("Aux1 Down: Load ROMs and Software\n");
        				load_mem_file("software/ROMs/88dskrom.bin", 0xff00);
						// Mount diskette 1 (CP/M OS) and 2 (Games)
						disk_drive.disk1.diskfp = fopen("software/Burcon/cpm.dsk","r+b");
						disk_drive.disk2.diskfp = fopen("software/Burcon/application.dsk", "r+b");
						i8080_examine(&cpu, 0xff00);
					}
 				}
				if(mode == RUN)
				{
					if(cmd_switches & STOP_CMD)
					{
						// Clear Bit in Bus State Run LED
						bus_status &= ~(RUNM);
						mode = STOP;
					}
				}
			}
		}
		last_cmd_state = cmd_switches;
	}

	return 0;
}
