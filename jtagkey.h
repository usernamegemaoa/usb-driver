//to dev
#define JTAGKEY_TCK	0x01
#define JTAGKEY_TMS	0x02
#define JTAGKEY_TDI	0x10
#define JTAGKEY_LED     0x20
#define JTAGKEY_RD      0x40

//to host
#define JTAGKEY_TDO	0x01

int __attribute__ ((visibility ("hidden"))) jtagkey_transfer(WD_TRANSFER *tr, int fd, unsigned int request, int ppbase, int ecpbase, int num);
int __attribute__ ((visibility ("hidden"))) jtagkey_open(int num);
void __attribute__ ((visibility ("hidden"))) jtagkey_close(int handle);
