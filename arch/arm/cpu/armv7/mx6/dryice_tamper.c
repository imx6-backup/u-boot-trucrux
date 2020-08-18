#include <common.h>
#include <asm/io.h>
#include <asm/system.h>
#include <asm/arch/snvs.h>


extern void active_tamper_test(unsigned int tx, unsigned int rx);
extern void passive_tamper_test(unsigned int rx);

static void get_tamper_status(void)
{
	unsigned int lpsr, lptdsr, hpsr, ssm;

	lpsr = readl(SNVS_LPSR);
	lptdsr = readl(SNVS_LPTDSR);
	hpsr = readl(SNVS_HPSR);
	ssm = (hpsr & 0xf00) >> 8;

	if (lpsr & (1 << 9))
		printf("External Tampering 1 Detected\n");
	if (lpsr & (1 << 10))
		printf("External Tampering 2 Detected\n");
	if (lptdsr & (1 << 0))
		printf("External Tampering 3 Detected\n");
	if (lptdsr & (1 << 1))
		printf("External Tampering 4 Detected\n");
	if (lptdsr & (1 << 2))
		printf("External Tampering 5 Detected\n");
	if (lptdsr & (1 << 3))
		printf("External Tampering 6 Detected\n");
	if (lptdsr & (1 << 4))
		printf("External Tampering 7 Detected\n");
	if (lptdsr & (1 << 5))
		printf("External Tampering 8 Detected\n");
	if (lptdsr & (1 << 6))
		printf("External Tampering 9 Detected\n");
	if (lptdsr & (1 << 7))
		printf("External Tampering 10 Detected\n");
	if (!(lpsr & (3 << 9)) && !(lptdsr & 0xff))
		printf("No External Tampering Detected\n");

	if (hpsr & 0x80000000)
		printf("Zeroizable Master Key is clear\n");
	else
		printf("Zeroizable Master Key is not zero\n");

	if (ssm == 0)
		printf("System Security Monitor State: Init\n");
	else if (ssm == 0x8)
		printf("System Security Monitor State: Init Intermediate\n");
	else if (ssm == 0x9)
		printf("System Security Monitor State: Check\n");
	else if (ssm == 0xb)
		printf("System Security Monitor State: Non-Secure\n");
	else if (ssm == 0xd)
		printf("System Security Monitor State: Trusted\n");
	else if (ssm == 0xf)
		printf("System Security Monitor State: Secure\n");
	else if (ssm == 0x3)
		printf("System Security Monitor State: Soft Fail\n");
	else if (ssm == 0x1)
		printf("System Security Monitor State: Hard Fail\n");
	else
		printf("System Security Monitor State: 0x%x\n", ssm);
}

static void clear_tamper_warning(void)
{
	unsigned int lpsr, lptdsr;

	lpsr = readl(SNVS_LPSR);
	lptdsr = readl(SNVS_LPTDSR);

	writel(lpsr, SNVS_LPSR);
	writel(lptdsr, SNVS_LPTDSR);
}

static int do_tamper_test(cmd_tbl_t *cmdtp, int flag, int argc, char *const argv[])
{
	const char *op = argc >= 2 ? argv[1] : NULL;
	unsigned int tx, rx;

	if (argc < 2)
		return CMD_RET_USAGE;

	if (!strcmp(op, "active")) {
		if (argc < 4)
			return CMD_RET_USAGE;

		tx = simple_strtoul(argv[2], NULL, 16);
		rx = simple_strtoul(argv[3], NULL, 16);
		//if ((tx > 9) || (tx < 5))
		//	return CMD_RET_USAGE;
		if ((tx > 9))
			return CMD_RET_USAGE;
		if ((rx > 9) || (rx == tx))
			return CMD_RET_USAGE;

		active_tamper_test(tx, rx);

	} else if (!strcmp(op, "passive")) {
		if (argc < 3)
		return CMD_RET_USAGE;

		rx = simple_strtoul(argv[2], NULL, 16);
		if (rx > 9)
			return CMD_RET_USAGE;

		passive_tamper_test(rx);

	} else if (!strcmp(op, "status")) {
		get_tamper_status();
	} else if (!strcmp(op, "clear")) {
		clear_tamper_warning();
	}

	return 0;
}


U_BOOT_CMD(
		tamper, CONFIG_SYS_MAXARGS, 0, do_tamper_test,
		"mx6ul tamper test",
		"active <tx rx>  - tx is active tamper pin from 9 ~ 5, \n"
		"    rx pin is from 9 ~ 0 and should not equal to tx pin\n"
		"passive <rx> ... - rx is passive tamper pin from 9 ~ 0\n"
		"status - Get tamper status\n"
		"clear - clear tamper warning\n"
	  );
