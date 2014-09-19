// Copyright 2011 INDILINX Co., Ltd.
//
// Jasmine is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Jasmine is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Jasmine. See the file COPYING.
// If not, see <http://www.gnu.org/licenses/>.


#include "jasmine.h"

static void sanity_check(void);
static BOOL32 is_bad_block(UINT32 const bank, UINT32 const vblk_offset);
static BOOL32 check_format_mark(void);
static void write_format_mark(void);
static void format(void);

UINT32 g_ftl_read_buf_id;
UINT32 g_ftl_write_buf_id;
static UINT32 g_target_row[NUM_BANKS];
static UINT32 g_target_bank;

static volatile UINT32 g_read_fail_count;
static volatile UINT32 g_program_fail_count;
static volatile UINT32 g_erase_fail_count;

static UINT32 g_scan_list_entries[NUM_BANKS];
static UINT32 g_bank_buffer[NUM_BANKS];

void ftl_open(void)
{
	sanity_check();

	uart_printf("LightNVM initializing!");
	uart_printf("Bytes per page: %u Bytes per VBLK: %u Bytes per sector: %u", BYTES_PER_PAGE, BYTES_PER_VBLK, BYTES_PER_SECTOR);

	// STEP 1 - read scan lists from NAND flash

	scan_list_t* scan_list = (scan_list_t*) SCAN_LIST_ADDR;
	UINT32 bank;

	// Since we are going to check the flash interrupt flags within this function, ftl_isr() should not be called.
	disable_irq();

	flash_clear_irq();	// clear any flash interrupt flags that might have been set

	for (bank = 0; bank < NUM_BANKS; bank++)
	{
		SETREG(FCP_CMD, FC_COL_ROW_READ_OUT);			// FC_COL_ROW_READ_OUT = sensing and data output
		SETREG(FCP_OPTION, FO_E);						// scan list was written in 1-plane mode by install.exe, so there is no FO_P
		SETREG(FCP_DMA_ADDR, scan_list + bank);			// target address should be DRAM or SRAM (see flash.h for rules)
		SETREG(FCP_DMA_CNT, SCAN_LIST_SIZE);			// number of bytes for data output
		SETREG(FCP_COL, 0);
		SETREG(FCP_ROW_L(bank), SCAN_LIST_PAGE_OFFSET);	// scan list was written to this position by install.exe
		SETREG(FCP_ROW_H(bank), SCAN_LIST_PAGE_OFFSET);	// Tutorial FTL always uses the same row addresses for high chip and low chip

		flash_issue_cmd(bank, RETURN_ON_ISSUE);			// Take a look at the source code of flash_issue_cmd() now.
	}

	// This while() statement waits the last issued command to be accepted.
	// If bit #0 of WR_STAT is one, a flash command is in the Waiting Room, because the target bank has not accepted it yet.
	while ((GETREG(WR_STAT) & 0x00000001) != 0);

	// Now, FC_COL_ROW_READ_OUT commands are accepted by all the banks.
	// Before checking whether scan lists are corrupted or not, we have to wait the completion of read operations.
	// This code shows how to wait for ALL the banks to become idle.
	while (GETREG(MON_CHABANKIDLE) != 0);

	// Now we can check the flash interrupt flags.

	for (bank = 0; bank < NUM_BANKS; bank++)
	{
		UINT32 num_entries = NULL;
		UINT32 result = OK;

		g_bank_buffer[bank] = 0;

		if (BSP_INTR(bank) & FIRQ_DATA_CORRUPT)
		{
			// Too many bits are corrupted so that they cannot be corrected by ECC.
			result = FAIL;
		}
		else
		{
			// Even though the scan list is not corrupt, we have to check whether its contents make sense.

			UINT32 i;

			num_entries = read_dram_16(&(scan_list[bank].num_entries));

			if (num_entries > SCAN_LIST_ITEMS)
			{
				result = FAIL;	// We cannot trust this scan list. Perhaps a software bug.
			}
			else
			{
				for (i = 0; i < num_entries; i++)
				{
					UINT16 entry = read_dram_16(&(scan_list[bank].list[i]));
					UINT16 pblk_offset = entry & 0x7FFF;

					if (pblk_offset == 0 || pblk_offset >= PBLKS_PER_BANK)
					{
						#if OPTION_REDUCED_CAPACITY == FALSE
						result = FAIL;	// We cannot trust this scan list. Perhaps a software bug.
						#endif
					}
					else
					{
						// Bit position 15 of scan list entry is high-chip/low-chip flag.
						// Remove the flag in order to make is_bad_block() simple.

						write_dram_16(&(scan_list[bank].list[i]), pblk_offset);
					}
				}
			}
		}

		if (result == FAIL)
		{
			mem_set_dram(scan_list + bank, 0, SCAN_LIST_SIZE);
			g_scan_list_entries[bank] = 0;
		}
		else
		{
			write_dram_16(&(scan_list[bank].num_entries), 0);
			g_scan_list_entries[bank] = num_entries;
		}
	}

	// STEP 2 - If necessary, do low-level format
	// format() should be called after loading scan lists, because format() calls is_bad_block().

	if (check_format_mark() == FALSE)
	{
		// When ftl_open() is called for the first time (i.e. the SSD is powered up the first time)
		// format() is called.

		format();
	}

	// STEP 4 - initialize global variables that belong to FTL

	g_ftl_read_buf_id = 0;
	g_ftl_write_buf_id = 0;
	g_target_bank = 0;

	for (bank = 0; bank < NUM_BANKS; bank++)
	{
		g_target_row[bank] = PAGES_PER_VBLK;
	}

	flash_clear_irq();

	// This example FTL can handle runtime bad block interrupts and read fail (uncorrectable bit errors) interrupts

	SETREG(INTR_MASK, FIRQ_DATA_CORRUPT | FIRQ_BADBLK_L | FIRQ_BADBLK_H);
	SETREG(FCONF_PAUSE, FIRQ_DATA_CORRUPT | FIRQ_BADBLK_L | FIRQ_BADBLK_H);

	enable_irq();
}

void ftl_read(UINT32 const lba, UINT32 const total_sectors)
{
	UINT32 bank, row, num_sectors_to_read, temp, vblk;

	UINT32 lpage_addr		= lba / SECTORS_PER_PAGE;	// logical page address
	UINT32 sect_offset 		= lba % SECTORS_PER_PAGE;	// sector offset within the page
	UINT32 sectors_remain	= total_sectors;

	vblk = lba / SECTORS_PER_VBLK;

//	uart_printf("READ: %u (%u) %u %u %u %u", lba, total_sectors, lpage_addr, sect_offset, vblk, SECTORS_PER_PAGE);

	while (sectors_remain != 0)	// one page per iteration
	{
		if (sect_offset + sectors_remain < SECTORS_PER_PAGE)
		{
			num_sectors_to_read = sectors_remain;
		}
		else
		{
			num_sectors_to_read = SECTORS_PER_PAGE - sect_offset;
		}

		temp = lpage_addr;

		if (temp != NULL)
		{
			bank = temp / PAGES_PER_BANK;	// most significant bits represent bank number
			row = vblk + lpage_addr % PAGES_PER_VBLK; //row address of the page to write to

			SETREG(FCP_CMD, FC_COL_ROW_READ_OUT);						// FC_COL_ROW_READ_OUT = sensing and data output
			SETREG(FCP_DMA_CNT, num_sectors_to_read * BYTES_PER_SECTOR);// byte count must be an integer multiple of 512
			SETREG(FCP_COL, sect_offset);								// data output does not necessarily start from the first sector of the page
			SETREG(FCP_DMA_ADDR, RD_BUF_PTR(g_ftl_read_buf_id));		// DRAM address (when FO_E is used, DMA starts at FCP_DMA_ADDR + FCP_COL*BYTES_PER_SECTOR)
			SETREG(FCP_OPTION, FO_P | FO_E | FO_B_SATA_R);				// FO_P = 2-plane mode, FO_E = ECC, FO_B_SATA_R = [flash -> DRAM] completion automatically triggers [DRAM -> host]
			SETREG(FCP_ROW_L(bank), row);								// NAND row address (when FO_P is used, automatically converted to 2*row and 2*row + 1)
			SETREG(FCP_ROW_H(bank), row);

			// If FO_B_SATA_R is used, the buffer manager waits for the completion of [flash -> DRAM] and then starts [DRAM -> host].
			// If FO_B_SATA_R is not used, [flash -> DRAM] is not monitored by the buffer manager.
			// When reading an FTL mapping table, do not use FO_B_SATA_R.

			// The buffer ID should increase by one every time a flash command with FO_B_SATA_R is issued,
			// because the buffer manager hardware was designed with circular buffer model.

			g_ftl_read_buf_id = (g_ftl_read_buf_id + 1) % NUM_RD_BUFFERS;	// circular buffer

			#if OPTION_FTL_TEST == 0
			{
				while (1)
				{
					UINT32 sata_id = GETREG(SATA_RBUF_PTR); // wait if the read buffer is full (slow host)

					if (g_ftl_read_buf_id != sata_id)
						break;
				}
			}
			#endif

			// Now that all the FCP registers contain valid values, we can call flash_issue_cmd().
			flash_issue_cmd(bank, RETURN_ON_ISSUE);
		}
		else
		{
			// The host is requesting to read a logical page that has never been written to.

			UINT32 next_read_buf_id = (g_ftl_read_buf_id + 1) % NUM_RD_BUFFERS;

			#if OPTION_FTL_TEST == 0
			while (next_read_buf_id == GETREG(SATA_RBUF_PTR));	// wait if the read buffer is full (slow host)
			#endif

            // fix bug @ v.1.0.6
            // Send 0xFF...FF to host when the host request to read the sector that has never been written.
            // In old version, for example, if the host request to read unwritten sector 0 after programming in sector 1, Jasmine would send 0x00...00 to host.
            // However, if the host already wrote to sector 1, Jasmine would send 0xFF...FF to host when host request to read sector 0. (ftl_read() in ftl_xxx/ftl.c)
			mem_set_dram(RD_BUF_PTR(g_ftl_read_buf_id) + sect_offset*BYTES_PER_SECTOR,
                         0xFFFFFFFF, num_sectors_to_read*BYTES_PER_SECTOR);

			while (GETREG(MON_CHABANKIDLE) != 0);	// This while() loop ensures that Waiting Room is empty and all the banks are idle.

			// bm_read_limit is automatically updated by Buffer Manager hardware when a flash command with FO_B_SATA_R is finished.
			// Now we are going to update bm_read_limit without any flash command. (forced update by firmware)
			// The while() statement above ensures that there is no flash command with FO_B_SATA_R in progress at the moment bm_read_limit
			// is updated by firmware. Without it, one or more flash commands with FO_B_SATA_R can be active now,
			// which will lead to a race condition between Buffer Manager and firmware.

			SETREG(BM_STACK_RDSET, next_read_buf_id);	// change bm_read_limit
			SETREG(BM_STACK_RESET, 0x02);				// change bm_read_limit

			g_ftl_read_buf_id = next_read_buf_id;
		}

		sect_offset = 0;
		sectors_remain -= num_sectors_to_read;
		lpage_addr++;
	}
}

void ftl_write(UINT32 const lba, UINT32 const total_sectors)
{
	UINT32 new_bank, new_buf, new_row, num_sectors_to_write, vblk;
	UINT32 write_buffer = 0;

	UINT32 lpage_addr	= lba / SECTORS_PER_PAGE;
	UINT32 sect_offset	= lba % SECTORS_PER_PAGE;
	vblk = lba / SECTORS_PER_VBLK;

//	uart_printf("WRITE: %u (%u) %u %u %u %u", lba, total_sectors, lpage_addr, sect_offset, vblk, SECTORS_PER_PAGE);

	UINT32 remain_sectors = total_sectors;

	while (remain_sectors != 0)
	{
		new_bank = lba / SECTORS_PER_BANK;
		new_row = vblk + lpage_addr % PAGES_PER_VBLK; //row address of the page to write to

		new_buf = g_bank_buffer[new_bank];
//		uart_printf("write br: %u %u", new_bank, new_row);

		num_sectors_to_write = remain_sectors;

		/* first write into buffer */
		if (sect_offset == 0) {

			mem_copy(WR_WRITE_BUF_PTR(new_bank, new_buf),
					WR_BUF_PTR(g_ftl_write_buf_id) + (sect_offset * BYTES_PER_SECTOR),
					4096);

		/* trigger write */
		} else if (sect_offset + 8 == SECTORS_PER_PAGE) {

			mem_copy(WR_WRITE_BUF_PTR(new_bank, new_buf) + (sect_offset * BYTES_PER_SECTOR),
					WR_BUF_PTR(g_ftl_write_buf_id) + (sect_offset * BYTES_PER_SECTOR),
					4096);

			write_buffer = 1;

		} else {
			mem_copy(WR_WRITE_BUF_PTR(new_bank, new_buf) + (sect_offset * BYTES_PER_SECTOR),
					WR_BUF_PTR(g_ftl_write_buf_id) + (sect_offset * BYTES_PER_SECTOR),
					4096);
		}

		if (write_buffer) {
			SETREG(FCP_CMD, FC_COL_ROW_IN_PROG);			// FC_COL_ROW_IN_PROG = data input and programming
			SETREG(FCP_OPTION, FO_P | FO_E | FO_B_W_DRDY);
			SETREG(FCP_DMA_ADDR, WR_WRITE_BUF_PTR(new_bank, new_buf));
			SETREG(FCP_DMA_CNT, BYTES_PER_PAGE);
			SETREG(FCP_COL, 0);
			SETREG(FCP_ROW_L(new_bank), new_row);
			SETREG(FCP_ROW_H(new_bank), new_row);

			flash_issue_cmd(new_bank, RETURN_ON_ISSUE);

			g_bank_buffer[new_bank] = (g_bank_buffer[new_bank] + 1) % 3;
		} 

		sect_offset = 0;
		remain_sectors -= num_sectors_to_write;
		lpage_addr++;

		while (GETREG(MON_CHABANKIDLE) != 0);	// This while() loop ensures that Waiting Room is empty and all the banks are idle.

		g_ftl_write_buf_id = (g_ftl_write_buf_id + 1) % NUM_WR_BUFFERS;		// Circular buffer

		SETREG(BM_STACK_WRSET, g_ftl_write_buf_id);	// change bm_read_limit
		SETREG(BM_STACK_RESET, 0x01);				// change bm_read_limit	
	}
}

void ftl_flush(void)
{
	// do nothing
}

static BOOL32 is_bad_block(UINT32 const bank, UINT32 const vblk_offset)
{
	// The scan list, which is installed by installer.c:install_block_zero(), contains physical block offsets of initial bad blocks.
	// Since the parameter to is_bad_block() is not a pblk_offset but a vblk_offset, we have to do some conversion.
	//
	// When 1-plane mode is used, vblk_offset is equivalent to pblk_offset.
	// When 2-plane mode is used, vblk_offset = pblk_offset / 2.
	// Two physical blocks 0 and 1 are grouped into virtual block 0.
	// Two physical blocks 2 and 3 are grouped into virtual block 1.
	// Two physical blocks 4 and 5 are grouped into virtual block 2.

#if OPTION_2_PLANE

	UINT32 pblk_offset;
	scan_list_t* scan_list = (scan_list_t*) SCAN_LIST_ADDR;

	pblk_offset = vblk_offset * NUM_PLANES;

	if (mem_search_equ_dram(scan_list + bank, sizeof(UINT16), g_scan_list_entries[bank], pblk_offset) < g_scan_list_entries[bank])
	{
		return TRUE;
	}

	pblk_offset = vblk_offset * NUM_PLANES + 1;

	if (mem_search_equ_dram(scan_list + bank, sizeof(UINT16), g_scan_list_entries[bank], pblk_offset) < g_scan_list_entries[bank])
	{
		return TRUE;
	}

	return FALSE;

#else

	scan_list_t* scan_list = (scan_list_t*) SCAN_LIST_ADDR;

	if (mem_search_equ_dram(scan_list + bank, sizeof(UINT16), g_scan_list_entries[bank], vblk_offset) < g_scan_list_entries[bank])
	{
		return TRUE;
	}

	return FALSE;

#endif
}

static BOOL32 check_format_mark(void)
{
	// This function reads a flash page from (bank #0, block #0) in order to check whether the SSD is formatted or not.

	#ifdef __GNUC__
	extern UINT32 size_of_firmware_image;
	UINT32 firmware_image_pages = (((UINT32) (&size_of_firmware_image)) + BYTES_PER_FW_PAGE - 1) / BYTES_PER_FW_PAGE;
	#else
	extern UINT32 Image$$ER_CODE$$RO$$Length;
	extern UINT32 Image$$ER_RW$$RW$$Length;
	UINT32 firmware_image_bytes = ((UINT32) &Image$$ER_CODE$$RO$$Length) + ((UINT32) &Image$$ER_RW$$RW$$Length);
	UINT32 firmware_image_pages = (firmware_image_bytes + BYTES_PER_FW_PAGE - 1) / BYTES_PER_FW_PAGE;
	#endif

	UINT32 format_mark_page_offset = FW_PAGE_OFFSET + firmware_image_pages;
	UINT32 temp;

	flash_clear_irq();	// clear any flash interrupt flags that might have been set

	SETREG(FCP_CMD, FC_COL_ROW_READ_OUT);
	SETREG(FCP_BANK, REAL_BANK(0));
	SETREG(FCP_OPTION, FO_E);
	SETREG(FCP_DMA_ADDR, FTL_BUF_ADDR); 	// flash -> DRAM
	SETREG(FCP_DMA_CNT, BYTES_PER_SECTOR);
	SETREG(FCP_COL, 0);
	SETREG(FCP_ROW_L(0), format_mark_page_offset);
	SETREG(FCP_ROW_H(0), format_mark_page_offset);

	// At this point, we do not have to check Waiting Room status before issuing a command,
	// because scan list loading has been completed just before this function is called.
	SETREG(FCP_ISSUE, NULL);

	// wait for the FC_COL_ROW_READ_OUT command to be accepted by bank #0
	while ((GETREG(WR_STAT) & 0x00000001) != 0);

	// wait until bank #0 finishes the read operation
	while (BSP_FSM(0) != BANK_IDLE);

	// Now that the read operation is complete, we can check interrupt flags.
	temp = BSP_INTR(0) & FIRQ_ALL_FF;

	// clear interrupt flags
	CLR_BSP_INTR(0, 0xFF);

	if (temp != 0)
	{
		return FALSE;	// the page contains all-0xFF (the format mark does not exist.)
	}
	else
	{
		return TRUE;	// the page contains something other than 0xFF (it must be the format mark)
	}
}

static void write_format_mark(void)
{
	// This function writes a format mark to a page at (bank #0, block #0).

	#ifdef __GNUC__
	extern UINT32 size_of_firmware_image;
	UINT32 firmware_image_pages = (((UINT32) (&size_of_firmware_image)) + BYTES_PER_FW_PAGE - 1) / BYTES_PER_FW_PAGE;
	#else
	extern UINT32 Image$$ER_CODE$$RO$$Length;
	extern UINT32 Image$$ER_RW$$RW$$Length;
	UINT32 firmware_image_bytes = ((UINT32) &Image$$ER_CODE$$RO$$Length) + ((UINT32) &Image$$ER_RW$$RW$$Length);
	UINT32 firmware_image_pages = (firmware_image_bytes + BYTES_PER_FW_PAGE - 1) / BYTES_PER_FW_PAGE;
	#endif

	UINT32 format_mark_page_offset = FW_PAGE_OFFSET + firmware_image_pages;

	mem_set_dram(FTL_BUF_ADDR, 0, BYTES_PER_SECTOR);

	SETREG(FCP_CMD, FC_COL_ROW_IN_PROG);
	SETREG(FCP_BANK, REAL_BANK(0));
	SETREG(FCP_OPTION, FO_E | FO_B_W_DRDY);
	SETREG(FCP_DMA_ADDR, FTL_BUF_ADDR); 	// DRAM -> flash
	SETREG(FCP_DMA_CNT, BYTES_PER_SECTOR);
	SETREG(FCP_COL, 0);
	SETREG(FCP_ROW_L(0), format_mark_page_offset);
	SETREG(FCP_ROW_H(0), format_mark_page_offset);

	// At this point, we do not have to check Waiting Room status before issuing a command,
	// because we have waited for all the banks to become idle before returning from format().
	SETREG(FCP_ISSUE, NULL);

	// wait for the FC_COL_ROW_IN_PROG command to be accepted by bank #0
	while ((GETREG(WR_STAT) & 0x00000001) != 0);

	// wait until bank #0 finishes the write operation
	while (BSP_FSM(0) != BANK_IDLE);
}

static void format(void)
{
	// This function is called upon the very first power-up of the SSD.
	// This function does the low-level format (i.e. FTL level format) of SSD.
	// A typical FTL would create its mapping table and the list of free blocks.
	// However, this example does nothing more than erasing all the free blocks.
	//
	// This function may take a long time to complete. For example, erasing all the flash blocks can
	// take more than ten seconds depending on the total density.
	// In that case, the host will declare time-out error. (no response from SSD for a long time)
	// A suggested solution to this problem is:
	// When you power-up the SSD for the first time, connect the power cable but not the SATA cable.
	// At the end of this function, you can put a call to led(1) to indicate that the low level format
	// has been completed. When the LED is on, turn off the power, connect the SATA cable, and turn on
	// the power again.

	UINT32 vblk_offset, bank;

	for (vblk_offset = 1; vblk_offset < VBLKS_PER_BANK; vblk_offset++)
	{
		for (bank = 0; bank < NUM_BANKS; bank++)
		{
			if (is_bad_block(bank, vblk_offset))
				continue;

			// You do not need to set the values of FCP_DMA_ADDR, FCP_DMA_CNT and FCP_COL for FC_ERASE.

			SETREG(FCP_CMD, FC_ERASE);
			SETREG(FCP_BANK, REAL_BANK(bank));
			SETREG(FCP_OPTION, FO_P);
			SETREG(FCP_ROW_L(bank), vblk_offset * PAGES_PER_VBLK);
			SETREG(FCP_ROW_H(bank), vblk_offset * PAGES_PER_VBLK);

			// You should not issue a new command when Waiting Room is not empty.

			while ((GETREG(WR_STAT) & 0x00000001) != 0);

			// By writing any value to FCP_ISSUE, you put FC_ERASE into Waiting Room.
			// The value written to FCP_ISSUE does not have any meaning.

			SETREG(FCP_ISSUE, NULL);
		}
	}

	uart_printf("formatted");
	// In general, write_format_mark() should be called upon completion of low level format in order to prevent
	// format() from being called again.
	// However, since the tutorial FTL does not support power off recovery,
	// format() should be called every time.

	#if 0
	write_format_mark();
	#endif

	led(1);
}

void ftl_isr(void)
{
	// interrupt service routine for flash interrupts

	UINT32 bank;
	UINT32 bsp_intr_flag;

	for (bank = 0; bank < NUM_BANKS; bank++)
	{
		while (BSP_FSM(bank) != BANK_IDLE);

		bsp_intr_flag = BSP_INTR(bank);

		if (bsp_intr_flag == 0)
		{
			continue;
		}

		UINT32 fc = GETREG(BSP_CMD(bank));

		CLR_BSP_INTR(bank, bsp_intr_flag);

		if (bsp_intr_flag & FIRQ_DATA_CORRUPT)
		{
			g_read_fail_count++;
		}

		if (bsp_intr_flag & (FIRQ_BADBLK_H | FIRQ_BADBLK_L))
		{
			if (fc == FC_COL_ROW_IN_PROG || fc == FC_IN_PROG || fc == FC_PROG)
			{
				g_program_fail_count++;
			}
			else
			{
				ASSERT(fc == FC_ERASE);
				g_erase_fail_count++;
			}
		}
	}

	// clear the flash interrupt flag at the interrupt controller
	SETREG(APB_INT_STS, INTR_FLASH);
}

static void sanity_check(void)
{
	UINT32 dram_requirement = RD_BUF_BYTES + WR_BUF_BYTES + COPY_BUF_BYTES + FTL_BUF_BYTES
		+ HIL_BUF_BYTES + TEMP_BUF_BYTES + SCAN_LIST_BYTES + WRITE_BUF_BYTES;

	if (dram_requirement > DRAM_SIZE)
	{
		uart_printf("Not enough DRAM available. Requires: %u", dram_requirement);
		while (1);
	}
}

void ftl_trim(UINT32 const lba, UINT32 const num_sectors)
{

}

void ftl_erase(UINT32 const lba, UINT32 const num_sectors)
{
	if (num_sectors == 0) /*protect first block on each bank */
		return;
	nand_block_erase_sync(lba, num_sectors);
}
