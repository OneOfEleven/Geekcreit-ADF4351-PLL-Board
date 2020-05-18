
#include "stmflash.h"

#define STM32_FLASH_SIZE			64		// FLASH size in k
//#define STM32_FLASH_SIZE			128

#define STM32_FLASH_BASE			0x08000000

#define EEPROM_ADDR					(STM32_FLASH_BASE + (1024 * STM32_FLASH_SIZE) - STM_SECTOR_SIZE)	// use last page in flash for eeprom emulation

#define WORDS_PER_SECTOR			(STM_SECTOR_SIZE / sizeof(uint16_t))

uint16_t sector_buffer[WORDS_PER_SECTOR];

void STMFLASH_Erase(void)
{	// eare the whole sector
	uint32_t error = 0;
	FLASH_EraseInitTypeDef flash_erase;
	flash_erase.NbPages     = 1;
	flash_erase.PageAddress = EEPROM_ADDR;
	flash_erase.TypeErase   = FLASH_TYPEERASE_PAGES;
	flash_erase.Banks       = FLASH_BANK_1;

	if (HAL_FLASH_Unlock() != HAL_OK)
		return;

	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR | FLASH_FLAG_EOP);

	if (FLASH_WaitForLastOperation(1000) != HAL_OK)
	{
		HAL_FLASH_Lock();
		return;
	}

	if (HAL_FLASHEx_Erase(&flash_erase, &error) != HAL_OK)
	{
		HAL_FLASH_Lock();
		return;
	}

	HAL_FLASH_Lock();
}

// offset_addr .. address
// pBuffer ...... data pointer
// length ....... number of half words (16 bits)
void STMFLASH_Read(const uint32_t offset_addr, uint16_t *pBuffer, const uint16_t length)
{
	uint16_t i;
	const uint16_t *addr = (uint16_t *)(EEPROM_ADDR + offset_addr);
	for (i = 0; i < length; i++)
		*pBuffer++ = *addr++;
}

// offset_addr .. address (this address must be a multiple of 2
// pBuffer ...... data pointer
// length ....... number of half words (16 bits)
void STMFLASH_Write(uint32_t offset_addr, uint16_t *pBuffer, uint16_t length)
{
	FLASH_EraseInitTypeDef flash_erase;
	flash_erase.NbPages     = 1;
	//flash_erase.PageAddress = flash_addr;
	flash_erase.TypeErase   = FLASH_TYPEERASE_PAGES;
	flash_erase.Banks       = FLASH_BANK_1;

	if (offset_addr >= (1024 * STM32_FLASH_SIZE))
		return;

	if ((offset_addr + (sizeof(uint16_t) * length)) > (1024 * STM32_FLASH_SIZE))
		return;

	if (HAL_FLASH_Unlock() != HAL_OK)
		return;

	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR | FLASH_FLAG_EOP);

	if (FLASH_WaitForLastOperation(1000) != HAL_OK)
	{
		HAL_FLASH_Lock();
		return;
	}

	// sector by sector
	while (length > 0)
	{
		uint16_t i;

		const uint32_t sector_num = offset_addr / STM_SECTOR_SIZE;
		const uint16_t word_offset = (offset_addr % STM_SECTOR_SIZE) / sizeof(uint16_t);

		uint16_t word_num = WORDS_PER_SECTOR - word_offset;
		if (word_num > length)
			word_num = length;

		// read the sector
		STMFLASH_Read(STM_SECTOR_SIZE * sector_num, sector_buffer, WORDS_PER_SECTOR);

		// erase the sector if need be
		for (i = 0; i < word_num; i++)
		{
			//if (sector_buffer[word_offset + i] != 0xffff)
			if ((sector_buffer[word_offset + i] & pBuffer[i]) != pBuffer[i])
			{	// sector needs erasing
				uint32_t error = 0;
				flash_erase.PageAddress = EEPROM_ADDR + (STM_SECTOR_SIZE * sector_num);
				if (HAL_FLASHEx_Erase(&flash_erase, &error) != HAL_OK)
				{
					HAL_FLASH_Lock();
					return;
				}
				break;
			}
		}

		// write the sector
		uint32_t addr = EEPROM_ADDR + (STM_SECTOR_SIZE * sector_num) + offset_addr;
		for (i = 0; i < word_num; i++)
		{
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr, *pBuffer++);
			addr += sizeof(uint16_t);
		}

		offset_addr += word_num;
		length      -= word_num;
	}

	HAL_FLASH_Lock();
}
