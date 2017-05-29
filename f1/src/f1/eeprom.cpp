#include "eeprom.h"

uint8_t buffer[EEPROM_BUFFER_SIZE];

EEPROM::EEPROM()
{
    header_.magic_be = 0xBE;
    header_.size = 0;
    header_.version = EEPROM_VERSION;

    footer_.magic_ef = 0xEF;
    footer_.crc = 0;

    memset(buffer, 0, EEPROM_BUFFER_SIZE);
}

bool EEPROM::read(uint8_t *data, uint32_t len)
{
    header_.size = len;

    if (!is_valid())
    {
        return false;
    }
    else
    {
        memcpy(data, (uint8_t *) (FLASH_WRITE_ADDR + sizeof(eeprom_header_t)), (size_t) len);
        return true;
    }
}

bool EEPROM::write(const uint8_t *data, uint32_t len)
{
    FLASH_Status status;
    uint8_t chk = 0;

    // calculate checksum
    for (const uint8_t* p = (const uint8_t*) data; p < (const uint8_t *)data + len; p++)
    {
        chk ^= *p;
    }

    // Get the header and footer ready for writing
    footer_.crc = chk;
    header_.size = len;

    const uint8_t* header_pointer = (const uint8_t *)&header_;
    const uint8_t* footer_pointer = (const uint8_t *)&footer_;



    uint16_t i = 0;
    for (i = 0; i < sizeof(eeprom_header_t) + len + sizeof(eeprom_footer_t); i++)
    {
        if (i < sizeof(eeprom_header_t))
            buffer[i] = * (header_pointer + i);
        else if (i < sizeof(eeprom_header_t) + len)
            buffer[i] = * (data + i - sizeof(eeprom_header_t));
        else
            buffer[i] = * (footer_pointer + i - sizeof(eeprom_header_t) - len);
    }

    // Unlock the FLASH for writing
    FLASH_Unlock();

    // Write the data
    for (uint8_t tries = 3; tries; tries--)
    {
        FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

        FLASH_ErasePage(FLASH_WRITE_ADDR);
        status = FLASH_ErasePage(FLASH_WRITE_ADDR + FLASH_PAGE_SIZE);

        // Write the data to the FLASH
        uint8_t i = 0;
        for (i = 0; i < (sizeof(eeprom_header_t) + len + sizeof(eeprom_footer_t)) && status == FLASH_COMPLETE; i += 4)
        {
            status = FLASH_ProgramWord(FLASH_WRITE_ADDR + i, *(uint32_t *)(buffer + i));
        }

        if (status == FLASH_COMPLETE)
            break;
    }
    FLASH_Lock();

    // Give the lock just a few microseconds to clear
    delay_us(10);

    // Flash Write Failed - Restart
    if(status != FLASH_COMPLETE || !is_valid())
    {
        reboot();
        return false;
    }

    return true;
}

bool EEPROM::is_valid()
{
    // Get a pointer to the beginning of the written data
    const uint8_t* temp = (const uint8_t*) FLASH_WRITE_ADDR;

    // Cast what should be the check struct at the end of the data to a check struct
    const eeprom_header_t* saved_header = (const eeprom_header_t*)(FLASH_WRITE_ADDR);
    const uint8_t* saved_payload = (const uint8_t*)(FLASH_WRITE_ADDR + sizeof(eeprom_header_t));
    const eeprom_footer_t* saved_footer = (const eeprom_footer_t*)(FLASH_WRITE_ADDR + sizeof(eeprom_header_t) + header_.size);

    // Check the header
    if (saved_header->version != EEPROM_VERSION)
        return false;
    if (saved_header->size != header_.size)
        return false;
    if (saved_header->magic_be != 0xBE)
        return false;

    // Check the footer
    if(saved_footer->magic_ef != 0xEF)
        return false;

    // Make sure the data is all good (with checksum)
    uint8_t chk = 0;
    for (const uint8_t* p = saved_payload; p < (const uint8_t*)saved_footer; p++)
    {
        chk ^= *p;
    }

    if (chk != saved_footer->crc)
        return false;

    else
        // looks good!  let's roll
        return true;
}
