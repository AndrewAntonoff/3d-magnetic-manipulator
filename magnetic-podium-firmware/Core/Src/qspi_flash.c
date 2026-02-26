#include "qspi_flash.h"
#include "string.h"
#include "stdio.h"
#include "debug_console.h"

extern QSPI_HandleTypeDef hqspi;

// --- Вспомогательные функции для STM32H7 ---
static void QSPI_WriteEnable(void) {
    QSPI_CommandTypeDef cmd = {0};

    cmd.Instruction = W25Q_WRITE_ENABLE;
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.AddressMode = QSPI_ADDRESS_NONE;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode = QSPI_DATA_NONE;
    cmd.DummyCycles = 0;
    cmd.DdrMode = QSPI_DDR_MODE_DISABLE;
    cmd.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_MAX_DELAY) != HAL_OK) {
        return;
    }
}

static uint8_t QSPI_ReadStatus(void) {
    QSPI_CommandTypeDef cmd = {0};
    uint8_t status = 0;

    cmd.Instruction = W25Q_READ_STATUS_REG;
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.AddressMode = QSPI_ADDRESS_NONE;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode = QSPI_DATA_1_LINE;
    cmd.NbData = 1;
    cmd.DummyCycles = 0;
    cmd.DdrMode = QSPI_DDR_MODE_DISABLE;
    cmd.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_MAX_DELAY) != HAL_OK) {
        return 0xFF;
    }

    if (HAL_QSPI_Receive(&hqspi, &status, HAL_MAX_DELAY) != HAL_OK) {
        return 0xFF;
    }

    return status;
}

static void QSPI_WaitForWriteEnd(void) {
    uint8_t status;
    do {
        status = QSPI_ReadStatus();
    } while (status & 0x01);
}

// --- Публичный API ---
bool QSPI_Flash_Init(void) {
    // Проверим JEDEC ID
    uint32_t jedec_id = QSPI_Flash_ReadJEDECID();
    (void)jedec_id; // Подавляем предупреждение о неиспользуемой переменной

    // Для W25Q64JV: Manuf ID = 0xEF, Mem Type = 0x40, Capacity = 0x17
    // Можно раскомментировать проверку
    /*
    if ((jedec_id >> 16) != 0xEF || ((jedec_id >> 8) & 0xFF) != 0x40) {
        return false;
    }
    */
    return true;
}

uint32_t QSPI_Flash_ReadJEDECID(void) {
    QSPI_CommandTypeDef cmd = {0};
    uint8_t id[3] = {0};

    cmd.Instruction = W25Q_JEDEC_ID;
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.AddressMode = QSPI_ADDRESS_NONE;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode = QSPI_DATA_1_LINE;
    cmd.NbData = 3;
    cmd.DummyCycles = 0;
    cmd.DdrMode = QSPI_DDR_MODE_DISABLE;
    cmd.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_MAX_DELAY) != HAL_OK) {
        return 0xFFFFFFFF;
    }

    if (HAL_QSPI_Receive(&hqspi, id, HAL_MAX_DELAY) != HAL_OK) {
        return 0xFFFFFFFF;
    }

    return (id[0] << 16) | (id[1] << 8) | id[2];
}

void QSPI_Flash_EraseSector(uint32_t address) {
    QSPI_CommandTypeDef cmd = {0};

    address &= ~(4096UL - 1);

    QSPI_WriteEnable();

    cmd.Instruction = W25Q_SECTOR_ERASE;
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.AddressMode = QSPI_ADDRESS_1_LINE;
    cmd.AddressSize = QSPI_ADDRESS_24_BITS;
    cmd.Address = address;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode = QSPI_DATA_NONE;
    cmd.DummyCycles = 0;
    cmd.DdrMode = QSPI_DDR_MODE_DISABLE;
    cmd.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

    HAL_QSPI_Command(&hqspi, &cmd, HAL_MAX_DELAY);
    QSPI_WaitForWriteEnd();
}

void QSPI_Flash_WritePage(uint32_t address, const uint8_t *data, uint32_t len) {
    if(len == 0) return;
    if(len > 256) len = 256;

    QSPI_CommandTypeDef cmd = {0};
    QSPI_AutoPollingTypeDef config = {0};

    QSPI_WriteEnable();

    cmd.Instruction = W25Q_PAGE_PROGRAM;
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.AddressMode = QSPI_ADDRESS_1_LINE;
    cmd.AddressSize = QSPI_ADDRESS_24_BITS;
    cmd.Address = address;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode = QSPI_DATA_1_LINE;
    cmd.NbData = len;
    cmd.DummyCycles = 0;
    cmd.DdrMode = QSPI_DDR_MODE_DISABLE;
    cmd.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

    HAL_QSPI_Command(&hqspi, &cmd, HAL_MAX_DELAY);
    HAL_QSPI_Transmit(&hqspi, (uint8_t*)data, HAL_MAX_DELAY);

    config.Match = 0x00;
    config.Mask = 0x01;
    config.MatchMode = QSPI_MATCH_MODE_AND;
    config.Interval = 0x10;
    config.StatusBytesSize = 1;
    config.AutomaticStop = QSPI_AUTOMATIC_STOP_ENABLE;

    cmd.Instruction = W25Q_READ_STATUS_REG;
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.AddressMode = QSPI_ADDRESS_NONE;
    cmd.DataMode = QSPI_DATA_1_LINE;
    cmd.NbData = 1;
    cmd.DummyCycles = 0;
    cmd.DdrMode = QSPI_DDR_MODE_DISABLE;
    cmd.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

    HAL_QSPI_AutoPolling(&hqspi, &cmd, &config, HAL_MAX_DELAY);
}

void QSPI_Flash_WriteBuffer(uint32_t address, const uint8_t *data, uint32_t len) {
    uint32_t page_addr = address;
    uint32_t num_of_page = len / 256;
    uint32_t num_of_single = len % 256;
    uint32_t idx = 0;

    for(; idx < num_of_page; idx++) {
        QSPI_Flash_WritePage(page_addr + (idx * 256), data + (idx * 256), 256);
    }

    if(num_of_single != 0) {
        QSPI_Flash_WritePage(page_addr + (num_of_page * 256), data + (num_of_page * 256), num_of_single);
    }
}

void QSPI_Flash_ReadBuffer(uint32_t address, uint8_t *data, uint32_t len) {
    QSPI_CommandTypeDef cmd = {0};
   // uint32_t tickstart = HAL_GetTick();

    cmd.Instruction = W25Q_READ_DATA;
    cmd.InstructionMode = QSPI_INSTRUCTION_1_LINE;
    cmd.AddressMode = QSPI_ADDRESS_1_LINE;
    cmd.AddressSize = QSPI_ADDRESS_24_BITS;
    cmd.Address = address;
    cmd.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
    cmd.DataMode = QSPI_DATA_1_LINE;
    cmd.NbData = len;
    cmd.DummyCycles = 0;
    cmd.DdrMode = QSPI_DDR_MODE_DISABLE;
    cmd.DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
    cmd.SIOOMode = QSPI_SIOO_INST_EVERY_CMD;

    if (HAL_QSPI_Command(&hqspi, &cmd, 5000) != HAL_OK) {  // Таймаут 1000ms
        Debug_Print(LOG_LEVEL_ERROR, "QSPI command timeout!\r\n");
        return;
    }

    if (HAL_QSPI_Receive(&hqspi, data, 5000) != HAL_OK) {  // Таймаут 1000ms
        Debug_Print(LOG_LEVEL_ERROR, "QSPI receive timeout!\r\n");
        return;
    }
}
