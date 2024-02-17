/*
 * A fairly complete port of the MFRC522 arduino library by Miki Balboa
 * https://github.com/miguelbalboa/rfid/tree/master
 * 
 * SPI implementation based on the MSPware driverlib MSP432 SPI - 3-wire Master Incremented Data example by Timothy Logan
 * 
 * only the uid read, as done in the example, was tested so any more advanced features may not work
 * 
 * assumes that pins are connected as follows
 *  P3.2 -> SDA
 *  P1.5 -> SCK
 *  P1.6 -> MOSI
 *  P1.7 -> MISO
 *
 * Author: Django Wardlow
 * Date: 2/16/24
 */

/* DriverLib Includes */
#include "ti/devices/msp432p4xx/driverlib/driverlib.h"
#include "rfid_library.h"
#include <stdint.h>
#include <stdio.h> //printf(); sprintf();


/* SPI Master Configuration Parameter */
const eUSCI_SPI_MasterConfig spiMasterConfig =
    {
        EUSCI_SPI_CLOCKSOURCE_SMCLK,                             // SMCLK Clock Source
        48000000,                                                // SMCLK = 48MHZ
        4000000,                                                 // SPICLK = 4mhz
        EUSCI_SPI_MSB_FIRST,                                     // MSB First
        EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT, // Phase
        EUSCI_SPI_CLOCKPOLARITY_INACTIVITY_HIGH,                 // High polarity
        EUSCI_SPI_3PIN                                           // 3Wire SPI Mode
};

// Size of the MFRC522 FIFO
uint8_t FIFO_SIZE = 64; // The FIFO is 64 uint8_ts.

void DelayMs(unsigned int nrms)
{
    unsigned int i, j;
    for (j = 0; j < nrms; j++)
        for (i = 0; i < 320; i++)
            ;
}

//configures the eusci peripheral
void configure_EUSCI(){
    
    //set pins for eusci to eusci mode
    GPIO_setAsPeripheralModuleFunctionInputPin(EUSCI_PORT, EUSCI_PINS, GPIO_PRIMARY_MODULE_FUNCTION);

    // configure chip select pin
    GPIO_setAsOutputPin(CHIP_SELECT_PORT, CHIP_SELECT_PIN);
    GPIO_setOutputHighOnPin(CHIP_SELECT_PORT, CHIP_SELECT_PIN);

    /* Configuring SPI in 3wire master mode */
    SPI_initMaster(EUSCI_PERIPHERAL, &spiMasterConfig);

    /* Enable SPI module */
    SPI_enableModule(EUSCI_PERIPHERAL);

    /*disable spi intrupt*/
    Interrupt_disableInterrupt(INT_EUSCIB0);

}

//sends a byte and receives a byte from the rfid module
uint8_t transfer(uint8_t send)
{
    //wait for transmit to finish
    while (!(SPI_getInterruptStatus(EUSCI_PERIPHERAL, EUSCI_SPI_TRANSMIT_INTERRUPT)))
        ;

    // send register
    SPI_transmitData(EUSCI_PERIPHERAL, send);

    //wait for data to be received
    while (!(SPI_getInterruptStatus(EUSCI_PERIPHERAL, EUSCI_SPI_RECEIVE_INTERRUPT)))
        ;

    // read value
    uint8_t value = SPI_receiveData(EUSCI_PERIPHERAL);

    // EUSCI_B_SPI_clearInterruptFlag(EUSCI_PERIPHERAL, EUSCI_SPI_TRANSMIT_INTERRUPT | EUSCI_SPI_RECEIVE_INTERRUPT);

    return value;
}

void PCD_WriteRegister(PCD_Register reg, ///< The register to write to. One of the PCD_Register enums.
                       uint8_t value     ///< The value to write.
)
{

    // SPI.beginTransaction(SPISettings(MFRC522_SPICLOCK, MSBFIRST, SPI_MODE0)); // Set the settings to work with SPI bus
    GPIO_setOutputLowOnPin(CHIP_SELECT_PORT, CHIP_SELECT_PIN);
    // digitalWrite(_chipSelectPin, LOW);                                        // Select slave
    transfer(reg); // MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
    transfer(value);
    // digitalWrite(_chipSelectPin, HIGH); // Release slave again
    GPIO_setOutputHighOnPin(CHIP_SELECT_PORT, CHIP_SELECT_PIN);

    // SPI.endTransaction();               // Stop using the SPI bus
} // End PCD_WriteRegister()

void PCD_WriteRegisters(PCD_Register reg, ///< The register to write to. One of the PCD_Register enums.
                        uint8_t count,    ///< The number of uint8_ts to write to the register
                        uint8_t *values   ///< The values to write. uint8_t array.
)
{

    // SPI.beginTransaction(SPISettings(MFRC522_SPICLOCK, MSBFIRST, SPI_MODE0)); // Set the settings to work with SPI bus
    // digitalWrite(_chipSelectPin, LOW);
    GPIO_setOutputLowOnPin(CHIP_SELECT_PORT, CHIP_SELECT_PIN); // Select slave
    transfer(reg);

    uint8_t i; // MSB == 0 is for writing. LSB is not used in address. Datasheet section 8.1.2.3.
    for (i = 0; i < count; i++)
    {
        transfer(values[i]);
    }
    // digitalWrite(_chipSelectPin, HIGH); // Release slave again
    GPIO_setOutputHighOnPin(CHIP_SELECT_PORT, CHIP_SELECT_PIN);
    // SPI.endTransaction();               // Stop using the SPI bus
} // End PCD_Wri

uint8_t PCD_ReadRegister(PCD_Register reg ///< The register to read from. One of the PCD_Register enums.
)
{
    uint8_t value;
    // SPI.beginTransaction(SPISettings(MFRC522_SPICLOCK, MSBFIRST, SPI_MODE0)); // Set the settings to work with SPI bus
    // digitalWrite(_chipSelectPin, LOW);
    GPIO_setOutputLowOnPin(CHIP_SELECT_PORT, CHIP_SELECT_PIN); // Select slave
    transfer(0x80 | reg);                            // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
    value = transfer(0);                             // Read the value back. Send 0 to stop reading.
    // digitalWrite(_chipSelectPin, HIGH);
    GPIO_setOutputHighOnPin(CHIP_SELECT_PORT, CHIP_SELECT_PIN); // Release slave again
    // SPI.endTransaction();                                                     // Stop using the SPI bus
    return value;

} // End PCD_ReadRegister()

/**
 * Reads a number of uint8_ts from the specified register in the MFRC522 chip.
 * The interface is described in the datasheet section 8.1.2.
 */
void PCD_ReadRegisters(PCD_Register reg, ///< The register to read from. One of the PCD_Register enums.
                       uint8_t count,    ///< The number of uint8_ts to read
                       uint8_t *values,  ///< uint8_t array to store the values in.
                       uint8_t rxAlign   ///< Only bit positions rxAlign..7 in values[0] are updated.
)
{
    if (count == 0)
    {
        return;
    }
    // printf(F("Reading ")); 	printf(count); printf(F(" uint8_ts from register."));
    uint8_t address = 0x80 | reg; // MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
    uint8_t index = 0;
    GPIO_setOutputLowOnPin(CHIP_SELECT_PORT, CHIP_SELECT_PIN); // Index in values array.
    count--;                                         // One read is performed outside of the loop
    transfer(address);                               // Tell MFRC522 which address we want to read
    if (rxAlign)
    { // Only update bit positions rxAlign..7 in values[0]
        // Create bit mask for bit positions rxAlign..7
        uint8_t mask = (0xFF << rxAlign) & 0xFF;
        // Read value and tell that we want to read the same address again.
        uint8_t value = transfer(address);
        // Apply mask to both current value of values[0] and the new data in value.
        values[0] = (values[0] & ~mask) | (value & mask);
        index++;
    }
    while (index < count)
    {
        values[index] = transfer(address); // Read value and tell that we want to read the same address again.
        index++;
    }
    values[index] = transfer(0); // Read the final uint8_t. Send 0 to stop reading.
    GPIO_setOutputHighOnPin(CHIP_SELECT_PORT, CHIP_SELECT_PIN);
} // End PCD_ReadRegister()

/**
 * Sets the bits given in mask in register reg.
 */
void PCD_SetRegisterBitMask(PCD_Register reg, ///< The register to update. One of the PCD_Register enums.
                            uint8_t mask      ///< The bits to set.
)
{
    uint8_t tmp;
    tmp = PCD_ReadRegister(reg);
    PCD_WriteRegister(reg, tmp | mask); // set bit mask
} // End PCD_SetRegisterBitMask()

/**
 * Clears the bits given in mask from register reg.
 */
void PCD_ClearRegisterBitMask(PCD_Register reg, ///< The register to update. One of the PCD_Register enums.
                              uint8_t mask      ///< The bits to clear.
)
{
    uint8_t tmp;
    tmp = PCD_ReadRegister(reg);
    PCD_WriteRegister(reg, tmp & (~mask)); // clear bit mask
} // End PCD_ClearRegisterBitMask()

/**
 * Use the CRC coprocessor in the MFRC522 to calculate a CRC_A.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PCD_CalculateCRC(uint8_t *data,  ///< In: Pointer to the data to transfer to the FIFO for CRC calculation.
                            uint8_t length, ///< In: The number of uint8_ts to transfer.
                            uint8_t *result ///< Out: Pointer to result buffer. Result is written to result[0..1], low uint8_t first.
)
{
    PCD_WriteRegister(CommandReg, PCD_Idle);       // Stop any active command.
    PCD_WriteRegister(DivIrqReg, 0x04);            // Clear the CRCIRq interrupt request bit
    PCD_WriteRegister(FIFOLevelReg, 0x80);         // FlushBuffer = 1, FIFO initialization
    PCD_WriteRegisters(FIFODataReg, length, data); // Write data to the FIFO
    PCD_WriteRegister(CommandReg, PCD_CalcCRC);    // Start the calculation

    // Wait for the CRC calculation to complete. Check for the register to
    // indicate that the CRC calculation is complete in a loop. If the
    // calculation is not indicated as complete in ~90ms, then time out
    // the operation.
    // const uint32_t deadline = millis() + 89;
    int checks = 6000;
    do
    {
        // DivIrqReg[7..0] bits are: Set2 reserved reserved MfinActIRq reserved CRCIRq reserved reserved
        uint8_t n = PCD_ReadRegister(DivIrqReg);
        if (n & 0x04)
        {                                            // CRCIRq bit set - calculation done
            PCD_WriteRegister(CommandReg, PCD_Idle); // Stop calculating CRC for new content in the FIFO.
            // Transfer the result from the registers to the result buffer
            result[0] = PCD_ReadRegister(CRCResultRegL);
            result[1] = PCD_ReadRegister(CRCResultRegH);
            return STATUS_OK;
        }
        checks--;
    } while (checks > 0);

    // 89ms passed and nothing happened. Communication with the MFRC522 might be down.
    return STATUS_TIMEOUT;
} // End PCD_CalculateCRC()

/////////////////////////////////////////////////////////////////////////////////////
// Functions for manipulating the MFRC522
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Initializes the MFRC522 chip.
 */
void PCD_Init()
{
    //configures the EUSCI peripheral on the msp432
    configure_EUSCI();

    PCD_Reset();

    // Reset baud rates
    PCD_WriteRegister(TxModeReg, 0x00);
    PCD_WriteRegister(RxModeReg, 0x00);
    // Reset ModWidthReg
    PCD_WriteRegister(ModWidthReg, 0x26);

    // When communicating with a PICC we need a timeout if something goes wrong.
    // f_timer = 13.56 MHz / (2*TPreScaler+1) where TPreScaler = [TPrescaler_Hi:TPrescaler_Lo].
    // TPrescaler_Hi are the four low bits in TModeReg. TPrescaler_Lo is TPrescalerReg.
    PCD_WriteRegister(TModeReg, 0x80);      // TAuto=1; timer starts automatically at the end of the transmission in all communication modes at all speeds
    PCD_WriteRegister(TPrescalerReg, 0xA9); // TPreScaler = TModeReg[3..0]:TPrescalerReg, ie 0x0A9 = 169 => f_timer=40kHz, ie a timer period of 25μs.
    PCD_WriteRegister(TReloadRegH, 0x03);   // Reload timer with 0x3E8 = 1000, ie 25ms before timeout.
    PCD_WriteRegister(TReloadRegL, 0xE8);

    PCD_WriteRegister(TxASKReg, 0x40); // Default 0x00. Force a 100 % ASK modulation independent of the ModGsPReg register setting
    PCD_WriteRegister(ModeReg, 0x3D);  // Default 0x3F. Set the preset value for the CRC coprocessor for the CalcCRC command to 0x6363 (ISO 14443-3 part 6.2.4)
    PCD_AntennaOn();                   // Enable the antenna driver pins TX1 and TX2 (they were disabled by the reset)
} // End PCD_Init()

/**
 * Performs a soft reset on the MFRC522 chip and waits for it to be ready again.
 */
void PCD_Reset()
{
    PCD_WriteRegister(CommandReg, PCD_SoftReset); // Issue the SoftReset command.
    // The datasheet does not mention how long the SoftRest command takes to complete.
    // But the MFRC522 might have been in soft power-down mode (triggered by bit 4 of CommandReg)
    // Section 8.8.2 in the datasheet says the oscillator start-up time is the start up time of the crystal + 37,74μs. Let us be generous: 50ms.
    uint8_t count = 0;
    do
    {
        // Wait for the PowerDown bit in CommandReg to be cleared (max 3x50ms)
        DelayMs(50);
    } while ((PCD_ReadRegister(CommandReg) & (1 << 4)) && (++count) < 3);
} // End PCD_Reset()

/**
 * Turns the antenna on by enabling pins TX1 and TX2.
 * After a reset these pins are disabled.
 */
void PCD_AntennaOn()
{
    uint8_t value = PCD_ReadRegister(TxControlReg);
    if ((value & 0x03) != 0x03)
    {
        PCD_WriteRegister(TxControlReg, value | 0x03);
    }
} // End PCD_AntennaOn()

/**
 * Turns the antenna off by disabling pins TX1 and TX2.
 */
void PCD_AntennaOff()
{
    PCD_ClearRegisterBitMask(TxControlReg, 0x03);
} // End PCD_AntennaOff()

/**
 * Get the current MFRC522 Receiver Gain (RxGain[2:0]) value.
 * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * NOTE: Return value scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
 *
 * @return Value of the RxGain, scrubbed to the 3 bits used.
 */
uint8_t PCD_GetAntennaGain()
{
    return PCD_ReadRegister(RFCfgReg) & (0x07 << 4);
} // End PCD_GetAntennaGain()

/**
 * Set the MFRC522 Receiver Gain (RxGain) to value specified by given mask.
 * See 9.3.3.6 / table 98 in http://www.nxp.com/documents/data_sheet/MFRC522.pdf
 * NOTE: Given mask is scrubbed with (0x07<<4)=01110000b as RCFfgReg may use reserved bits.
 */
void PCD_SetAntennaGain(uint8_t mask)
{
    if (PCD_GetAntennaGain() != mask)
    {                                                         // only bother if there is a change
        PCD_ClearRegisterBitMask(RFCfgReg, (0x07 << 4));      // clear needed to allow 000 pattern
        PCD_SetRegisterBitMask(RFCfgReg, mask & (0x07 << 4)); // only set RxGain[2:0] bits
    }
} // End PCD_SetAntennaGain()

/////////////////////////////////////////////////////////////////////////////////////
// Power control
/////////////////////////////////////////////////////////////////////////////////////

// IMPORTANT NOTE!!!!
// Calling any other function that uses CommandReg will disable soft power down mode !!!
// For more details about power control, refer to the datasheet - page 33 (8.6)

void PCD_SoftPowerDown()
{                                               // Note : Only soft power down mode is available throught software
    uint8_t val = PCD_ReadRegister(CommandReg); // Read state of the command register
    val |= (1 << 4);                            // set PowerDown bit ( bit 4 ) to 1
    PCD_WriteRegister(CommandReg, val);         // write new value to the command register
}

void PCD_SoftPowerUp()
{
    uint8_t val = PCD_ReadRegister(CommandReg); // Read state of the command register
    val &= ~(1 << 4);                           // set PowerDown bit ( bit 4 ) to 0
    PCD_WriteRegister(CommandReg, val);         // write new value to the command register
    // wait until PowerDown bit is cleared (this indicates end of wake up procedure)
    uint32_t trys = 32000; // create timer for timeout (just in case)

    while (trys > 0)
    {
        trys--;                             // set timeout to 500 ms
        val = PCD_ReadRegister(CommandReg); // Read state of the command register
        if (!(val & (1 << 4)))
        {          // if powerdown bit is 0
            break; // wake up procedure is finished
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with PICCs
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Executes the Transceive command.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PCD_TransceiveData(uint8_t *sendData,  ///< Pointer to the data to transfer to the FIFO.
                              uint8_t sendLen,    ///< Number of uint8_ts to transfer to the FIFO.
                              uint8_t *backData,  ///< nullptr or pointer to buffer if data should be read back after executing the command.
                              uint8_t *backLen,   ///< In: Max number of uint8_ts to write to *backData. Out: The number of uint8_ts returned.
                              uint8_t *validBits, ///< In/Out: The number of valid bits in the last uint8_t. 0 for 8 valid bits. Default nullptr.
                              uint8_t rxAlign,    ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
                              int checkCRC        ///< In: True => The last two uint8_ts of the response is assumed to be a CRC_A that must be validated.
)
{
    uint8_t waitIRq = 0x30; // RxIRq and IdleIRq
    return PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, sendData, sendLen, backData, backLen, validBits, rxAlign, checkCRC);
} // End PCD_TransceiveData()

/**
 * Transfers data to the MFRC522 FIFO, executes a command, waits for completion and transfers data back from the FIFO.
 * CRC validation can only be done if backData and backLen are specified.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PCD_CommunicateWithPICC(uint8_t command,    ///< The command to execute. One of the PCD_Command enums.
                                   uint8_t waitIRq,    ///< The bits in the ComIrqReg register that signals successful completion of the command.
                                   uint8_t *sendData,  ///< Pointer to the data to transfer to the FIFO.
                                   uint8_t sendLen,    ///< Number of uint8_ts to transfer to the FIFO.
                                   uint8_t *backData,  ///< nullptr or pointer to buffer if data should be read back after executing the command.
                                   uint8_t *backLen,   ///< In: Max number of uint8_ts to write to *backData. Out: The number of uint8_ts returned.
                                   uint8_t *validBits, ///< In/Out: The number of valid bits in the last uint8_t. 0 for 8 valid bits.
                                   uint8_t rxAlign,    ///< In: Defines the bit position in backData[0] for the first bit received. Default 0.
                                   int checkCRC        ///< In: True => The last two uint8_ts of the response is assumed to be a CRC_A that must be validated.
)
{
    // Prepare values for BitFramingReg
    uint8_t txLastBits = validBits ? *validBits : 0;
    uint8_t bitFraming = (rxAlign << 4) + txLastBits; // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

    PCD_WriteRegister(CommandReg, PCD_Idle);            // Stop any active command.
    PCD_WriteRegister(ComIrqReg, 0x7F);                 // Clear all seven interrupt request bits
    PCD_WriteRegister(FIFOLevelReg, 0x80);              // FlushBuffer = 1, FIFO initialization
    PCD_WriteRegisters(FIFODataReg, sendLen, sendData); // Write sendData to the FIFO
    PCD_WriteRegister(BitFramingReg, bitFraming);       // Bit adjustments
    PCD_WriteRegister(CommandReg, command);             // Execute the command
    if (command == PCD_Transceive)
    {
        PCD_SetRegisterBitMask(BitFramingReg, 0x80); // StartSend=1, transmission of data starts
    }

    // In PCD_Init() we set the TAuto flag in TModeReg. This means the timer
    // automatically starts when the PCD stops transmitting.
    //
    // Wait here for the command to complete. The bits specified in the
    // `waitIRq` parameter define what bits constitute a completed command.
    // When they are set in the ComIrqReg register, then the command is
    // considered complete. If the command is not indicated as complete in
    // ~36ms, then consider the command as timed out.
    int checks = 2500;
    int completed = false;

    do
    {
        uint8_t n = PCD_ReadRegister(ComIrqReg); // ComIrqReg[7..0] bits are: Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
        if (n & waitIRq)
        { // One of the interrupts that signal success has been set.
            completed = true;
            break;
        }
        if (n & 0x01)
        { // Timer interrupt - nothing received in 25ms
            return STATUS_TIMEOUT;
        }
        checks--;
    } while (checks > 0);

    // 36ms and nothing happened. Communication with the MFRC522 might be down.
    if (!completed)
    {
        return STATUS_TIMEOUT;
    }

    // Stop now if any errors except collisions were detected.
    uint8_t errorRegValue = PCD_ReadRegister(ErrorReg); // ErrorReg[7..0] bits are: WrErr TempErr reserved BufferOvfl CollErr CRCErr ParityErr ProtocolErr
    if (errorRegValue & 0x13)
    { // BufferOvfl ParityErr ProtocolErr
        return STATUS_ERROR;
    }

    uint8_t _validBits = 0;

    // If the caller wants data back, get it from the MFRC522.
    if (backData && backLen)
    {
        uint8_t n = PCD_ReadRegister(FIFOLevelReg); // Number of uint8_ts in the FIFO
        if (n > *backLen)
        {
            return STATUS_NO_ROOM;
        }
        *backLen = n;                                         // Number of uint8_ts returned
        PCD_ReadRegisters(FIFODataReg, n, backData, rxAlign); // Get received data from FIFO
        _validBits = PCD_ReadRegister(ControlReg) & 0x07;     // RxLastBits[2:0] indicates the number of valid bits in the last received uint8_t. If this value is 000b, the whole uint8_t is valid.
        if (validBits)
        {
            *validBits = _validBits;
        }
    }

    // Tell about collisions
    if (errorRegValue & 0x08)
    { // CollErr
        return STATUS_COLLISION;
    }

    // Perform CRC_A validation if requested.
    if (backData && backLen && checkCRC)
    {
        // In this case a MIFARE Classic NAK is not OK.
        if (*backLen == 1 && _validBits == 4)
        {
            return STATUS_MIFARE_NACK;
        }
        // We need at least the CRC_A value and all 8 bits of the last uint8_t must be received.
        if (*backLen < 2 || _validBits != 0)
        {
            return STATUS_CRC_WRONG;
        }
        // Verify CRC_A - do our own calculation and store the control in controlBuffer.
        uint8_t controlBuffer[2];
        StatusCode status = PCD_CalculateCRC(&backData[0], *backLen - 2, &controlBuffer[0]);
        if (status != STATUS_OK)
        {
            return status;
        }
        if ((backData[*backLen - 2] != controlBuffer[0]) || (backData[*backLen - 1] != controlBuffer[1]))
        {
            return STATUS_CRC_WRONG;
        }
    }

    return STATUS_OK;
} // End PCD_CommunicateWithPICC()

/**
 * Transmits a REQuest command, Type A. Invites PICCs in state IDLE to go to READY and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PICC_RequestA(uint8_t *bufferATQA, ///< The buffer to store the ATQA (Answer to request) in
                         uint8_t *bufferSize  ///< Buffer size, at least two uint8_ts. Also number of uint8_ts returned if STATUS_OK.
)
{
    return PICC_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
} // End PICC_RequestA()

/**
 * Transmits a Wake-UP command, Type A. Invites PICCs in state IDLE and HALT to go to READY(*) and prepare for anticollision or selection. 7 bit frame.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PICC_WakeupA(uint8_t *bufferATQA, ///< The buffer to store the ATQA (Answer to request) in
                        uint8_t *bufferSize  ///< Buffer size, at least two uint8_ts. Also number of uint8_ts returned if STATUS_OK.
)
{
    return PICC_REQA_or_WUPA(PICC_CMD_WUPA, bufferATQA, bufferSize);
} // End PICC_WakeupA()

/**
 * Transmits REQA or WUPA commands.
 * Beware: When two PICCs are in the field at the same time I often get STATUS_TIMEOUT - probably due do bad antenna design.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PICC_REQA_or_WUPA(uint8_t command,     ///< The command to send - PICC_CMD_REQA or PICC_CMD_WUPA
                             uint8_t *bufferATQA, ///< The buffer to store the ATQA (Answer to request) in
                             uint8_t *bufferSize  ///< Buffer size, at least two uint8_ts. Also number of uint8_ts returned if STATUS_OK.
)
{
    uint8_t validBits;
    StatusCode status;

    if (bufferATQA == NULL || *bufferSize < 2)
    { // The ATQA response is 2 uint8_ts long.
        return STATUS_NO_ROOM;
    }
    PCD_ClearRegisterBitMask(CollReg, 0x80); // ValuesAfterColl=1 => Bits received after collision are cleared.
    validBits = 7;                           // For REQA and WUPA we need the short frame format - transmit only 7 bits of the last (and only) uint8_t. TxLastBits = BitFramingReg[2..0]
    status = PCD_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits, 0, 0);
    if (status != STATUS_OK)
    {
        return status;
    }
    if (*bufferSize != 2 || validBits != 0)
    { // ATQA must be exactly 16 bits.
        return STATUS_ERROR;
    }
    return STATUS_OK;
} // End PICC_REQA_or_WUPA()

/**
 * Transmits SELECT/ANTICOLLISION commands to select a single PICC.
 * Before calling this function the PICCs must be placed in the READY(*) state by calling PICC_RequestA() or PICC_WakeupA().
 * On success:
 * 		- The chosen PICC is in state ACTIVE(*) and all other PICCs have returned to state IDLE/HALT. (Figure 7 of the ISO/IEC 14443-3 draft.)
 * 		- The UID size and value of the chosen PICC is returned in *uid along with the SAK.
 *
 * A PICC UID consists of 4, 7 or 10 uint8_ts.
 * Only 4 uint8_ts can be specified in a SELECT command, so for the longer UIDs two or three iterations are used:
 * 		UID size	Number of UID uint8_ts		Cascade levels		Example of PICC
 * 		========	===================		==============		===============
 * 		single				 4						1				MIFARE Classic
 * 		double				 7						2				MIFARE Ultralight
 * 		triple				10						3				Not currently in use?
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PICC_Select(Uid *uid,         ///< Pointer to Uid struct. Normally output, but can also be used to supply a known UID.
                       uint8_t validBits ///< The number of known UID bits supplied in *uid. Normally 0. If set you must also supply uid->size.
)
{
    int uidComplete;
    int selectDone;
    int useCascadeTag;
    uint8_t cascadeLevel = 1;
    StatusCode result;
    uint8_t count;
    uint8_t checkBit;
    uint8_t index;
    uint8_t uidIndex;             // The first index in uid->uiduint8_t[] that is used in the current Cascade Level.
    int8_t currentLevelKnownBits; // The number of known UID bits in the current Cascade Level.
    uint8_t buffer[9];            // The SELECT/ANTICOLLISION commands uses a 7 uint8_t standard frame + 2 uint8_ts CRC_A
    uint8_t bufferUsed;           // The number of uint8_ts used in the buffer, ie the number of uint8_ts to transfer to the FIFO.
    uint8_t rxAlign;              // Used in BitFramingReg. Defines the bit position for the first bit received.
    uint8_t txLastBits;           // Used in BitFramingReg. The number of valid bits in the last transmitted uint8_t.
    uint8_t *responseBuffer;
    uint8_t responseLength;

    // Description of buffer structure:
    //		uint8_t 0: SEL 				Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
    //		uint8_t 1: NVB					Number of Valid Bits (in complete command, not just the UID): High nibble: complete uint8_ts, Low nibble: Extra bits.
    //		uint8_t 2: UID-data or CT		See explanation below. CT means Cascade Tag.
    //		uint8_t 3: UID-data
    //		uint8_t 4: UID-data
    //		uint8_t 5: UID-data
    //		uint8_t 6: BCC					Block Check Character - XOR of uint8_ts 2-5
    //		uint8_t 7: CRC_A
    //		uint8_t 8: CRC_A
    // The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
    //
    // Description of uint8_ts 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
    //		UID size	Cascade level	uint8_t2	uint8_t3	uint8_t4	uint8_t5
    //		========	=============	=====	=====	=====	=====
    //		 4 uint8_ts		1			uid0	uid1	uid2	uid3
    //		 7 uint8_ts		1			CT		uid0	uid1	uid2
    //						2			uid3	uid4	uid5	uid6
    //		10 uint8_ts		1			CT		uid0	uid1	uid2
    //						2			CT		uid3	uid4	uid5
    //						3			uid6	uid7	uid8	uid9

    // Sanity checks
    if (validBits > 80)
    {
        return STATUS_INVALID;
    }

    // Prepare MFRC522
    PCD_ClearRegisterBitMask(CollReg, 0x80); // ValuesAfterColl=1 => Bits received after collision are cleared.

    // Repeat Cascade Level loop until we have a complete UID.
    uidComplete = false;
    while (!uidComplete)
    {
        // Set the Cascade Level in the SEL uint8_t, find out if we need to use the Cascade Tag in uint8_t 2.
        switch (cascadeLevel)
        {
        case 1:
            buffer[0] = PICC_CMD_SEL_CL1;
            uidIndex = 0;
            useCascadeTag = validBits && uid->size > 4; // When we know that the UID has more than 4 uint8_ts
            break;

        case 2:
            buffer[0] = PICC_CMD_SEL_CL2;
            uidIndex = 3;
            useCascadeTag = validBits && uid->size > 7; // When we know that the UID has more than 7 uint8_ts
            break;

        case 3:
            buffer[0] = PICC_CMD_SEL_CL3;
            uidIndex = 6;
            useCascadeTag = false; // Never used in CL3.
            break;

        default:
            return STATUS_INTERNAL_ERROR;
            break;
        }

        // How many UID bits are known in this Cascade Level?
        currentLevelKnownBits = validBits - (8 * uidIndex);
        if (currentLevelKnownBits < 0)
        {
            currentLevelKnownBits = 0;
        }
        // Copy the known bits from uid->uiduint8_t[] to buffer[]
        index = 2; // destination index in buffer[]
        if (useCascadeTag)
        {
            buffer[index++] = PICC_CMD_CT;
        }
        uint8_t uint8_tsToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of uint8_ts needed to represent the known bits for this level.
        if (uint8_tsToCopy)
        {
            uint8_t maxuint8_ts = useCascadeTag ? 3 : 4; // Max 4 uint8_ts in each Cascade Level. Only 3 left if we use the Cascade Tag
            if (uint8_tsToCopy > maxuint8_ts)
            {
                uint8_tsToCopy = maxuint8_ts;
            }
            for (count = 0; count < uint8_tsToCopy; count++)
            {
                buffer[index++] = uid->uiduint8_t[uidIndex + count];
            }
        }
        // Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
        if (useCascadeTag)
        {
            currentLevelKnownBits += 8;
        }

        // Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
        selectDone = false;
        while (!selectDone)
        {
            // Find out how many bits and uint8_ts to send and receive.
            if (currentLevelKnownBits >= 32)
            { // All UID bits in this Cascade Level are known. This is a SELECT.
                // printf(F("SELECT: currentLevelKnownBits=")); printf(currentLevelKnownBits, DEC);
                buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole uint8_ts
                // Calculate BCC - Block Check Character
                buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
                // Calculate CRC_A
                result = PCD_CalculateCRC(buffer, 7, &buffer[7]);
                if (result != STATUS_OK)
                {
                    return result;
                }
                txLastBits = 0; // 0 => All 8 bits are valid.
                bufferUsed = 9;
                // Store response in the last 3 uint8_ts of buffer (BCC and CRC_A - not needed after tx)
                responseBuffer = &buffer[6];
                responseLength = 3;
            }
            else
            { // This is an ANTICOLLISION.
                // printf(F("ANTICOLLISION: currentLevelKnownBits=")); printf(currentLevelKnownBits, DEC);
                txLastBits = currentLevelKnownBits % 8;
                count = currentLevelKnownBits / 8;     // Number of whole uint8_ts in the UID part.
                index = 2 + count;                     // Number of whole uint8_ts: SEL + NVB + UIDs
                buffer[1] = (index << 4) + txLastBits; // NVB - Number of Valid Bits
                bufferUsed = index + (txLastBits ? 1 : 0);
                // Store response in the unused part of buffer
                responseBuffer = &buffer[index];
                responseLength = sizeof(buffer) - index;
            }

            // Set bit adjustments
            rxAlign = txLastBits;                                          // Having a separate variable is overkill. But it makes the next line easier to read.
            PCD_WriteRegister(BitFramingReg, (rxAlign << 4) + txLastBits); // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

            // Transmit the buffer and receive the response.
            result = PCD_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits, rxAlign, 0);
            if (result == STATUS_COLLISION)
            {                                                       // More than one PICC in the field => collision.
                uint8_t valueOfCollReg = PCD_ReadRegister(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
                if (valueOfCollReg & 0x20)
                {                            // CollPosNotValid
                    return STATUS_COLLISION; // Without a valid collision position we cannot continue
                }
                uint8_t collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
                if (collisionPos == 0)
                {
                    collisionPos = 32;
                }
                if (collisionPos <= currentLevelKnownBits)
                { // No progress - should not happen
                    return STATUS_INTERNAL_ERROR;
                }
                // Choose the PICC with the bit set.
                currentLevelKnownBits = collisionPos;
                count = currentLevelKnownBits % 8; // The bit to modify
                checkBit = (currentLevelKnownBits - 1) % 8;
                index = 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First uint8_t is index 0.
                buffer[index] |= (1 << checkBit);
            }
            else if (result != STATUS_OK)
            {
                return result;
            }
            else
            { // STATUS_OK
                if (currentLevelKnownBits >= 32)
                {                      // This was a SELECT.
                    selectDone = true; // No more anticollision
                                       // We continue below outside the while.
                }
                else
                { // This was an ANTICOLLISION.
                    // We now have all 32 bits of the UID in this Cascade Level
                    currentLevelKnownBits = 32;
                    // Run loop again to do the SELECT.
                }
            }
        } // End of while (!selectDone)

        // We do not check the CBB - it was constructed by us above.

        // Copy the found UID uint8_ts from buffer[] to uid->uiduint8_t[]
        index = (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
        uint8_tsToCopy = (buffer[2] == PICC_CMD_CT) ? 3 : 4;
        for (count = 0; count < uint8_tsToCopy; count++)
        {
            uid->uiduint8_t[uidIndex + count] = buffer[index++];
        }

        // Check response SAK (Select Acknowledge)
        if (responseLength != 3 || txLastBits != 0)
        { // SAK must be exactly 24 bits (1 uint8_t + CRC_A).
            return STATUS_ERROR;
        }
        // Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those uint8_ts are not needed anymore.
        result = PCD_CalculateCRC(responseBuffer, 1, &buffer[2]);
        if (result != STATUS_OK)
        {
            return result;
        }
        if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2]))
        {
            return STATUS_CRC_WRONG;
        }
        if (responseBuffer[0] & 0x04)
        { // Cascade bit set - UID not complete yes
            cascadeLevel++;
        }
        else
        {
            uidComplete = true;
            uid->sak = responseBuffer[0];
        }
    } // End of while (!uidComplete)

    // Set correct uid->size
    uid->size = 3 * cascadeLevel + 1;

    return STATUS_OK;
} // End PICC_Select()

/**
 * Instructs a PICC in state ACTIVE(*) to go to state HALT.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PICC_HaltA()
{
    StatusCode result;
    uint8_t buffer[4];

    // Build command buffer
    buffer[0] = PICC_CMD_HLTA;
    buffer[1] = 0;
    // Calculate CRC_A
    result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
    if (result != STATUS_OK)
    {
        return result;
    }

    // Send the command.
    // The standard says:
    //		If the PICC responds with any modulation during a period of 1 ms after the end of the frame containing the
    //		HLTA command, this response shall be interpreted as 'not acknowledge'.
    // We interpret that this way: Only STATUS_TIMEOUT is a success.
    result = PCD_TransceiveData(buffer, sizeof(buffer), NULL, 0, 0, 0, 0);
    if (result == STATUS_TIMEOUT)
    {
        return STATUS_OK;
    }
    if (result == STATUS_OK)
    { // That is ironically NOT ok in this case ;-)
        return STATUS_ERROR;
    }
    return result;
} // End PICC_HaltA()

/////////////////////////////////////////////////////////////////////////////////////
// Functions for communicating with MIFARE PICCs
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Executes the MFRC522 MFAuthent command.
 * This command manages MIFARE authentication to enable a secure communication to any MIFARE Mini, MIFARE 1K and MIFARE 4K card.
 * The authentication is described in the MFRC522 datasheet section 10.3.1.9 and http://www.nxp.com/documents/data_sheet/MF1S503x.pdf section 10.1.
 * For use with MIFARE Classic PICCs.
 * The PICC must be selected - ie in state ACTIVE(*) - before calling this function.
 * Remember to call PCD_StopCrypto1() after communicating with the authenticated PICC - otherwise no new communications can start.
 *
 * All keys are set to FFFFFFFFFFFFh at chip delivery.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise. Probably STATUS_TIMEOUT if you supply the wrong key.
 */
StatusCode PCD_Authenticate(uint8_t command,   ///< PICC_CMD_MF_AUTH_KEY_A or PICC_CMD_MF_AUTH_KEY_B
                            uint8_t blockAddr, ///< The block number. See numbering in the comments in the .h file.
                            MIFARE_Key *key,   ///< Pointer to the Crypteo1 key to use (6 uint8_ts)
                            Uid *uid           ///< Pointer to Uid struct. The first 4 uint8_ts of the UID is used.
)
{
    uint8_t waitIRq = 0x10; // IdleIRq

    // Build command buffer
    uint8_t sendData[12];
    sendData[0] = command;
    sendData[1] = blockAddr;

    uint8_t i;
    for (i = 0; i < MF_KEY_SIZE; i++)
    { // 6 key uint8_ts
        sendData[2 + i] = key->keyuint8_t[i];
    }
    // Use the last uid uint8_ts as specified in http://cache.nxp.com/documents/application_note/AN10927.pdf
    // section 3.2.5 "MIFARE Classic Authentication".
    // The only missed case is the MF1Sxxxx shortcut activation,
    // but it requires cascade tag (CT) uint8_t, that is not part of uid.
    for (i = 0; i < 4; i++)
    { // The last 4 uint8_ts of the UID
        sendData[8 + i] = uid->uiduint8_t[i + uid->size - 4];
    }

    // Start the authentication.
    return PCD_CommunicateWithPICC(PCD_MFAuthent, waitIRq, &sendData[0], sizeof(sendData), 0, 0, 0, 0, 0);
} // End PCD_Authenticate()

/**
 * Used to exit the PCD from its authenticated state.
 * Remember to call this function after communicating with an authenticated PICC - otherwise no new communications can start.
 */
void PCD_StopCrypto1()
{
    // Clear MFCrypto1On bit
    PCD_ClearRegisterBitMask(Status2Reg, 0x08); // Status2Reg[7..0] bits are: TempSensClear I2CForceHS reserved reserved MFCrypto1On ModemState[2:0]
} // End PCD_StopCrypto1()

/**
 * Reads 16 uint8_ts (+ 2 uint8_ts CRC_A) from the active PICC.
 *
 * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
 *
 * For MIFARE Ultralight only addresses 00h to 0Fh are decoded.
 * The MF0ICU1 returns a NAK for higher addresses.
 * The MF0ICU1 responds to the READ command by sending 16 uint8_ts starting from the page address defined by the command argument.
 * For example; if blockAddr is 03h then pages 03h, 04h, 05h, 06h are returned.
 * A roll-back is implemented: If blockAddr is 0Eh, then the contents of pages 0Eh, 0Fh, 00h and 01h are returned.
 *
 * The buffer must be at least 18 uint8_ts because a CRC_A is also returned.
 * Checks the CRC_A before returning STATUS_OK.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode MIFARE_Read(uint8_t blockAddr,  ///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The first page to return data from.
                       uint8_t *buffer,    ///< The buffer to store the data in
                       uint8_t *bufferSize ///< Buffer size, at least 18 uint8_ts. Also number of uint8_ts returned if STATUS_OK.
)
{
    StatusCode result;

    // Sanity check
    if (buffer == NULL || *bufferSize < 18)
    {
        return STATUS_NO_ROOM;
    }

    // Build command buffer
    buffer[0] = PICC_CMD_MF_READ;
    buffer[1] = blockAddr;
    // Calculate CRC_A
    result = PCD_CalculateCRC(buffer, 2, &buffer[2]);
    if (result != STATUS_OK)
    {
        return result;
    }

    // Transmit the buffer and receive the response, validate CRC_A.
    return PCD_TransceiveData(buffer, 4, buffer, bufferSize, NULL, 0, true);
} // End MIFARE_Read()

/**
 * Writes 16 uint8_ts to the active PICC.
 *
 * For MIFARE Classic the sector containing the block must be authenticated before calling this function.
 *
 * For MIFARE Ultralight the operation is called "COMPATIBILITY WRITE".
 * Even though 16 uint8_ts are transferred to the Ultralight PICC, only the least significant 4 uint8_ts (uint8_ts 0 to 3)
 * are written to the specified address. It is recommended to set the remaining uint8_ts 04h to 0Fh to all logic 0.
 * *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode MIFARE_Write(uint8_t blockAddr, ///< MIFARE Classic: The block (0-0xff) number. MIFARE Ultralight: The page (2-15) to write to.
                        uint8_t *buffer,   ///< The 16 uint8_ts to write to the PICC
                        uint8_t bufferSize ///< Buffer size, must be at least 16 uint8_ts. Exactly 16 uint8_ts are written.
)
{
    StatusCode result;

    // Sanity check
    if (buffer == NULL || bufferSize < 16)
    {
        return STATUS_INVALID;
    }

    // Mifare Classic protocol requires two communications to perform a write.
    // Step 1: Tell the PICC we want to write to block blockAddr.
    uint8_t cmdBuffer[2];
    cmdBuffer[0] = PICC_CMD_MF_WRITE;
    cmdBuffer[1] = blockAddr;
    result = PCD_MIFARE_Transceive(cmdBuffer, 2, 0); // Adds CRC_A and checks that the response is MF_ACK.
    if (result != STATUS_OK)
    {
        return result;
    }

    // Step 2: Transfer the data
    result = PCD_MIFARE_Transceive(buffer, bufferSize, 0); // Adds CRC_A and checks that the response is MF_ACK.
    if (result != STATUS_OK)
    {
        return result;
    }

    return STATUS_OK;
} // End MIFARE_Write()

/**
 * Writes a 4 uint8_t page to the active MIFARE Ultralight PICC.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode MIFARE_Ultralight_Write(uint8_t page,      ///< The page (2-15) to write to.
                                   uint8_t *buffer,   ///< The 4 uint8_ts to write to the PICC
                                   uint8_t bufferSize ///< Buffer size, must be at least 4 uint8_ts. Exactly 4 uint8_ts are written.
)
{
    StatusCode result;

    // Sanity check
    if (buffer == NULL || bufferSize < 4)
    {
        return STATUS_INVALID;
    }

    // Build commmand buffer
    uint8_t cmdBuffer[6];
    cmdBuffer[0] = PICC_CMD_UL_WRITE;
    cmdBuffer[1] = page;
    memcpy(&cmdBuffer[2], buffer, 4);

    // Perform the write
    result = PCD_MIFARE_Transceive(cmdBuffer, 6, 0); // Adds CRC_A and checks that the response is MF_ACK.
    if (result != STATUS_OK)
    {
        return result;
    }
    return STATUS_OK;
} // End MIFARE_Ultralight_Write()

/**
 * MIFARE Decrement subtracts the delta from the value of the addressed block, and stores the result in a volatile memory.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
 * Use MIFARE_Transfer() to store the result in a block.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode MIFARE_Decrement(uint8_t blockAddr, ///< The block (0-0xff) number.
                            int32_t delta      ///< This number is subtracted from the value of block blockAddr.
)
{
    return MIFARE_TwoStepHelper(PICC_CMD_MF_DECREMENT, blockAddr, delta);
} // End MIFARE_Decrement()

/**
 * MIFARE Increment adds the delta to the value of the addressed block, and stores the result in a volatile memory.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
 * Use MIFARE_Transfer() to store the result in a block.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode MIFARE_Increment(uint8_t blockAddr, ///< The block (0-0xff) number.
                            int32_t delta      ///< This number is added to the value of block blockAddr.
)
{
    return MIFARE_TwoStepHelper(PICC_CMD_MF_INCREMENT, blockAddr, delta);
} // End MIFARE_Increment()

/**
 * MIFARE Restore copies the value of the addressed block into a volatile memory.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
 * Use MIFARE_Transfer() to store the result in a block.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode MIFARE_Restore(uint8_t blockAddr ///< The block (0-0xff) number.
)
{
    // The datasheet describes Restore as a two step operation, but does not explain what data to transfer in step 2.
    // Doing only a single step does not work, so I chose to transfer 0L in step two.
    return MIFARE_TwoStepHelper(PICC_CMD_MF_RESTORE, blockAddr, 0L);
} // End MIFARE_Restore()

/**
 * Helper function for the two-step MIFARE Classic protocol operations Decrement, Increment and Restore.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode MIFARE_TwoStepHelper(uint8_t command,   ///< The command to use
                                uint8_t blockAddr, ///< The block (0-0xff) number.
                                int32_t data       ///< The data to transfer in step 2
)
{
    StatusCode result;
    uint8_t cmdBuffer[2]; // We only need room for 2 uint8_ts.

    // Step 1: Tell the PICC the command and block address
    cmdBuffer[0] = command;
    cmdBuffer[1] = blockAddr;
    result = PCD_MIFARE_Transceive(cmdBuffer, 2, 0); // Adds CRC_A and checks that the response is MF_ACK.
    if (result != STATUS_OK)
    {
        return result;
    }

    // Step 2: Transfer the data
    result = PCD_MIFARE_Transceive((uint8_t *)&data, 4, true); // Adds CRC_A and accept timeout as success.
    if (result != STATUS_OK)
    {
        return result;
    }

    return STATUS_OK;
} // End MIFARE_TwoStepHelper()

/**
 * MIFARE Transfer writes the value stored in the volatile memory into one MIFARE Classic block.
 * For MIFARE Classic only. The sector containing the block must be authenticated before calling this function.
 * Only for blocks in "value block" mode, ie with access bits [C1 C2 C3] = [110] or [001].
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode MIFARE_Transfer(uint8_t blockAddr ///< The block (0-0xff) number.
)
{
    StatusCode result;
    uint8_t cmdBuffer[2]; // We only need room for 2 uint8_ts.

    // Tell the PICC we want to transfer the result into block blockAddr.
    cmdBuffer[0] = PICC_CMD_MF_TRANSFER;
    cmdBuffer[1] = blockAddr;
    result = PCD_MIFARE_Transceive(cmdBuffer, 2, 0); // Adds CRC_A and checks that the response is MF_ACK.
    if (result != STATUS_OK)
    {
        return result;
    }
    return STATUS_OK;
} // End MIFARE_Transfer()

/**
 * Helper routine to read the current value from a Value Block.
 *
 * Only for MIFARE Classic and only for blocks in "value block" mode, that
 * is: with access bits [C1 C2 C3] = [110] or [001]. The sector containing
 * the block must be authenticated before calling this function.
 *
 * @param[in]   blockAddr   The block (0x00-0xff) number.
 * @param[out]  value       Current value of the Value Block.
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode MIFARE_GetValue(uint8_t blockAddr, int32_t *value)
{
    StatusCode status;
    uint8_t buffer[18];
    uint8_t size = sizeof(buffer);

    // Read the block
    status = MIFARE_Read(blockAddr, buffer, &size);
    if (status == STATUS_OK)
    {
        // Extract the value
        *value = (((int32_t)buffer[3]) << 24) | (((int32_t)buffer[2]) << 16) | (((int32_t)buffer[1]) << 8) | (int32_t)buffer[0];
    }
    return status;
} // End MIFARE_GetValue()

/**
 * Helper routine to write a specific value into a Value Block.
 *
 * Only for MIFARE Classic and only for blocks in "value block" mode, that
 * is: with access bits [C1 C2 C3] = [110] or [001]. The sector containing
 * the block must be authenticated before calling this function.
 *
 * @param[in]   blockAddr   The block (0x00-0xff) number.
 * @param[in]   value       New value of the Value Block.
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode MIFARE_SetValue(uint8_t blockAddr, int32_t value)
{
    uint8_t buffer[18];

    // Translate the int32_t into 4 uint8_ts; repeated 2x in value block
    buffer[0] = buffer[8] = (value & 0xFF);
    buffer[1] = buffer[9] = (value & 0xFF00) >> 8;
    buffer[2] = buffer[10] = (value & 0xFF0000) >> 16;
    buffer[3] = buffer[11] = (value & 0xFF000000) >> 24;
    // Inverse 4 uint8_ts also found in value block
    buffer[4] = ~buffer[0];
    buffer[5] = ~buffer[1];
    buffer[6] = ~buffer[2];
    buffer[7] = ~buffer[3];
    // Address 2x with inverse address 2x
    buffer[12] = buffer[14] = blockAddr;
    buffer[13] = buffer[15] = ~blockAddr;

    // Write the whole data block
    return MIFARE_Write(blockAddr, buffer, 16);
} // End MIFARE_SetValue()

/**
 * Authenticate with a NTAG216.
 *
 * Only for NTAG216. First implemented by Gargantuanman.
 *
 * @param[in]   passWord   password.
 * @param[in]   pACK       result success???.
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PCD_NTAG216_AUTH(uint8_t *passWord, uint8_t pACK[]) // Authenticate with 32bit password
{
    // TODO: Fix cmdBuffer length and rxlength. They really should match.
    //       (Better still, rxlength should not even be necessary.)

    StatusCode result;
    uint8_t cmdBuffer[18]; // We need room for 16 uint8_ts data and 2 uint8_ts CRC_A.

    cmdBuffer[0] = 0x1B; // Comando de autentificacion

    uint8_t i;
    for (i = 0; i < 4; i++)
        cmdBuffer[i + 1] = passWord[i];

    result = PCD_CalculateCRC(cmdBuffer, 5, &cmdBuffer[5]);

    if (result != STATUS_OK)
    {
        return result;
    }

    // Transceive the data, store the reply in cmdBuffer[]
    uint8_t waitIRq = 0x30; // RxIRq and IdleIRq
                            //	uint8_t cmdBufferSize	= sizeof(cmdBuffer);
    uint8_t validBits = 0;
    uint8_t rxlength = 5;
    result = PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, cmdBuffer, 7, cmdBuffer, &rxlength, &validBits, 0, 0);

    pACK[0] = cmdBuffer[0];
    pACK[1] = cmdBuffer[1];

    if (result != STATUS_OK)
    {
        return result;
    }

    return STATUS_OK;
} // End PCD_NTAG216_AUTH()

/////////////////////////////////////////////////////////////////////////////////////
// Support functions
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Wrapper for MIFARE protocol communication.
 * Adds CRC_A, executes the Transceive command and checks that the response is MF_ACK or a timeout.
 *
 * @return STATUS_OK on success, STATUS_??? otherwise.
 */
StatusCode PCD_MIFARE_Transceive(uint8_t *sendData, ///< Pointer to the data to transfer to the FIFO. Do NOT include the CRC_A.
                                 uint8_t sendLen,   ///< Number of uint8_ts in sendData.
                                 int acceptTimeout  ///< True => A timeout is also success
)
{
    StatusCode result;
    uint8_t cmdBuffer[18]; // We need room for 16 uint8_ts data and 2 uint8_ts CRC_A.

    // Sanity check
    if (sendData == NULL || sendLen > 16)
    {
        return STATUS_INVALID;
    }

    // Copy sendData[] to cmdBuffer[] and add CRC_A
    memcpy(cmdBuffer, sendData, sendLen);
    result = PCD_CalculateCRC(cmdBuffer, sendLen, &cmdBuffer[sendLen]);
    if (result != STATUS_OK)
    {
        return result;
    }
    sendLen += 2;

    // Transceive the data, store the reply in cmdBuffer[]
    uint8_t waitIRq = 0x30; // RxIRq and IdleIRq
    uint8_t cmdBufferSize = sizeof(cmdBuffer);
    uint8_t validBits = 0;
    result = PCD_CommunicateWithPICC(PCD_Transceive, waitIRq, cmdBuffer, sendLen, cmdBuffer, &cmdBufferSize, &validBits, 0, 0);
    if (acceptTimeout && result == STATUS_TIMEOUT)
    {
        return STATUS_OK;
    }
    if (result != STATUS_OK)
    {
        return result;
    }
    // The PICC must reply with a 4 bit ACK
    if (cmdBufferSize != 1 || validBits != 4)
    {
        return STATUS_ERROR;
    }
    if (cmdBuffer[0] != MF_ACK)
    {
        return STATUS_MIFARE_NACK;
    }
    return STATUS_OK;
} // End PCD_MIFARE_Transceive()

// /**
//  * Returns a __FlashStringHelper pointer to a status code name.
//  *
//  * @return const __FlashStringHelper *
//  */
// const __FlashStringHelper *GetStatusCodeName(StatusCode code	///< One of the StatusCode enums.
// 										) {
// 	switch (code) {
// 		case STATUS_OK:				return F("Success.");
// 		case STATUS_ERROR:			return F("Error in communication.");
// 		case STATUS_COLLISION:		return F("Collision detected.");
// 		case STATUS_TIMEOUT:		return F("Timeout in communication.");
// 		case STATUS_NO_ROOM:		return F("A buffer is not big enough.");
// 		case STATUS_INTERNAL_ERROR:	return F("Internal error in the code. Should not happen.");
// 		case STATUS_INVALID:		return F("Invalid argument.");
// 		case STATUS_CRC_WRONG:		return F("The CRC_A does not match.");
// 		case STATUS_MIFARE_NACK:	return F("A MIFARE PICC responded with NAK.");
// 		default:					return F("Unknown error");
// 	}
// } // End GetStatusCodeName()

/**
 * Translates the SAK (Select Acknowledge) to a PICC type.
 *
 * @return PICC_Type
 */
PICC_Type PICC_GetType(uint8_t sak ///< The SAK uint8_t returned from PICC_Select().
)
{
    // http://www.nxp.com/documents/application_note/AN10833.pdf
    // 3.2 Coding of Select Acknowledge (SAK)
    // ignore 8-bit (iso14443 starts with LSBit = bit 1)
    // fixes wrong type for manufacturer Infineon (http://nfc-tools.org/index.php?title=ISO14443A)
    sak &= 0x7F;
    switch (sak)
    {
    case 0x04:
        return PICC_TYPE_NOT_COMPLETE; // UID not complete
    case 0x09:
        return PICC_TYPE_MIFARE_MINI;
    case 0x08:
        return PICC_TYPE_MIFARE_1K;
    case 0x18:
        return PICC_TYPE_MIFARE_4K;
    case 0x00:
        return PICC_TYPE_MIFARE_UL;
    case 0x10:
    case 0x11:
        return PICC_TYPE_MIFARE_PLUS;
    case 0x01:
        return PICC_TYPE_TNP3XXX;
    case 0x20:
        return PICC_TYPE_ISO_14443_4;
    case 0x40:
        return PICC_TYPE_ISO_18092;
    default:
        return PICC_TYPE_UNKNOWN;
    }
} // End PICC_GetType()

/**
 * Calculates the bit pattern needed for the specified access bits. In the [C1 C2 C3] tuples C1 is MSB (=4) and C3 is LSB (=1).
 */
void MIFARE_SetAccessBits(uint8_t *accessBitBuffer, ///< Pointer to uint8_t 6, 7 and 8 in the sector trailer. uint8_ts [0..2] will be set.
                          uint8_t g0,               ///< Access bits [C1 C2 C3] for block 0 (for sectors 0-31) or blocks 0-4 (for sectors 32-39)
                          uint8_t g1,               ///< Access bits C1 C2 C3] for block 1 (for sectors 0-31) or blocks 5-9 (for sectors 32-39)
                          uint8_t g2,               ///< Access bits C1 C2 C3] for block 2 (for sectors 0-31) or blocks 10-14 (for sectors 32-39)
                          uint8_t g3                ///< Access bits C1 C2 C3] for the sector trailer, block 3 (for sectors 0-31) or block 15 (for sectors 32-39)
)
{
    uint8_t c1 = ((g3 & 4) << 1) | ((g2 & 4) << 0) | ((g1 & 4) >> 1) | ((g0 & 4) >> 2);
    uint8_t c2 = ((g3 & 2) << 2) | ((g2 & 2) << 1) | ((g1 & 2) << 0) | ((g0 & 2) >> 1);
    uint8_t c3 = ((g3 & 1) << 3) | ((g2 & 1) << 2) | ((g1 & 1) << 1) | ((g0 & 1) << 0);

    accessBitBuffer[0] = (~c2 & 0xF) << 4 | (~c1 & 0xF);
    accessBitBuffer[1] = c1 << 4 | (~c3 & 0xF);
    accessBitBuffer[2] = c3 << 4 | c2;
} // End MIFARE_SetAccessBits()

/////////////////////////////////////////////////////////////////////////////////////
// Convenience functions - does not add extra functionality
/////////////////////////////////////////////////////////////////////////////////////

/**
 * Returns true if a PICC responds to PICC_CMD_REQA.
 * Only "new" cards in state IDLE are invited. Sleeping cards in state HALT are ignored.
 *
 * @return int
 */
int PICC_IsNewCardPresent()
{
    uint8_t bufferATQA[2];
    uint8_t bufferSize = sizeof(bufferATQA);

    // Reset baud rates
    PCD_WriteRegister(TxModeReg, 0x00);
    PCD_WriteRegister(RxModeReg, 0x00);
    // Reset ModWidthReg
    PCD_WriteRegister(ModWidthReg, 0x26);

    StatusCode result = PICC_RequestA(bufferATQA, &bufferSize);
    return (result == STATUS_OK || result == STATUS_COLLISION);
} // End PICC_IsNewCardPresent()

/**
 * Simple wrapper around PICC_Select.
 * Returns true if a UID could be read.
 * Remember to call PICC_IsNewCardPresent(), PICC_RequestA() or PICC_WakeupA() first.
 * The read UID is available in the class variable uid.
 *
 * @return int
 */
int PICC_ReadCardSerial()
{
    StatusCode result = PICC_Select(&uid, 0);
    return (result == STATUS_OK);
} // End
