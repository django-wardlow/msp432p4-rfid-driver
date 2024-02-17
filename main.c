#include "ti/devices/msp432p4xx/driverlib/driverlib.h"
#include "msp.h"
#include "rfid_library.h"

///turn the uid from the last read card into a
///single number that can be easily understood by
///summing the bytes
uint32_t get_uid_sum()
{
    uint32_t sum = 0;
    int i;
    for (i = 0; i < uid.size; i++)
    {
        sum += uid.uiduint8_t[i];
    }

    return sum;
}

/**
 * main.c
 */
void main(void)
{
    //halt watchdog
    WDT_A_holdTimer();

    //configure clock
    configHFXT();

    //init the rfid module
    PCD_Init();

    while(1){
        //if a new card is present and it is readable
        if (PICC_IsNewCardPresent() && PICC_ReadCardSerial())
        {
            uint32_t uid_sum = get_uid_sum();
            printf("card uid: %d", uid_sum);

            //turns the selected card off so that it is not re-detected until
            //it is out of range and then back in range
            PICC_HaltA();
        }


    }


}
