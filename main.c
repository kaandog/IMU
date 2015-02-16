/******************************************************************************
*  Nano-RK, a real-time operating system for sensor networks.
*  Copyright (C) 2007, Real-Time and Multimedia Lab, Carnegie Mellon University
*  All rights reserved.
*
*  This is the Open Source Version of Nano-RK included as part of a Dual
*  Licensing Model. If you are unsure which license to use please refer to:
*  http://www.nanork.org/nano-RK/wiki/Licensing
*
*  This program is free software: you can redistribute it and/or modify
*  it under the terms of the GNU General Public License as published by
*  the Free Software Foundation, version 2.0 of the License.
*
*  This program is distributed in the hope that it will be useful,
*  but WITHOUT ANY WARRANTY; without even the implied warranty of
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*  GNU General Public License for more details.
*
*  You should have received a copy of the GNU General Public License
*  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*******************************************************************************/

#include <nrk.h>
#include <include.h>
#include <ulib.h>
#include <stdio.h>
#include <hal.h>
#include <nrk_error.h>
#include <nrk_timer.h>
#include <nrk_stack_check.h>
#include <nrk_stats.h>
#include <TWI_Master.h>



#define TWI_GEN_CALL         0x00  // The General Call address is 0

// Sample TWI transmission commands
#define TWI_CMD_MASTER_WRITE 0x10
#define TWI_CMD_MASTER_READ  0x20

// Sample TWI transmission states, used in the main application.
#define SEND_DATA             0x01
#define REQUEST_DATA          0x02
#define READ_DATA_FROM_BUFFER 0x03


// Constants for the accelrometer
//There are 6 data registers, they are sequential starting 
//with the LSB of X.  We'll read all 6 in a burst and won't
//address them individually
#define ADXL345_REGISTER_XLSB (0x32)

//Need to set power control bit to wake up the adxl345
#define ADXL_REGISTER_PWRCTL (0x2D)
#define ADXL_PWRCTL_MEASURE (1 << 3)
#define  ADXL345_ADDRESS (0xA6)

unsigned char TWI_Act_On_Failure_In_Last_Transmission ( unsigned char TWIerrorMsg )
{
                    // A failure has occurred, use TWIerrorMsg to determine the nature of the failure
                    // and take appropriate actions.
                    // Se header file for a list of possible failures messages.
                    
                    // Here is a simple sample, where if received a NACK on the slave address,
                    // then a retransmission will be initiated.
if ( (TWIerrorMsg == TWI_MTX_ADR_NACK) | (TWIerrorMsg == TWI_MRX_ADR_NACK) ){
    TWI_Start_Transceiver();
}
printf("%c \n",TWIerrorMsg);
    
  return TWIerrorMsg; 
}




NRK_STK Stack1[NRK_APP_STACKSIZE];
nrk_task_type TaskOne;
void Task1(void);

void init_adxl345(void);


void nrk_create_taskset();

int
main ()
{
  nrk_setup_ports();
  nrk_setup_uart(UART_BAUDRATE_115K2);

  TWI_Master_Initialise();
  sei();
  init_adxl345();
  nrk_init();

  nrk_led_clr(ORANGE_LED);
  nrk_led_clr(BLUE_LED);
  nrk_led_clr(GREEN_LED);
  nrk_led_clr(RED_LED);
 
  nrk_time_set(0,0);
  nrk_create_taskset ();
  nrk_start();
  
  return 0;
}

uint8_t messageBuf[16];
unsigned int read = 0;
void init_adxl345() {
  while(!read){
  messageBuf[0] = (ADXL345_ADDRESS<<TWI_ADR_BITS) | (FALSE<<TWI_READ_BIT);
  messageBuf[1] = ADXL_REGISTER_PWRCTL;
  messageBuf[2] = ADXL_PWRCTL_MEASURE;
  TWI_Start_Transceiver_With_Data(messageBuf, 3);
  //Check to see if it worked!
  messageBuf[0] = ADXL345_ADDRESS | TRUE<<TWI_READ_BIT;
  messageBuf[1] = ADXL_REGISTER_PWRCTL;
  TWI_Start_Transceiver_With_Data(messageBuf, 2);
  messageBuf[0] = 0;
  messageBuf[1] = 0;
  messageBuf[2] = 0;
  read = TWI_Get_Data_From_Transceiver(messageBuf, 3);
  }
  // printf("sent: %x \n",(unsigned int)TWI_Get_Data_From_Transceiver(messageBuf, 3));
  printf("sent!\r\n");
  printf("ctrl register: %x \r\n",(unsigned int) messageBuf[2]);
  printf("}\n");
}


void Task_Accelorometer()
{
// while(1){nrk_wait_until_next_period();}
while(1){
  //read 6 bytes from the ADXL345
  // messageBuf[0] = (ADXL345_ADDRESS<<TWI_ADR_BITS) | (FALSE<<TWI_READ_BIT);
  // messageBuf[1] = ADXL345_REGISTER_XLSB;
  // TWI_Start_Transceiver_With_Data(messageBuf, 3);
  // messageBuf[0] = 0;
  // messageBuf[1] = 0;
  // printf("sent: %x \r\n",(unsigned int)TWI_Get_Data_From_Transceiver(messageBuf, 1));

  // messageBuf[0] = ((ADXL345_ADDRESS<<TWI_ADR_BITS) | (FALSE<<TWI_READ_BIT));
  // messageBuf[1] = ADXL345_REGISTER_XLSB;
  // TWI_Start_Transceiver_With_Data(messageBuf, 2);

  // /* Read low byte */
  // messageBuf[0] = (ADXL345_ADDRESS<<TWI_ADR_BITS) |
  //                 (TRUE<<TWI_READ_BIT);
  // TWI_Start_Transceiver_With_Data(messageBuf, 2);
  // TWI_Get_Data_From_Transceiver(messageBuf, 2);
  // // uint16_t pixel_l = messageBuf[1];
  // printf("block1 : %x \r\n",messageBuf[1]);
  nrk_wait_until_next_period();
}
  // while (1) {
  //     pixel_addr_l=0x80;
  //     pixel_addr_h=0x81;

  //     int row, col, status;
  //     for (row = 0; row < 8; row++) {
  //         for (col = 0; col < 8; col++) {
  //             /* Request low byte */
  //             messageBuf[0] = (ADXL345_ADDRESS<<TWI_ADR_BITS) | (FALSE<<TWI_READ_BIT);
  //             messageBuf[1] = ADXL_REGISTER_PWRCTL;
  //             TWI_Start_Transceiver_With_Data(messageBuf, 2);

  //             /* Read low byte */
              // messageBuf[0] = (grideye_addr<<TWI_ADR_BITS) |
              //     (TRUE<<TWI_READ_BIT);
              // TWI_Start_Transceiver_With_Data(messageBuf, 2);
              // TWI_Get_Data_From_Transceiver(messageBuf, 2);
  //             uint16_t pixel_l = messageBuf[1];

  //             /* Request high byte */
  //             messageBuf[0] = (grideye_addr<<TWI_ADR_BITS) |
  //                 (FALSE<<TWI_READ_BIT);
  //             messageBuf[1] = pixel_addr_h;
  //             TWI_Start_Transceiver_With_Data(messageBuf, 2);

  //             /* Read high byte */
  //             messageBuf[0] = (grideye_addr<<TWI_ADR_BITS) |
  //                 (TRUE<<TWI_READ_BIT);
  //             TWI_Start_Transceiver_With_Data(messageBuf, 2);
  //             TWI_Get_Data_From_Transceiver(messageBuf, 2);
  //             uint16_t pixel_h = messageBuf[1];

  //             /* Store pixel and advance */
  //             raw_therm[row][col] = (pixel_h << 8) | pixel_l;
  //             pixel_addr_l += 2;
  //             pixel_addr_h += 2;
  //         }
  //         int n = sprintf(printbuf, "$ADXL345:,%d,%04X,%04X,%04X,%04X,%04X,%04X,%04X,%04X*",
  //                 row,
  //                 raw_therm[row][0], raw_therm[row][1],
  //                 raw_therm[row][2], raw_therm[row][3],
  //                 raw_therm[row][4], raw_therm[row][5],
  //                 raw_therm[row][6], raw_therm[row][7]);
  //         uint8_t i;
  //         uint8_t checksum = 0;
  //         for (i = 1; i < n - 1; i++) checksum ^= printbuf[i];
  //         printf("%s%02X\r\n", printbuf, checksum);
  //     }
  //     nrk_wait_until_next_period();
}



void
nrk_create_taskset()
{
  nrk_task_set_entry_function( &TaskOne, Task_Accelorometer);
  nrk_task_set_stk( &TaskOne, Stack1, NRK_APP_STACKSIZE);
  TaskOne.prio = 1;
  TaskOne.FirstActivation = TRUE;
  TaskOne.Type = BASIC_TASK;
  TaskOne.SchType = PREEMPTIVE;
  TaskOne.period.secs = 0;
  TaskOne.period.nano_secs = 250*NANOS_PER_MS;
  TaskOne.cpu_reserve.secs = 0;
  TaskOne.cpu_reserve.nano_secs = 100*NANOS_PER_MS;
  TaskOne.offset.secs = 1;
  TaskOne.offset.nano_secs= 0;
  nrk_activate_task (&TaskOne);
}



