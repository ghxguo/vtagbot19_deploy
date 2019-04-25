//********************************************************************************************
//                                                                                           *
// AB&T Tecnologie Informatiche - Ivrea Italy                                                *
// http://www.bausano.net                                                                    *
// https://www.ethercat.org/en/products/791FFAA126AD43859920EA64384AD4FD.htm                 *
//                                                                                           *  
//********************************************************************************************    

//********************************************************************************************    
//                                                                                           *
// This software is distributed as an example, "AS IS", in the hope that it could            *
// be useful, WITHOUT ANY WARRANTY of any kind, express or implied, included, but            *
// not limited,  to the warranties of merchantability, fitness for a particular              *
// purpose, and non infringiment. In no event shall the authors be liable for any            *    
// claim, damages or other liability, arising from, or in connection with this software.     *
//                                                                                           *
//******************************************************************************************** 



//---- AB&T EasyCAT shield application example V.2_0 -------------------------------------------  



#include "EasyCAT.h"                // EasyCAT library to interface the LAN9252
#include <SPI.h>                    // SPI library

 
EasyCAT EASYCAT;                    // EasyCAT istantiation

                                    // The constructor allow us to choose the pin used for the EasyCAT SPI chip select 
                                    // Without any parameter pin 9 will be used 
                   
                                    // We can choose between:
                                    // 8, 9, 10, A5, 6, 7                                    

                                    // On the EasyCAT board the SPI chip select is selected through a bank of jumpers              

                                    // (The EasyCAT board REV_A allows only pins 8, 9, 10 through 0 ohm resistors)

 //EasyCAT EASYCAT(8);              // example:                                  
                                    // pin 8 will be used as SPI chip select
                                    // The chip select chosen by the firmware must match the setting on the board  


//---- pins declaration ------------------------------------------------------------------------------

int PreviousMillis = 0;
int Millis = 0;


//---- global variables ---------------------------------------------------------------------------


//---- setup ---------------------------------------------------------------------------------------
 
void setup()
{
  Serial.begin(9600);                                             // serial line initialization
                                                                  //(used only for debug)
           
  Serial.print ("\nEasyCAT - Generic EtherCAT slave\n");          // print the banner
                                      //---- initialize the EasyCAT board -----
                                                                  
  if (EASYCAT.Init() == true)                                     // initialization
  {                                                               // succesfully completed
    Serial.print ("initialized");                                 //
  }                                                               //
  
  else                                                            // initialization failed   
  {                                                               // the EasyCAT board was not recognized
    Serial.print ("initialization failed");                       //     
                                                                  // The most common reason is that the SPI 
                                                                  // chip select choosen on the board doesn't 
                                                                // with the Arduino led blinking
    while(1)                                                      //
    {                                                             //   
                                                // 
    }                                                             // 
  } 
  

}


//---- main loop ----------------------------------------------------------------------------------------
 
void loop()                                             // In the main loop we must call ciclically the 
{                                                       // EasyCAT task and our application
                                                        //
                                                        // This allows the bidirectional exachange of the data
                                                        // between the EtherCAT master and our application
                                                        //
                                                        // The EasyCAT cycle and the Master cycle are asynchronous
                                                        //   

  EASYCAT.MainTask();                                   // execute the EasyCAT task
  
  Application();   
  delay(10);// user applications
}



//---- user application ------------------------------------------------------------------------------

void Application ()                                        

{

    
  Millis = millis();                                    // As an example for this application 
  if (Millis - PreviousMillis >= 10)                    // we choose a cycle time of 10 mS 
  {                                                     // 
    PreviousMillis = Millis;                            //
 
                                                        // --- analog inputs management ---

    EASYCAT.BufferIn.Byte[0] =  0x12;  
    EASYCAT.BufferIn.Byte[1] =  0x34;      // and put the result into
    EASYCAT.BufferIn.Byte[2] =  0x56;                                                    // input Byte 1   
    EASYCAT.BufferIn.Byte[3] =  0x78;
    EASYCAT.BufferIn.Byte[4] =  0x9A;                                                   // --- four output bits management ----
    EASYCAT.BufferIn.Byte[5] =  0xBC;
    EASYCAT.BufferIn.Byte[6] =  0xDE;
    EASYCAT.BufferIn.Byte[7] =  0xFF;
    EASYCAT.BufferIn.Byte[8] =  0x12;  
    EASYCAT.BufferIn.Byte[9] =  0x34;      // and put the result into
    EASYCAT.BufferIn.Byte[10] =  0x56;                                                    // input Byte 1   
    EASYCAT.BufferIn.Byte[11] =  0x78;
    EASYCAT.BufferIn.Byte[12] =  0x9A;                                                   // --- four output bits management ----
    EASYCAT.BufferIn.Byte[13] =  0xBC;
    EASYCAT.BufferIn.Byte[14] =  0xDE;
    EASYCAT.BufferIn.Byte[15] =  0xFF; 
    for (int i = 0; i < 16; i++)
    {
      int data = (EASYCAT.BufferOut.Byte[i*2] << 8) | EASYCAT.BufferOut.Byte[i*2+1]; 
      Serial.print(data);
      Serial.print(',');
    }                
    Serial.println();
  }   
}
