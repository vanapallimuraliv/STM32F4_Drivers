

#include "stm32f407xx.h"

static uint8_t dummy_write = 0xFF;
static uint8_t dummy_read;
uint8_t ack;
uint8_t arg[2];
uint8_t commandcode =0;

// commands
#define CMD_LED_CTRL		0x50
#define CMD_SENSOR_READ		0x51
#define CMD_LED_READ		0x52
#define CMD_PRINT			0x53
#define CMD_ID_READ			0x54

// LED states ON / OFF
#define LED_ON				1
#define LED_OFF				0

// LED pin numbers
#define LED_PIN				9

// sensor read
#define ANALOG_PIN0			0
#define ANALOG_PIN1			1
#define ANALOG_PIN2			2
#define ANALOG_PIN3			3
#define ANALOG_PIN4			4

// Helper functions
void GPIO_Initialization(void);
void SPI2_Initialization(void);
void delay(void);
uint8_t SPI_VerifyResponse(uint8_t ack);
void Command_LedCtrl();
void Command_SensorRead();
void command_LedRead(void);
void command_Print(void);
void command_ReadID(void);

/* PB14--> SPI2_MISO
 * PB15--> SPI2_MOSI
 * PB13--> SPI2_SCLK
 * PB12--> SPI2_NSS
 * ALT --5
 * */

int main()
{
	GPIO_Initialization();
	SPI2_Initialization();

	SPI_SSIConfig(SPI2, ENABLE);

	SPI_PeripheralCtl(SPI2, ENABLE);
	while(1)
	{
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay();
		SPI_PeripheralCtl(SPI2, ENABLE);
		// send first command
		Command_LedCtrl();

		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay();
		// send second command
		Command_SensorRead();

		// send 3rd command
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay();
		command_LedRead();

		// send 4th command
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay();
		command_Print();

		// send 5th command
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay();
		command_ReadID();

		while(SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG));

		SPI_PeripheralCtl(SPI2, DISABLE);
	}
	return 0;
}


void delay(void)
{
	for(uint32_t i =0; i<500000/2; i++);
}


void GPIO_Initialization(void)
{
	GPIO_Handle_t SPIPin2;
		SPIPin2.pGPIOx = GPIOB;
		SPIPin2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
		SPIPin2.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
		SPIPin2.GPIO_PinConfig.GPIO_PinOutType = GPIO_OUT_PUPL;
		SPIPin2.GPIO_PinConfig.GPIO_PinPuPdCtrl = GPIO_NO_PUPD;
		SPIPin2.GPIO_PinConfig.GPIO_PinSpeed = GPIO_HIGH_SPEED;

		// MOSI
		SPIPin2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
		GPIO_Init(&SPIPin2);


		// SCLK
		SPIPin2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
		GPIO_Init(&SPIPin2);
		 //MISO
		SPIPin2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
		GPIO_Init(&SPIPin2);

		//NSS
		SPIPin2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
		GPIO_Init(&SPIPin2);
}

void SPI2_Initialization(void)
{
	SPI_Handle_t SPI2Handle;
	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_config.SPI_DeviceMode = SPI_MASTER;
	SPI2Handle.SPI_config.SPI_BusConfig = SPI_BUS_CFG_FULLD;
	SPI2Handle.SPI_config.SPI_Speed = SPI_BAUD_RATE8; // speed 2Mbps;
	SPI2Handle.SPI_config.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPI_config.SPI_CPHA = SPI_CPHA0;
	SPI2Handle.SPI_config.SPI_CPOL = SPI_CPOL0;
	SPI2Handle.SPI_config.SPI_SSM = SPI_SSM_HW;

	SPI_Init(&SPI2Handle);
}

uint8_t SPI_VerifyResponse(uint8_t ack)
{
	if(ack == 0xF5)
	{
		//ACK
		return 1;
	}
	//NACK
	return 0;
}

void Command_LedCtrl()
{

		// 1.CMD_LED_CTRL		Pin no 		value(1)
		uint8_t commandcode = CMD_LED_CTRL;


		// Note: if we send some data form the master than read/ receive data from the slave
		// so, we send a command and receive dummy data

		// step 1:
		//send command CMD_LED_CTRL
		SPI_SendData(SPI2, &commandcode, 1);
		// Read a dummy data from the slave to clear off RXNE bit
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// step 2:
		// send some dummy data (1byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write,1);
		// Receive Acknoledgement from slave
		SPI_ReceiveData(SPI2, &ack, 1);

		if(SPI_VerifyResponse(ack))
		{
			// Argument assend Pin no
			arg[0] = LED_PIN;
			arg[1] = LED_ON;
			SPI_SendData(SPI2, arg, 2);
		}
		// end of command CMD_LED_CTRL
}

void Command_SensorRead()
{
		// CMD_SENSOR_READ   <Analog pin no.>
		commandcode = CMD_SENSOR_READ;
		// Send a command to slave
		SPI_SendData(SPI2, &commandcode, 1);
		// Read a dummy data from the slave to clear off RXNE bit
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// step 2:
		// send some dummy data (1byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write,1);
		// Receive Acknoledgement from slave
		SPI_ReceiveData(SPI2, &ack, 1);

		// Step 3:
		if(SPI_VerifyResponse(ack))
		{
			arg[0] = ANALOG_PIN0;
			// Argument as send Pin no
			SPI_SendData(SPI2, arg, 1);

	// Step 4:
			//send some dummy data (1byte) to fetch the response from the slave
			SPI_SendData(SPI2, &dummy_write,1);

			//insert some delay so that analog pin can be read in arduino
			delay();

			// Receive Acknoledgement from slave
			SPI_ReceiveData(SPI2, &ack, 1);

	// step 5:
			uint8_t AnalogRead;
			// Read a analog pin from the slave
			SPI_ReceiveData(SPI2, &AnalogRead, 1);
		}

}


void command_LedRead(void)
{
	commandcode = CMD_LED_READ;
	// step 1:
			//send command CMD_LED_CTRL
			SPI_SendData(SPI2, &commandcode, 1);
			// Read a dummy data from the slave to clear off RXNE bit
			SPI_ReceiveData(SPI2, &dummy_read, 1);

	// step 2:
			// send some dummy data (1byte) to fetch the response from the slave
			SPI_SendData(SPI2, &dummy_write,1);
			// Receive Acknoledgement from slave
			SPI_ReceiveData(SPI2, &ack, 1);

	// Step 3:
		if(SPI_VerifyResponse(ack))
		{
			arg[1] = LED_PIN;
			// Argument as send Pin no
			SPI_SendData(SPI2, arg, 1);
			//
			SPI_SendData(SPI2, &dummy_write,1);

			//insert some delay so that analog pin can be read in arduino
			delay();

			// Receive Acknoledgement from slave
			SPI_ReceiveData(SPI2, &ack, 1);

	// step 4:
			uint8_t LedRead;
			// Read a analog pin from the slave
			SPI_ReceiveData(SPI2, &LedRead, 1);
		}
}

void command_Print(void)
{
	commandcode = CMD_PRINT;
	// step 1:
			//send command CMD_LED_CTRL
			SPI_SendData(SPI2, &commandcode, 1);
			// Read a dummy data from the slave to clear off RXNE bit
			SPI_ReceiveData(SPI2, &dummy_read, 1);

	// step 2:
			// send some dummy data (1byte) to fetch the response from the slave
			SPI_SendData(SPI2, &dummy_write,1);
			// Receive Acknoledgement from slave
			SPI_ReceiveData(SPI2, &ack, 1);

	if(SPI_VerifyResponse(ack))
			{
				char msg[] = "Hi, How are you";
				// Argument as send Pin no
				arg[0] = strlen((char*)msg);

				// sending a Massage Length
				SPI_SendData(SPI2, arg, 1);
				//Sending Massage
				SPI_SendData(SPI2, (uint8_t*)msg, arg[0]);
			}

}
void command_ReadID(void)
{
	commandcode = CMD_ID_READ;
	// step 1:
			//send command CMD_LED_CTRL
			SPI_SendData(SPI2, &commandcode, 1);
			// Read a dummy data from the slave to clear off RXNE bit
			SPI_ReceiveData(SPI2, &dummy_read, 1);

	// step 2:
			// send some dummy data (1byte) to fetch the response from the slave
			SPI_SendData(SPI2, &dummy_write,1);
			// Receive Acknoledgement from slave
			SPI_ReceiveData(SPI2, &ack, 1);

	// step 3:
	uint8_t ReadID[10];
	uint8_t i= 0;
	if(SPI_VerifyResponse(ack))
		{
		// send some dummy data (1byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummy_write,1);
		for(i=0; i<10; i++)
			{
			// Read a ID from the slave
			SPI_ReceiveData(SPI2, &ReadID[i], 1);
			}

		ReadID[11] = '\0';
		}
}
