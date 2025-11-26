/**
 ******************************************************************************
 * @file           : can.c
 * @brief          : CAN handling functions
 ******************************************************************************
 */
#include <stdio.h>
#include <stdint.h>
#include "main.h"
#include "stm32f429i_discovery_lcd.h"
#include "tempsensor.h"

// ToDo: korrekte Prescaler-Einstellung
// 45 MHz / (22 * 16) = ~128 kBit/s
#define   CAN1_CLOCK_PRESCALER    16  //

CAN_HandleTypeDef canHandle;

static void initGpio(void);
static void initCanPeripheral(void);

void canInitHardware(void) {
	initGpio();
	initCanPeripheral();
}

/**
 * canInit function, set up hardware and display
 */
void canInit(void) {
	canInitHardware();

	LCD_SetFont(&Font12);
	LCD_SetColors(LCD_COLOR_WHITE, LCD_COLOR_BLACK);
	LCD_SetPrintPosition(3, 1);
	printf("CAN1: Send-Recv (C-Impl)"); // Hinweis auf C-Version

	LCD_SetColors(LCD_COLOR_GREEN, LCD_COLOR_BLACK);
	LCD_SetPrintPosition(5, 1);
	printf("Send-Cnt:");
	LCD_SetPrintPosition(5, 15);
	printf("%5d", 0);
	LCD_SetPrintPosition(7, 1);
	printf("Recv-Cnt:");
	LCD_SetPrintPosition(7, 15);
	printf("%5d", 0);
	LCD_SetPrintPosition(9, 1);
	printf("Send-Data:");
	LCD_SetPrintPosition(15, 1);
	printf("Recv-Data:");

	LCD_SetPrintPosition(30, 1);
	printf("Bit-Timing-Register: 0x%lx", CAN1->BTR);

	// ToDo (2): set up DS18B20 (temperature sensor)
	tempSensorInit(); //
}

/**
 * sends a CAN frame, if mailbox is free
 */
void canSendTask(void) {
	// ToDo declare the required variables
	static uint8_t sendCnt = 0;
	CAN_TxHeaderTypeDef txHeader;
	uint8_t txData[8] = { 0 };
	uint32_t txMailbox;
	float temperature = 0.0f;
	uint16_t tempScaled = 0;

	// Prüfen, ob eine Mailbox frei ist (mindestens 1 von 3)
	if (HAL_CAN_GetTxMailboxesFreeLevel(&canHandle) == 0) {
		return; // Keine Mailbox frei, abbrechen
	}

	// ToDo (2): get temperature value
	temperature = tempSensorGetTemperature(); //

	// Temperatur skalieren (z.B. 25.5°C -> 255)
	tempScaled = (uint16_t) (temperature * 10);

	// ToDo prepare send data
	// Frame-Aufbau laut Aufgabenstellung:
	// Byte 0: 0xAF
	// Byte 1: Vorwärtszähler
	// Byte 2+3: Temperatur (Stufe 2)
	txData[0] = 0xAF;      //
	txData[1] = sendCnt;   //
	txData[2] = (uint8_t) (tempScaled & 0xFF);      // Low Byte
	txData[3] = (uint8_t) ((tempScaled >> 8) & 0xFF); // High Byte

	// Header konfigurieren
	txHeader.StdId = 0x0F5;         // Beispiel-ID
	txHeader.ExtId = 0x00;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.IDE = CAN_ID_STD;
	txHeader.DLC = 4;               // Länge: 4 Bytes

	// ToDo send CAN frame
	if (HAL_CAN_AddTxMessage(&canHandle, &txHeader, txData, &txMailbox)
			== HAL_OK) {
		sendCnt++; // Nur erhöhen, wenn erfolgreich in Mailbox gelegt
	}

	// ToDo display send counter and send data
	LCD_SetColors(LCD_COLOR_GREEN, LCD_COLOR_BLACK);

	// Zähler anzeigen
	LCD_SetPrintPosition(5, 15);
	printf("%5d", sendCnt);

	// Temperatur Text anzeigen
	LCD_SetPrintPosition(11, 1);
	printf("T: %3.2f C", temperature);

	// Datenbytes anzeigen (Hex)
	LCD_SetPrintPosition(9, 13); // Zeile 9, hinter "Send-Data:"
	printf("%02x %02x %02x %02x", txData[0], txData[1], txData[2], txData[3]);
}

/**
 * checks if a can frame has been received and shows content on display
 */
void canReceiveTask(void) {
	static unsigned int recvCnt = 0;
	CAN_RxHeaderTypeDef rxHeader;
	uint8_t rxData[8] = { 0 };

	// NEU: Variablen zur Speicherung der decodierten Werte
	uint8_t receivedCounter = 0;
	uint16_t receivedTempScaled = 0;
	float receivedTemperature = 0.0f;

	// ToDo: check if CAN frame has been received
	// Prüfen, ob Füllstand von FIFO0 ungleich 0 ist
	if (HAL_CAN_GetRxFifoFillLevel(&canHandle, CAN_RX_FIFO0) != 0) {
		// ToDo: Get CAN frame from RX fifo
		if (HAL_CAN_GetRxMessage(&canHandle, CAN_RX_FIFO0, &rxHeader, rxData)
				== HAL_OK) {
			// ToDo: Process received CAN Frame (extract data)
			recvCnt++;

			// --- DECODIERUNG DER DATEN ---

			// 1. Zählerwert: Byte 1 (rxData[1]) enthält den Zähler
			receivedCounter = rxData[1];

			// 2. Temperaturwert: Bytes 2 und 3 zusammenfügen (Low Byte + High Byte)
			// Daten[2] ist Low Byte, Daten[3] ist High Byte (Little Endian Konvention im Sender)
			receivedTempScaled = (uint16_t) rxData[2]
					| ((uint16_t) rxData[3] << 8);

			// 3. Temperatur zurückrechnen: Wir haben mit 10 multipliziert, also jetzt durch 10 teilen
			receivedTemperature = (float) receivedTempScaled / 10.0f;

			// --- ENDE DECODIERUNG ---

			// ToDo display recv counter and recv data
			LCD_SetColors(LCD_COLOR_GREEN, LCD_COLOR_BLACK);

			// Empfangszähler aktualisieren
			LCD_SetPrintPosition(7, 15);
			printf("%5d", recvCnt);

			// NEU: Anzeige der decodierten, verständlichen Werte
			LCD_SetPrintPosition(17, 1);
			printf("ID: %03lx | Cnt: %3d", rxHeader.StdId, receivedCounter);

			// Temperaturwert anzeigen
			LCD_SetPrintPosition(19, 1);
			printf("T: %3.2f C", receivedTemperature);

			// Hex-Daten zum Vergleich (optional, aber hilfreich):
			LCD_SetPrintPosition(15, 1);
			printf("Recv-Data: %02x %02x %02x %02x", rxData[0], rxData[1],
					rxData[2], rxData[3]);
		}
	}
}

// ... initGpio und initCanPeripheral bleiben gleich wie im vorherigen Code ...
static void initGpio(void) {
	GPIO_InitTypeDef canPins;
	__HAL_RCC_GPIOB_CLK_ENABLE();
	canPins.Alternate = GPIO_AF9_CAN1;
	canPins.Mode = GPIO_MODE_AF_OD;
	canPins.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	canPins.Pull = GPIO_PULLUP;
	canPins.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOB, &canPins);
}

static void initCanPeripheral(void) {
	CAN_FilterTypeDef canFilter;
	__HAL_RCC_CAN1_CLK_ENABLE();

	canHandle.Instance = CAN1;
	canHandle.Init.TimeTriggeredMode = DISABLE;
	canHandle.Init.AutoBusOff = DISABLE;
	canHandle.Init.AutoWakeUp = DISABLE;
	canHandle.Init.AutoRetransmission = ENABLE;
	canHandle.Init.ReceiveFifoLocked = DISABLE;
	canHandle.Init.TransmitFifoPriority = DISABLE;
	canHandle.Init.Mode = CAN_MODE_NORMAL;
	canHandle.Init.SyncJumpWidth = CAN_SJW_1TQ;
	canHandle.Init.TimeSeg1 = CAN_BS1_15TQ;
	canHandle.Init.TimeSeg2 = CAN_BS2_6TQ;
	canHandle.Init.Prescaler = CAN1_CLOCK_PRESCALER;

	if (HAL_CAN_Init(&canHandle) != HAL_OK) {
		Error_Handler();
	}

	canFilter.FilterBank = 0;
	canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
	canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
	canFilter.FilterIdHigh = 0x0000;
	canFilter.FilterIdLow = 0x0000;
	canFilter.FilterMaskIdHigh = 0x0000;
	canFilter.FilterMaskIdLow = 0x0000;
	canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
	canFilter.FilterActivation = ENABLE;
	canFilter.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&canHandle, &canFilter) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_CAN_Start(&canHandle) != HAL_OK) {
		Error_Handler();
	}
}

void CAN1_RX0_IRQHandler(void) {
	HAL_CAN_IRQHandler(&canHandle);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	// Receive is done in main loop via Polling
}
