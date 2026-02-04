/**
 * @file           : can.c
 * @brief          : CAN handling functions
 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "main.h"
#include "stm32f429i_discovery_lcd.h"
#include "tempsensor.h" // Stellen Sie sicher, dass die Datei so heißt!

// 45 MHz / (22 * 16) = ~128 kBit/s
#define  CAN1_CLOCK_PRESCALER     16

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
	printf("CAN1: Temp + Name");

	LCD_SetColors(LCD_COLOR_GREEN, LCD_COLOR_BLACK);
	LCD_SetPrintPosition(5, 1);
	printf("Send-Cnt:");
	LCD_SetPrintPosition(7, 1);
	printf("Recv-Cnt:");
	LCD_SetPrintPosition(9, 1);
	printf("Data:");

	// Temperatur-Sensor initialisieren
	tempSensorInit();
}

/**
 * sends a CAN frame with Name, Counter and Temperature
 */
void canSendTask(void) {
	static uint8_t sendCnt = 0;
	CAN_TxHeaderTypeDef txHeader;
	uint8_t txData[8] = { 0 };
	uint32_t txMailbox;

	// 1. Sensordaten holen
	float temp = tempSensorGetTemperature();
	uint16_t tempScaled = (uint16_t) (temp * 10);

	// ÄNDERN SIE HIER IHREN NAMEN (Max 5 Zeichen!)
	char myName[] = "Michi";

	// Prüfen, ob eine Mailbox frei ist (Sicherheits-Check)
	if (HAL_CAN_GetTxMailboxesFreeLevel(&canHandle) == 0)
		return;

	// 2. Datenpaket "packen"
	// Bytes 0-4: Name kopieren
	strncpy((char*) txData, myName, 5);

	// Byte 5: Zähler
	txData[5] = sendCnt;

	// Byte 6-7: Temperatur (Little Endian)
	txData[6] = (uint8_t) (tempScaled & 0xFF);
	txData[7] = (uint8_t) ((tempScaled >> 8) & 0xFF);

	// 3. CAN Header konfigurieren
	txHeader.StdId = 0x0F5; // Die ID der Nachricht
	txHeader.ExtId = 0x00;
	txHeader.RTR = CAN_RTR_DATA;
	txHeader.IDE = CAN_ID_STD;
	txHeader.DLC = 8;

	// 4. Senden
	if (HAL_CAN_AddTxMessage(&canHandle, &txHeader, txData, &txMailbox)
			== HAL_OK) {
		sendCnt++;
	}

	// 5. Anzeige (Lokal)
	LCD_SetColors(LCD_COLOR_GREEN, LCD_COLOR_BLACK);

	// Zähler aktualisieren
	LCD_SetPrintPosition(5, 15);
	printf("%5d", sendCnt);

	// Infozeile
	LCD_SetPrintPosition(11, 1);
	printf("Gesendet: %s, %.1fC   ", myName, temp);
}

/**
 * checks if a can frame has been received and shows content on display
 */
// Funktion für UART Logging in Terraterm NÄCHSTES MAL HINZUFÜGEN

void canReceiveTask(void) {
	static unsigned int recvTotal = 0;
	CAN_RxHeaderTypeDef rxHeader;
	uint8_t rxData[8] = { 0 };

	// Prüfen, ob Daten im FIFO liegen
	if (HAL_CAN_GetRxFifoFillLevel(&canHandle, CAN_RX_FIFO0) != 0) {
		// Daten abholen
		if (HAL_CAN_GetRxMessage(&canHandle, CAN_RX_FIFO0, &rxHeader, rxData)
				== HAL_OK) {
			recvTotal++;

			// --- Extraktion (Entpacken) ---

			// Name: Die ersten 5 Bytes (wir brauchen einen Puffer mit \0 am Ende)
			char rName[6] = { 0 };
			strncpy(rName, (char*) rxData, 5);

			// Zähler: Byte 5
			uint8_t rCnt = rxData[5];

			// Temperatur: Byte 6 und 7 wieder zusammensetzen
			uint16_t rTempRaw = (uint16_t) rxData[6]
					| ((uint16_t) rxData[7] << 8);
			float rTemp = (float) rTempRaw / 10.0f;

			// --- Anzeige am LCD ---
			LCD_SetColors(LCD_COLOR_CYAN, LCD_COLOR_BLACK);

			LCD_SetPrintPosition(15, 1);
			printf("RX von: %s (#%d)   ", rName, rCnt);

			LCD_SetPrintPosition(17, 1);
			printf("Temp:   %.1f C     ", rTemp);

			// Empfangszähler aktualisieren
			LCD_SetPrintPosition(7, 15);
			printf("%5d", recvTotal);

			// Debug: Rohdaten Hex anzeigen (optional)
			LCD_SetPrintPosition(19, 1);
			printf("Hex: %02x %02x ...", rxData[0], rxData[1]);

		}
	}
}

// --- Hardware Init (GPIO & CAN Core) ---

static void initGpio(void) {
	GPIO_InitTypeDef canPins;
	__HAL_RCC_GPIOB_CLK_ENABLE();
	canPins.Alternate = GPIO_AF9_CAN1;
	canPins.Mode = GPIO_MODE_AF_OD;
	canPins.Pin = GPIO_PIN_8 | GPIO_PIN_9; // RX | TX
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
	canHandle.Init.Mode = CAN_MODE_LOOPBACK;				// zum selbsttesten
	canHandle.Init.SyncJumpWidth = CAN_SJW_1TQ;
	canHandle.Init.TimeSeg1 = CAN_BS1_15TQ;
	canHandle.Init.TimeSeg2 = CAN_BS2_6TQ;
	canHandle.Init.Prescaler = CAN1_CLOCK_PRESCALER;

	if (HAL_CAN_Init(&canHandle) != HAL_OK) {
		// Error_Handler(); // Falls Sie keinen Error_Handler haben, Endlosschleife:
		while (1)
			;
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
		while (1)
			;
	}
	if (HAL_CAN_Start(&canHandle) != HAL_OK) {
		while (1)
			;
	}
}

// Interrupt Handler (falls Interrupts genutzt würden, hier aber Polling in ReceiveTask)
void CAN1_RX0_IRQHandler(void) {
	HAL_CAN_IRQHandler(&canHandle);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	// Receive is done in main loop via Polling
}
