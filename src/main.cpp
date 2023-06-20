#include <SPI.h>
#include <Update.h>
#include <LoRa.h>

#define BAND 868E6 // LoRa-Frequenz

const int csPin = 18;    // LoRa radio chip select
const int resetPin = 14; // LoRa radio reset
const int irqPin = 26;   // change for your board; must be a hardware interrupt pin

uint16_t expectedMessageNumber = 0;
size_t totalBytesRead = 0;
size_t updateSize = -1;
bool updateEnd = false;
// const esp_partition_t *otaPartition;

size_t convertBufferToSize(const uint8_t *buffer, size_t bufferSize)
{
  size_t result = 0;
  size_t byteCount = 0;

  // Start from index 2 to exclude the first two elements
  for (size_t i = 2; i < bufferSize; i++)
  {
    result |= static_cast<size_t>(buffer[i]) << (8 * byteCount);
    byteCount++;
  }

  return result;
}

void listPartiton(int offset = 0)
{
  esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, NULL);
  esp_partition_t *pPart;
  while (it != nullptr)
  {
    pPart = (esp_partition_t *)esp_partition_get(it);
    printf("main: partition type = %d.\n", pPart->type);
    printf("main: partition subtype = %d.\n", pPart->subtype);
    printf("main: partition starting address = %x.\n", pPart->address);
    printf("main: partition size = %x.\n", pPart->size);
    printf("main: partition label = %s.\n", pPart->label);
    printf("\n");
    uint8_t read_data[253];
    esp_err_t ret = esp_partition_read(pPart, offset * 253, (void *)read_data, 253);
    Serial.println(ret);
    for (int i = 0; i < sizeof(read_data); i++)
    {
      Serial.print(read_data[i], HEX); // HEX will print the values in hexadecimal format
      Serial.print(", ");              // Add a space between values for readability
    }

    Serial.println();
    it = esp_partition_next(it);
  }
}

void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ;

  LoRa.setPins(csPin, resetPin, irqPin);
  Serial.println("LoRa Receiver v2");

  if (!LoRa.begin(BAND))
  {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }
  Serial.println("Init.");
  // otaPartition = esp_partition_find_first(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, "otadata");
  listPartiton();
}

void loop()
{
  if (int packetSize = LoRa.parsePacket())
  {
    uint8_t chunk[packetSize];
    size_t bytesRead = 0;
    bytesRead = LoRa.readBytes(chunk, packetSize);

    // Read the message number from the chunk
    uint16_t messageNumber = (chunk[1] << 8) | chunk[0];

    if (messageNumber == expectedMessageNumber)
    {

      if (messageNumber == 0)
      {
        size_t bufferSize = sizeof(chunk) / sizeof(chunk[0]);
        updateSize = convertBufferToSize(chunk, bufferSize);
        Serial.println(updateSize);
        // Update.begin(updateSize,0,-1, 0U, "UPDATE");
        Update.begin();
        Update.printError(Serial);
        Serial.println("Start");
      }
      else
      {
        // Remove the message number from the buffer
        bytesRead -= 2;
        totalBytesRead += bytesRead;
        if (chunk[2] == 0xE9)
        {
          Serial.println(chunk[2]);
        }
        Update.write(chunk + 2, bytesRead); // Skip the first two bytes (message number)
        // Serial.print("messageNumber: ");
        // Serial.println(messageNumber);
        Serial.print("Percent: ");
        Serial.println(((float)totalBytesRead / (float)updateSize) * 100);

        if (messageNumber == 20 || messageNumber == 40)
        {
          Serial.println("20*253 offset bytes:");
          listPartiton(20);
        }
      }

      // Increment the expected message number
      expectedMessageNumber++;
    }

    // Send acknowledgment with the message number
    LoRa.beginPacket();
    LoRa.write((uint8_t *)&messageNumber, sizeof(messageNumber)); // Include message number
    LoRa.endPacket();
  }

  if (totalBytesRead == updateSize && !updateEnd)
  {
    delay(1000);
    updateEnd = true;
    Serial.println("END");
    listPartiton();
    if (Update.end(true))
    { // true to set the size to the current progress
      Serial.printf("Update Success! \nRebooting...\n");
      ESP.restart();
    }
    else
    {
      Update.printError(Serial);
    }
  }
}

/*
#include <SPI.h>
#include <LoRa.h>
#include "SPIFFS.h"

#define LORA_CHUNK_SIZE 253
#define ACK_TIMEOUT 1000
#define MAX_RETRY_COUNT 3

const int loraCS = 18;
const int loraRST = 14;
const int loraDI0 = 26;
const int loraBAND = 868E6;

bool once = true;

// Method to send a message and wait for acknowledgment
bool sendMessageAndWaitForAck(uint16_t messageNumber, const uint8_t* data, size_t dataSize)
{
  bool ackReceived = false; // Track if an acknowledgment is received
  uint8_t retryCount = 0;   // Track the number of retries

  while (!ackReceived && retryCount < MAX_RETRY_COUNT)
  {
    // Send the message over LoRa with message number
    LoRa.beginPacket();
    LoRa.write((uint8_t *)&messageNumber, sizeof(messageNumber)); // Include message number
    LoRa.write(data, dataSize);
    LoRa.endPacket();

    // Wait for acknowledgment
    unsigned long startTime = millis();
    while (millis() - startTime < ACK_TIMEOUT)
    {
      if (int packetSize = LoRa.parsePacket())
      {
        uint8_t chunk[packetSize];
        LoRa.readBytes(chunk, packetSize);
        uint16_t received = (chunk[1] << 8) | chunk[0];
        Serial.println(received);
        if (received == messageNumber)
        {
          ackReceived = true;
          break;
        }
      }
    }

    // Increment retry count if no acknowledgment is received
    if (!ackReceived)
    {
      retryCount++;
      Serial.print("Retry ");
      Serial.println(retryCount);
    }

    delay(100); // Adjust delay as needed for your LoRa module
  }

  return ackReceived;
}

// Method to send the firmware update
void sendFirmwareUpdate()
{
  // Open the binary file from SPIFFS
  File firmwareFile = SPIFFS.open("/firmware.bin", "r");
  if (!firmwareFile)
  {
    Serial.println("Failed to open firmware file");
    return;
  }

  uint16_t messageNumber = 0; // Initialize message number
  // Get the size of the firmware file
  size_t firmwareSize = firmwareFile.size();

  // Send the first message with firmware size
  if (!sendMessageAndWaitForAck(messageNumber, (uint8_t*)&firmwareSize, sizeof(firmwareSize)))
  {
    Serial.println("No acknowledgment received for the first message. Aborting firmware transfer.");
    firmwareFile.close();
    return;
  }

  messageNumber++; // Increment message number for the next chunk

  // Read and send the file in chunks
  uint8_t chunk[LORA_CHUNK_SIZE];
  size_t bytesRead = 0;
  size_t totalBytesSent = 0;

  while ((bytesRead = firmwareFile.read(chunk, LORA_CHUNK_SIZE)) > 0)
  {
    if (sendMessageAndWaitForAck(messageNumber, chunk, bytesRead))
    {
      totalBytesSent += bytesRead;
      messageNumber++; // Increment message number for the next chunk

      // Print progress
      Serial.print("Sent ");
      Serial.print(totalBytesSent);
      Serial.print(" bytes of firmware");
    }
    else
    {
      // Handle the case when no ack is received after maximum retries
      Serial.println("No acknowledgment received. Resending the message...");
      // Reset file pointer to resend the same chunk
      firmwareFile.seek(firmwareFile.position() - bytesRead);
    }
  }

  // Close the firmware file
  firmwareFile.close();

  Serial.println("Firmware transfer complete");
}


void setup()
{
  SPIFFS.begin();
  Serial.begin(9600);

  LoRa.setPins(loraCS, loraRST, loraDI0);
  if (!LoRa.begin(loraBAND))
  {
    Serial.println("LoRa initialization failed");
    while (1)
      ;
  }
  Serial.println("LoRa initialized");
}

void loop()
{
  if (once)
  {
    once = false;
    sendFirmwareUpdate();
  }
  if (LoRa.parsePacket())
  {
    Serial.print("Received packet ");

    int reader = LoRa.read();
    byte *res = new byte[255];
    int counter = 0;
    while (reader != -1)
    {
      res[counter] = (byte)reader;
      reader = LoRa.read();
      counter++;
    }

    if (strncmp((char *)res, "START", 5) == 0)
    {
      Serial.println("START");
      sendFirmwareUpdate();
    }
  }
}*/