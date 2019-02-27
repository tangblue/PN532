#include <SPI.h>
#include <PN532_SPI.h>
#include <PN532Interface.h>
#include <PN532.h>

PN532_SPI pn532spi(SPI, 10);
PN532 nfc(pn532spi);

const uint8_t PPSE_APDU_SELECT[] = {
        (byte) 0x00, // CLA (class of command)
        (byte) 0xA4, // INS (instruction); A4 = select
        (byte) 0x04, // P1  (parameter 1)  (0x04: select by name)
        (byte) 0x00, // P2  (parameter 2)
        (byte) 0x0E, // LC  (length of data)  14 (0x0E) = length("2PAY.SYS.DDF01")
        // 2PAY.SYS.DDF01 (ASCII values of characters used):
        // This value requests the card or payment device to list the application
        // identifiers (AIDs) it supports in the response:
        '2', 'P', 'A', 'Y', '.', 'S', 'Y', 'S', '.', 'D', 'D', 'F', '0', '1',
        (byte) 0x00 // LE   (max length of expected result, 0 implies 256)
};

const uint8_t PPSE_APDU_SELECT_RESPONSE[] = {
        (byte) 0x6F,  // FCI Template
        (byte) 0x23,  // length = 35
        (byte) 0x84,  // DF Name
        (byte) 0x0E,  // length("2PAY.SYS.DDF01")
        // Data (ASCII values of characters used):
        '2', 'P', 'A', 'Y', '.', 'S', 'Y', 'S', '.', 'D', 'D', 'F', '0', '1',
        (byte) 0xA5, // FCI Proprietary Template
        (byte) 0x11, // length = 17
        (byte) 0xBF, // FCI Issuer Discretionary Data
        (byte) 0x0C, // length = 12
        (byte) 0x0E,
        (byte) 0x61, // Directory Entry
        (byte) 0x0C, // Entry length = 12
        (byte) 0x4F, // ADF Name
        (byte) 0x07, // ADF Length = 7
        // Tell the POS (point of sale terminal) that we support the standard
        // Visa credit or debit applet: A0000000031010
        // Visa's RID (Registered application provider IDentifier) is 5 bytes:
        (byte) 0xA0, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x03,
        // PIX (Proprietary application Identifier eXtension) is the last 2 bytes.
        // 10 10 (means visa credit or debit)
        (byte) 0x10, (byte) 0x10,
        (byte) 0x87,  // Application Priority Indicator
        (byte) 0x01,  // length = 1
        (byte) 0x01,
        (byte) 0x90, // SW1  (90 00 = Success)
        (byte) 0x00  // SW2
};

/*
 *  MSD (Magnetic Stripe Data)
 */
const uint8_t VISA_MSD_SELECT[] = {
        (byte) 0x00,  // CLA
        (byte) 0xa4,  // INS
        (byte) 0x04,  // P1
        (byte) 0x00,  // P2
        (byte) 0x07,  // LC (data length = 7)
        // POS is selecting the AID (Visa debit or credit) that we specified in the PPSE
        // response:
        (byte) 0xA0, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x03, (byte) 0x10, (byte) 0x10,
        (byte) 0x00   // LE
};


const uint8_t VISA_MSD_SELECT_RESPONSE[] = {
        (byte) 0x6F,  // File Control Information (FCI) Template
        (byte) 0x1E,  // length = 30 (0x1E)
        (byte) 0x84,  // Dedicated File (DF) Name
        (byte) 0x07,  // DF length = 7

        // A0000000031010  (Visa debit or credit AID)
        (byte) 0xA0, (byte) 0x00, (byte) 0x00, (byte) 0x00, (byte) 0x03, (byte) 0x10, (byte) 0x10,

        (byte) 0xA5,  // File Control Information (FCI) Proprietary Template
        (byte) 0x13,  // length = 19 (0x13)
        (byte) 0x50,  // Application Label
        (byte) 0x0B,  // length
        'V', 'I', 'S', 'A', ' ', 'C', 'R', 'E', 'D', 'I', 'T',
        (byte) 0x9F, (byte) 0x38,  // Processing Options Data Object List (PDOL)
        (byte) 0x03,  // length
        (byte) 0x9F, (byte) 0x66, (byte) 0x02, // PDOL value (Does this request terminal type?)
        (byte) 0x90,  // SW1
        (byte) 0x00   // SW2
};


/*
 *  GPO (Get Processing Options) command
 */
const uint8_t GPO_COMMAND[] = {
        (byte) 0x80,  // CLA
        (byte) 0xA8,  // INS
        (byte) 0x00,  // P1
        (byte) 0x00,  // P2
        (byte) 0x05,  // LC (length)
        // data
        (byte) 0x04,
        (byte) 0x83,  // tag
        (byte) 0x02,  // length
        (byte) 0x00,    //  { These 2 bytes can vary, so we'll only        }
        (byte) 0x00,    //  { compare the header of this GPO command below }
        (byte) 0x00   // Le
};


/*
 *  SwipeYours only emulates Visa MSD, so our response is not dependant on the GPO command
 *  data.
 */
const uint8_t GPO_COMMAND_RESPONSE[] = {
        (byte) 0x80,
        (byte) 0x06,  // length
        (byte) 0x00,
        (byte) 0x80,
        (byte) 0x08,
        (byte) 0x01,
        (byte) 0x01,
        (byte) 0x00,
        (byte) 0x90,  // SW1
        (byte) 0x00   // SW2
};


const uint8_t READ_REC_COMMAND[] = {
        (byte) 0x00,  // CLA
        (byte) 0xB2,  // INS
        (byte) 0x01,  // P1
        (byte) 0x0C,  // P2
        (byte) 0x00   // length
};

void setup()
{
    Serial.begin(115200);
    Serial.println("-------Peer to Peer HCE--------");

    nfc.begin();

    uint32_t versiondata = nfc.getFirmwareVersion();
    if (! versiondata) {
      Serial.print("Didn't find PN53x board");
      while (1); // halt
    }

    // Got ok data, print it out!
    Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX);
    Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC);
    Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);

    // Set the max number of retry attempts to read from a card
    // This prevents us from waiting forever for a card, which is
    // the default behaviour of the PN532.
    //nfc.setPassiveActivationRetries(0xFF);

    // configure board to read RFID tags
    nfc.SAMConfig();
}

void loop()
{
  bool success;

  Serial.println("Waiting for an ISO14443A card");
  success = nfc.inListPassiveTarget();
  if(!success) {
    Serial.println("Didn't find anything!");
    delay(1000);

    return;
  }

  Serial.println("Found something!");

  success = exchangeAPDU("PPSE_APDU_SELECT",
                         PPSE_APDU_SELECT, sizeof(PPSE_APDU_SELECT),
                         PPSE_APDU_SELECT_RESPONSE, sizeof(PPSE_APDU_SELECT_RESPONSE));
  if (!success) {
    return;
  }

  success = exchangeAPDU("VISA_MSD_SELECT",
                         VISA_MSD_SELECT, sizeof(VISA_MSD_SELECT),
                         VISA_MSD_SELECT_RESPONSE, sizeof(VISA_MSD_SELECT_RESPONSE));
  if (!success) {
    return;
  }

  success = exchangeAPDU("GPO",
                         GPO_COMMAND, sizeof(GPO_COMMAND),
                         GPO_COMMAND_RESPONSE, sizeof(GPO_COMMAND_RESPONSE));
  if (!success) {
    return;
  }

  success = exchangeAPDU("READ_REC",
                         READ_REC_COMMAND, sizeof(READ_REC_COMMAND),
                         NULL, 0);
  if (!success) {
    return;
  }

  delay(1000);
}

bool exchangeAPDU(const char *desc, const uint8_t *req, uint8_t reqSize, const uint8_t *resp, uint8_t respSize)
{
  uint8_t response[255];
  uint8_t responseLength = sizeof(response);

  Serial.print(desc); Serial.println(": ->");
  nfc.PrintHexChar(req, reqSize);

  if(!nfc.inDataExchange(req, reqSize, response, &responseLength)) {
    Serial.println("Broken connection?");
    return false;
  }

  Serial.print("<- responseLength: "); Serial.println(responseLength);
  nfc.PrintHexChar(response, responseLength);
  if (resp && memcmp(response, resp, respSize)) {
    Serial.print("Response Error: "); Serial.println(desc);
    return false;
  }

  return true;
}
