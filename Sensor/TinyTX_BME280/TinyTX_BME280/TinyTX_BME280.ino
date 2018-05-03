#define LED 8
int zeit;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <JeeLib.h> // https://github.com/jcw/jeelib
#include <PortsBME280.h> // Part of JeeLib

ISR(WDT_vect) { Sleepy::watchdogEvent(); } // interrupt handler for JeeLabs Sleepy power saving

//#define myNodeID 1        // RF12 node ID in the range 1-30
//#define network 210       // RF12 Network group
//#define freq RF12_433MHZ  // Frequency of RFM12B module
//
//										   //#define USE_ACK           // Enable ACKs, comment out to disable
//#define RETRY_PERIOD 5    // How soon to retry (in seconds) if ACK didn't come in
//#define RETRY_LIMIT 5     // Maximum number of times to retry
//#define ACK_TIME 10       // Number of milliseconds to wait for an ack

PortI2C i2c(2);         // BME280 SDA to D8 and SCL to D7
						// PortI2C i2c (1);      // BME280 SDA to D10 and SCL to D9
BME280 psensor(i2c, 3); // ultra high resolution
#define BME280_POWER 9   // BME280 Power pin is connected on D9

						//########################################################################################################################
						//Data Structure to be sent
						//########################################################################################################################

//typedef struct {
//	int16_t temp;	// Temperature reading
//	int supplyV;	// Supply voltage
//	int32_t pres;	// Pressure reading
//} Payload;
//
//Payload tinytx;

//// Wait a few milliseconds for proper ACK
//#ifdef USE_ACK
//static byte waitForAck() {
//	MilliTimer ackTimer;
//	while (!ackTimer.poll(ACK_TIME)) {
//		if (rf12_recvDone() && rf12_crc == 0 &&
//			rf12_hdr == (RF12_HDR_DST | RF12_HDR_CTL | myNodeID))
//			return 1;
//	}
//	return 0;
//}
//#endif

//--------------------------------------------------------------------------------------------------
// Send payload data via RF
//-------------------------------------------------------------------------------------------------
//static void rfwrite() {
//#ifdef USE_ACK
//	for (byte i = 0; i <= RETRY_LIMIT; ++i) {  // tx and wait for ack up to RETRY_LIMIT times
//		rf12_sleep(-1);              // Wake up RF module
//		while (!rf12_canSend())
//			rf12_recvDone();
//		rf12_sendStart(RF12_HDR_ACK, &tinytx, sizeof tinytx);
//		rf12_sendWait(2);           // Wait for RF to finish sending while in standby mode
//		byte acked = waitForAck();  // Wait for ACK
//		rf12_sleep(0);              // Put RF module to sleep
//		if (acked) { return; }      // Return if ACK received
//
//		Sleepy::loseSomeTime(RETRY_PERIOD * 1000);     // If no ack received wait and try again
//	}
//#else
//	rf12_sleep(-1);              // Wake up RF module
//	while (!rf12_canSend())
//		rf12_recvDone();
//	rf12_sendStart(0, &tinytx, sizeof tinytx);
//	rf12_sendWait(2);           // Wait for RF to finish sending while in standby mode
//	rf12_sleep(0);              // Put RF module to sleep
//	return;
//#endif
//}

////--------------------------------------------------------------------------------------------------
//// Read current supply voltage
////--------------------------------------------------------------------------------------------------
//long readVcc() {
//	bitClear(PRR, PRADC); ADCSRA |= bit(ADEN); // Enable the ADC
//	long result;
//	// Read 1.1V reference against Vcc
//#if defined(__AVR_ATtiny84__) 
//	ADMUX = _BV(MUX5) | _BV(MUX0); // For ATtiny84
//#else
//	ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);  // For ATmega328
//#endif 
//	delay(2); // Wait for Vref to settle
//	ADCSRA |= _BV(ADSC); // Convert
//	while (bit_is_set(ADCSRA, ADSC));
//	result = ADCL;
//	result |= ADCH << 8;
//	result = 1126400L / result; // Back-calculate Vcc in mV
//	ADCSRA &= ~bit(ADEN); bitSet(PRR, PRADC); // Disable the ADC to save power
//	return result;
//}
//########################################################################################################################

void setup() {
	//pinMode(LED, OUTPUT);
	pinMode(BME280_POWER, OUTPUT); // set power pin for BME280 to output
	digitalWrite(BME280_POWER, HIGH); // turn BME280 sensor on
	//Sleepy::loseSomeTime(50);
	//psensor.getCalibData();

	//rf12_initialize(myNodeID, freq, network); // Initialize RFM12 with settings defined above 
	//rf12_sleep(0);                          // Put the RFM12 to sleep

	//PRR = bit(PRTIM1); // only keep timer 0 going
	ADCSRA &= ~bit(ADEN); bitSet(PRR, PRADC); // Disable the ADC to save power

}

void loop() {

	//// Get raw temperature reading
	//psensor.startMeas(BME280::TEMP);
	//Sleepy::loseSomeTime(16);
	//int32_t traw = psensor.getResult(BME280::TEMP);

	//// Get raw pressure reading
	//psensor.startMeas(BME280::PRES);
	//Sleepy::loseSomeTime(32);
	//int32_t praw = psensor.getResult(BME280::PRES);

	//// Calculate actual temperature and pressure
	//int32_t press;
	//psensor.calculate(tinytx.temp, press);
	//tinytx.pres = (press * 0.01);

	//tinytx.supplyV = readVcc(); // Get supply voltage

	//rfwrite(); // Send data via RF 

	//Sleepy::loseSomeTime(60000); //JeeLabs power save function: enter low power mode for 60 seconds (valid range 16-65000 ms)

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//void LED_Debug (int zeit) {
//	for (int i = 0; i < 5; i++)
//	{
//	digitalWrite(LED, HIGH);	// turn the LED on (HIGH is the voltage level)
//	delay(zeit);				// wait for a second
//	digitalWrite(LED, LOW);		// turn the LED off by making the voltage LOW
//	delay(zeit);
//	}
//}