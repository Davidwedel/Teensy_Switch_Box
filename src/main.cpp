//Created by David Wedel 3/13/2023
//copied from everywhere

/*todo
	Sections 9-16 aren't working
 */

#include <Arduino.h>
#include <EEPROM.h> 


// libraries
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

#define EEP_Ident 0x5426  
#define AutoSwitch 36 //auto/manual
#define ManuelSwitch 37//on-off
#define valveSwitch A11 //fertilizer valve switch
#define valveRelay 6 //fertilizer valve relay--follows section 5 for now
#define mainRelay 7
//rgb
#define r 10
#define g 12
#define b 11
#define pulsePin 33 //GPS speed pulse

// ethernet
struct ConfigIP {
	uint8_t ipOne = 192;
	uint8_t ipTwo = 168;
	uint8_t ipThree = 0;
};
ConfigIP networkAddress; // 3 bytes

byte Eth_myip[4] = {0, 0, 0, 0}; // this is now set via agio
byte mac[] = {0x00, 0x00, 0x56, 0x00, 0x00, 0x7B};

uint16_t portMy = 5123; // this is port of this module: Autosteer = 5577 IMU =
												// 5566 Section Control = 5555 GPS = 5544
uint16_t PortFromAOG = 8888;     // port to listen for AOG
uint16_t portDestination = 9999; // Port of AOG that listens // ports
uint8_t PGN_237[] = {0x80, 0x81, 0x7f, 237, 8, 1, 2, 3, 4, 0, 0, 0, 0, 0xCC};
int8_t PGN_237_Size = sizeof(PGN_237) - 1;
uint8_t AOG[] = { 0x80, 0x81, 0x7B, 0xEA, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0xCC };//pgn 234
byte Eth_ipDest_ending = 255; // ending of IP address to send UDP data to
IPAddress Eth_ipDestination;

uint8_t helloFromMachine[] = {128, 129, 123, 123, 5, 0, 0, 0, 0, 0, 71};
uint8_t onLo = 0, offLo = 0, onHi = 0, offHi = 0, mainByte = 0;

EthernetUDP EthUDPToAOG;
EthernetUDP EthUDPFromAOG;

uint8_t udpData[UDP_TX_PACKET_MAX_SIZE]; // Buffer For Receiving UDP Data
bool EthUdpRunning = false;

byte CK_A = 0;
byte CK_B = 0;
int relposnedByteCount = 0;
// end ethernet

//config from AOG
struct Config {
	uint8_t raiseTime = 2;
	uint8_t lowerTime = 4;
	uint8_t enableToolLift = 0;
	uint8_t isRelayActiveHigh = 1; // if zero, active low (default)

	uint8_t user1 = 0; // user defined values set in machine tab
	uint8_t user2 = 0;
	uint8_t user3 = 0;
	uint8_t user4 = 0;
};
Config aogConfig; // 4 bytes

/*
 * Functions as below assigned to pins
0: -
1 thru 16: Section 1,Section 2,Section 3,Section 4,Section 5,Section 6,Section
7,Section 8, Section 9, Section 10, Section 11, Section 12, Section 13, Section
14, Section 15, Section 16, 17,18    Hyd Up, Hyd Down, 19 Tramline, 20: Geo Stop
21,22,23 - unused so far*/
// location is pin, number is section
uint8_t pin[] = {24, 3, 4, 5, 6, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// read value from Machine data and set 1 or zero according to list
uint8_t relayState[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
bool isRaise = false, isLower = false;
// The variables used for storage
uint8_t relayHi = 0, relayLo = 0, tramline = 0, uTurn = 0, hydLift = 0,
				geoStop = 0;
float gpsSpeed;
uint8_t raiseTimer = 0, lowerTimer = 0, lastTrigger = 0;

// timing
const uint8_t LOOP_TIME = 100; // 5hz
uint32_t lastTime = LOOP_TIME;
uint32_t currentTime = LOOP_TIME;
uint32_t fifthTime = 0;
uint16_t count = 0;

// Comm checks
uint8_t watchdogTimer = 20;   // make sure we are talking to AOG
uint8_t serialResetTimer = 0; // if serial buffer is getting full, empty it

elapsedMillis speedPulseUpdateTimer = 0;
elapsedMillis gpsSpeedUpdateTimer = 0;

int16_t EEread = 0;

int analogvalue = 0;
int trigger =0;
boolean autoModeIsOn = false;
boolean manuelModeIsOn = false;
boolean firstConnection = true;
boolean secOnMan = false;
const uint8_t switchPinArray[] = {A0, A1, A2, A3, A4, A5, A6, A7}; //Pins, Switch activation sections A5 à A0 et D12  //<-
#define NUM_OF_RELAYS 8 //number of switches

//forward declarations
void SetRelays(void);
void SendUdp(uint8_t *data, uint8_t datalen, IPAddress dip, uint16_t dport);
void receiveUDP();
void setRGBcolor(int red, int green, int blue);

void setup() {
	EEPROM.get(0, EEread);              // read identifier

	if (EEread != EEP_Ident)   // check on first start and write EEPROM
	{
		EEPROM.put(0, EEP_Ident);
		EEPROM.put(6, aogConfig);
		EEPROM.put(20, pin);
		EEPROM.put(50, networkAddress);
	}
	else
	{
		EEPROM.get(6, aogConfig);
		EEPROM.get(20, pin);
		EEPROM.get(50, networkAddress);
	}


	if (Ethernet.linkStatus() == LinkOFF) {
		Serial.println("Ethernet cable is not connected - Who cares we will start "
				"ethernet anyway.");
	}
	// start the Ethernet connection
	Serial.println("Initializing ethernet with static IP address");

	// try to congifure using IP:
	Ethernet.begin(mac, 0); // Start Ethernet with IP 0.0.0.0
													// set ips manually
	Eth_myip[0] = networkAddress.ipOne;
	Eth_myip[1] = networkAddress.ipTwo;
	Eth_myip[2] = networkAddress.ipThree;
	Eth_myip[3] = 123;

	Ethernet.setLocalIP(Eth_myip); // Change IP address to IP set by user
	Serial.println("\r\nEthernet status OK");
	Serial.print("IP set Manually: ");
	Serial.println(Ethernet.localIP());

	Eth_ipDestination[0] = Eth_myip[0];
	Eth_ipDestination[1] = Eth_myip[1];
	Eth_ipDestination[2] = Eth_myip[2];
	Eth_ipDestination[3] = 255;

	Serial.print("\r\nEthernet IP of module: ");
	Serial.println(Ethernet.localIP());
	Ethernet.begin(mac, 0);
	if (Ethernet.linkStatus() == LinkOFF) {
		Serial.println("Ethernet cable is not connected.");
	}
	for (byte n = 0; n < 3; n++) {
		Eth_ipDestination[n] = Eth_myip[n];
	}
	Eth_ipDestination[3] = Eth_ipDest_ending;
	Ethernet.setLocalIP(Eth_myip);
	Serial.print("Ethernet IP of Section Control module: ");
	Serial.println(Ethernet.localIP());
	Serial.print("Ethernet sending to IP: ");
	Serial.println(Eth_ipDestination);
	// init UPD Port sending to AOG
	if (EthUDPToAOG.begin(portMy)) {
		Serial.print("Ethernet UDP sending from port: ");
		Serial.println(portMy);
	}
	if (EthUDPFromAOG.begin(PortFromAOG)) {
		Serial.print("Ethernet UDP listening to port: ");
		Serial.println(PortFromAOG);
		EthUdpRunning = true;
	}
	//end ethernet

	pinMode(24, OUTPUT);//relay pins
	pinMode(3, OUTPUT);
	pinMode(4, OUTPUT);
	pinMode(5, OUTPUT);
	pinMode(6, OUTPUT);
	pinMode(7, OUTPUT);
	//rgb led
	pinMode(r, OUTPUT);
	pinMode(g, OUTPUT);
	pinMode(b, OUTPUT);
	pinMode(AutoSwitch, INPUT_PULLUP);  
	pinMode(ManuelSwitch, INPUT_PULLUP);
	pinMode(valveSwitch, INPUT_PULLUP);

	for (count = 0; count < NUM_OF_RELAYS; count++) {//switch pins
		pinMode(switchPinArray[count], INPUT_PULLUP);
	}
} // end setup

void loop() {
	currentTime = millis();

	if (currentTime - lastTime >= LOOP_TIME) {
		lastTime = currentTime;
		/*								Serial.println("");
											Serial.print("etl	");
											Serial.print("wdt:");
											Serial.print(watchdogTimer);*/

		// If connection lost to AgOpenGPS, the watchdog will count up
		if (watchdogTimer++ > 250)
			watchdogTimer = 20;

		// clean out serial buffer to prevent buffer overflow
		if (serialResetTimer++ > 20) {
			while (Serial.available() > 0)
				Serial.read();
			serialResetTimer = 0;
		}

		if (watchdogTimer > 20) {
			if (aogConfig.isRelayActiveHigh) {
				relayLo = 255;
				relayHi = 255;
			} else {
				relayLo = 0;
				relayHi = 0;
			}
		}

		// hydraulic lift

		if (hydLift != lastTrigger && (hydLift == 1 || hydLift == 2)) {
			lastTrigger = hydLift;
			lowerTimer = 0;
			raiseTimer = 0;

			// 200 msec per frame so 5 per second
			switch (hydLift) {
				// lower
				case 1:
					lowerTimer = aogConfig.lowerTime * 5;
					break;

					// raise
				case 2:
					raiseTimer = aogConfig.raiseTime * 5;
					break;
			}
		}

		// countdown if not zero, make sure up only
		if (raiseTimer) {
			raiseTimer--;
			lowerTimer = 0;
		}
		if (lowerTimer)
			lowerTimer--;

		// if anything wrong, shut off hydraulics, reset last
		if ((hydLift != 1 && hydLift != 2) || watchdogTimer > 10) //|| gpsSpeed < 2)
		{
			lowerTimer = 0;
			raiseTimer = 0;
			if ((aogConfig.lowerTime >= 255 ) || (aogConfig.raiseTime >= 255 )) lastTrigger = 0;  
		}

		if (aogConfig.isRelayActiveHigh) {
			isLower = isRaise = false;
			if (lowerTimer)
				isLower = true;
			if (raiseTimer)
				isRaise = true;
		} else {
			isLower = isRaise = true;
			if (lowerTimer)
				isLower = false;
			if (raiseTimer)
				isRaise = false;
		}

		// section relays
		if (aogConfig.user2 == 0){
			SetRelays();
		}

		// checksum
		int16_t CK_A = 0;
		for (uint8_t i = 2; i < PGN_237_Size; i++) {
			CK_A = (CK_A + PGN_237[i]);
		}
		PGN_237[PGN_237_Size] = CK_A;

		// off to AOG
		SendUdp(PGN_237, sizeof(PGN_237), Eth_ipDestination, portDestination);

		//fertilizer valve
		if(aogConfig.user1 > 0){
			analogvalue = analogRead(valveSwitch);
			//Serial.print("analogvalue of valve switch: ");
			//Serial.println(analogvalue);
			if((analogvalue<900)&&(analogvalue>100)){//off
				digitalWrite(6, LOW);
				setRGBcolor(35, 0, 0);
			}else if(analogvalue<300){//on manual
				digitalWrite(6,HIGH);
				setRGBcolor(55, 35, 0);
			}else{//auto
				if(relayLo>0){
					digitalWrite(6, HIGH);
					setRGBcolor(0, 35, 0);
				}else{//do nothing
					digitalWrite(6, LOW);
					setRGBcolor(35, 0, 0);
				}
			}
		}else{
			setRGBcolor(0, 0, 35);
		}
		//check mainswitch if Auto/Manuel:
		autoModeIsOn = !digitalRead(AutoSwitch); //Switch has to close for autoModeOn, Switch closes ==> LOW state ==> ! makes it to true
		manuelModeIsOn = !digitalRead(ManuelSwitch);
		//built in safety
		if(!manuelModeIsOn){
			digitalWrite(mainRelay, HIGH);
		}else{
			digitalWrite(mainRelay, LOW);
		}
		if (autoModeIsOn) {
			mainByte = 1;
		} else {
			mainByte = 2;
			if (!manuelModeIsOn) firstConnection = false;
		}

		if (!autoModeIsOn) {
			if(manuelModeIsOn && !firstConnection) { //Mode Manuel
				for (count = 0; count < NUM_OF_RELAYS; count++) {
					analogvalue = analogRead(switchPinArray[count]);
					if((analogvalue> 900)||(analogvalue< 300)) { //Signal LOW ==> switch is closed
						if (count < 8) {
							bitClear(offLo, count);
							bitSet(onLo, count);
						} else {
							bitClear(offHi, count-8);
							bitSet(onHi, count-8);
						}
					} else {
						if (count < 8) {
							bitSet(offLo, count);
							bitClear(onLo, count);
						} else {
							bitSet(offHi, count-8);
							bitClear(onHi, count-8);
						}
					}
				}
			} else { //Mode off
				onLo = onHi = 0;
				offLo = offHi = 0b11111111;
			}
		} else if (!firstConnection) { //Mode Auto
																	 //        onLo = onHi = 0b00000000;
																	 //        offLo = offHi = 0b00000000;
			for (count = 0; count < NUM_OF_RELAYS; count++) {
				analogvalue = analogRead(switchPinArray[count]);
				if ((analogvalue<900)&&(analogvalue>100)) {//to shut off the section
					if (count < 8) {
						bitSet(offLo, count); //Info for AOG switch OFF
						bitClear(onLo, count);
					} else {
						bitSet(offHi, count-8); //Info for AOG switch OFF
						bitClear(onHi, count-8);

					}
				} else if(analogvalue<300){//turn section on manual
					if (count < 8) {
						bitSet(onLo, count);
						bitClear(offLo, count); 
					} else {
						bitSet(onHi, count-8); 
						bitClear(offHi, count-8);
					}
				} else { //Signal LOW ==> switch is closed middle position
					Serial.print("bitread =");
					Serial.println(bitRead(onLo, count));
					if(bitRead(onLo, count)==1){
						mainByte = 2;
					}
					if (count < 8){
						bitClear(onLo, count);
						bitClear(offLo, count);
					} else {
						bitClear(offHi, count-8); 
						bitClear(onHi, count-8);
					}
				}
			}
		} else { //FirstConnection
			onLo = onHi = 0;
			offLo = offHi = 0b11111111;
			mainByte = 2;
		}     
		AOG[5] = (uint8_t)mainByte;
		AOG[9] = (uint8_t)onLo;
		AOG[10] = (uint8_t)offLo;
		AOG[11] = (uint8_t)onHi;
		AOG[12] = (uint8_t)offHi;

		//add the checksum
		int16_t CK_B = 0;
		for (uint8_t v = 2; v < sizeof(AOG)-1; v++)
		{
			CK_B = (CK_B + AOG[v]);
		}
		AOG[sizeof(AOG)-1] = CK_B;

		SendUdp(AOG, sizeof(AOG), Eth_ipDestination, portDestination);
	} // end of timed loop
	receiveUDP();
	// Speed pulse
	//	Serial.print("updater");
	//Serial.println(gpsSpeedUpdateTimer);
	if (gpsSpeedUpdateTimer < 1000)
	{
		if (speedPulseUpdateTimer > 200) // 100 (10hz) seems to cause tone lock ups occasionally
		{
			speedPulseUpdateTimer = 0;
			Serial.println("running");

			//130 pp meter, 3.6 kmh = 1 m/sec = 130hz or gpsSpeed * 130/3.6 or gpsSpeed * 36.1111
			//gpsSpeed = ((float)(autoSteerUdpData[5] | autoSteerUdpData[6] << 8)) * 0.1;
			float speedPulse = gpsSpeed * 3.61111;

			Serial.print(gpsSpeed); Serial.print(" -> "); Serial.println(speedPulse);

			if (gpsSpeed > 0.11) { // 0.10 wasn't high enough
				tone(pulsePin, uint16_t(speedPulse));
			}
			else {
				noTone(pulsePin);
				//Serial.println("below .1");
			}
		}
	}
	else  // if gpsSpeedUpdateTimer hasn't update for 1000 ms, turn off speed pulse
	{
		noTone(pulsePin);
		Serial.println("old signal");
	}



} // end of void loop()

void SetRelays(void) {
	// pin, rate, duration  130 pp meter, 3.6 kmh = 1 m/sec or gpsSpeed * 130/3.6
	// or gpsSpeed * 36.1111 gpsSpeed is 10x actual speed so 3.61111
	//gpsSpeed *= 3.61111;
	// tone(13, gpsSpeed);

	// Load the current pgn relay state - Sections
	for (uint8_t i = 0; i < 8; i++) {
		relayState[i] = bitRead(relayLo, i);
	}

	for (uint8_t i = 0; i < 8; i++) {
		relayState[i + 8] = bitRead(relayHi, i);
	}

	// Hydraulics
	relayState[16] = isLower;
	relayState[17] = isRaise;

	// Tram
	relayState[18] = bitRead(tramline, 0); // right
	relayState[19] = bitRead(tramline, 1); // left

	// GeoStop
	relayState[20] = (geoStop == 0) ? 0 : 1;

	if (pin[0])
		digitalWrite(24, relayState[pin[0] - 1]);
	if (pin[1])
		digitalWrite(3, relayState[pin[1] - 1]);
	if (pin[2])
		digitalWrite(4, relayState[pin[2] - 1]);
	if (pin[3])
		digitalWrite(5, relayState[pin[3] - 1]);

	if (pin[4])
		digitalWrite(6, relayState[pin[4] - 1]);
	/*if (pin[5])
		digitalWrite(7, relayState[pin[5] - 1]);

		if (pin[6])
		digitalWrite(8, relayState[pin[6] - 1]);
		if (pin[7])
		digitalWrite(9, relayState[pin[7] - 1]);

		if (pin[8])
		digitalWrite(12, relayState[pin[8] - 1]);
		if (pin[9])
		digitalWrite(4, relayState[pin[9] - 1]);

		if (pin[10]) digitalWrite(IO#Here, relayState[pin[10]-1]);
		if (pin[11]) digitalWrite(IO#Here, relayState[pin[11]-1]);
		if (pin[12]) digitalWrite(IO#Here, relayState[pin[12]-1]);
		if (pin[13]) digitalWrite(IO#Here, relayState[pin[13]-1]);
		if (pin[14]) digitalWrite(IO#Here, relayState[pin[14]-1]);
		if (pin[15]) digitalWrite(IO#Here, relayState[pin[15]-1]);*/
	// if (pin[16]) digitalWrite(2, relayState[pin[16]-1]);
	// if (pin[17]) digitalWrite(17, relayState[pin[17]-1]);
	//  if (pin[18]) digitalWrite(18, relayState[pin[18]-1]);
	// if (pin[19]) digitalWrite(IO#Here, relayState[pin[19]-1]);
}

void SendUdp(uint8_t *data, uint8_t datalen, IPAddress dip, uint16_t dport)
{   
	EthUDPToAOG.beginPacket(dip, dport);
	EthUDPToAOG.write(data, datalen);
	EthUDPToAOG.endPacket();
	//Serial.print("S-UDP ");
}

void setRGBcolor(int red, int green, int blue)
{
	analogWrite(r, red);
	analogWrite(g, green);
	analogWrite(b, blue);
}

void receiveUDP()
{
	uint16_t len = EthUDPFromAOG.parsePacket();
	if (len > 4) {
		EthUDPFromAOG.read(udpData, UDP_TX_PACKET_MAX_SIZE);

		if (udpData[0] == 0x80 && udpData[1] == 0x81 && udpData[2] == 0x7F) // Data
		{

			/*							for (int16_t i = 0; i < len; i++) {
											Serial.print(udpData[i], HEX);
											Serial.print("  ");
											}
			 */
			if (udpData[3] == 239) // machine data
			{
				Serial.println("machine");
				//																Serial.print("pgn==");
				//															Serial.print(udpData[3]);
				//														Serial.print("	machine data	");
				uTurn = udpData[5];
				//gpsSpeed = ((float)(udpData[5] | udpData[6] << 8)) * 0.1;
				//gpsSpeed = ((float)(udpData[6] << 8)) * 0.1;
				//gpsSpeed = ((float)(udpData[6] << 8)) / 10;
				gpsSpeed = (float)udpData[6];
				Serial.println((float)udpData[6]);
				gpsSpeedUpdateTimer = 0;


				hydLift = udpData[7];
				tramline = udpData[8]; // bit 0 is right bit 1 is left

				relayLo = udpData[11]; // read relay control from AgOpenGPS
				relayHi = udpData[12];

				if (aogConfig.isRelayActiveHigh) {
					tramline = 255 - tramline;
					relayLo = 255 - relayLo;
					relayHi = 255 - relayHi;
				}

				// Bit 13 CRC

				// reset watchdog
				watchdogTimer = 0;
			}

			else if (udpData[3] == 200) // Hello from AgIO
			{
				if (udpData[7] == 1) {
					relayLo -= 255;
					relayHi -= 255;
					watchdogTimer = 0;
				}

				helloFromMachine[5] = relayLo;
				helloFromMachine[6] = relayHi;

				//													Serial.print("hello from agio");

				SendUdp(helloFromMachine, sizeof(helloFromMachine), Eth_ipDestination,
						portDestination);
				delay(50);
			}

			else if (udpData[3] == 238) {
				aogConfig.raiseTime = udpData[5];
				aogConfig.lowerTime = udpData[6];
				aogConfig.enableToolLift = udpData[7];

				// set1
				uint8_t sett = udpData[8]; // setting0
				if (bitRead(sett, 0))
					aogConfig.isRelayActiveHigh = 1;
				else
					aogConfig.isRelayActiveHigh = 0;

				aogConfig.user1 = udpData[9];
				aogConfig.user2 = udpData[10];
				aogConfig.user3 = udpData[11];
				aogConfig.user4 = udpData[12];

				// crc

				// save in EEPROM and restart
				EEPROM.put(6, aogConfig);
				//         resetFunc();
			}

			else if (udpData[3] == 201) {
				// make really sure this is the subnet pgn
				if (udpData[4] == 5 && udpData[5] == 201 && udpData[6] == 201) {
					networkAddress.ipOne = udpData[7];
					networkAddress.ipTwo = udpData[8];
					networkAddress.ipThree = udpData[9];

					// save in EEPROM and restart
					EEPROM.put(50, networkAddress);
					//         resetFunc();
				}

			}

			// Scan Reply
			else if (udpData[3] == 202) {
				// make really sure this is the subnet pgn
				if (udpData[4] == 3 && udpData[5] == 202 && udpData[6] == 202) {
					uint8_t scanReply[] = {
						128,
						129,
						123,
						203,
						7,
						networkAddress.ipOne,
						networkAddress.ipTwo,
						networkAddress.ipThree,
						123,
						//  src_ip[0], src_ip[1], src_ip[2], 23
					};

					// checksum
					int16_t CK_A = 0;
					for (uint8_t i = 2; i < sizeof(scanReply) - 1; i++) {
						CK_A = (CK_A + scanReply[i]);
					}
					scanReply[sizeof(scanReply)] = CK_A;

					static uint8_t ipDest[] = {255, 255, 255, 255};
					uint16_t portDest = 9999; // AOG port that listens

					// off to AOG
					SendUdp(scanReply, sizeof(scanReply), ipDest, portDest);
				}
			}

			else if (udpData[3] == 236) // EC Relay Pin Settings
			{
				//Serial.println("relay settings");
				for (uint8_t i = 0; i < 24; i++) {
					pin[i] = udpData[i + 5];
				}

				//save in EEPROM and restart
				EEPROM.put(20, pin);
			}
		}
	}
}
