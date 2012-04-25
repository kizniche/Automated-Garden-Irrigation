
//  Automated Rain-water Irrigation System
//  Copyright 2012 Kyle Gabriel

//  Basic Features:
//  Push-button rotary encoder for input (A knob that turns indefinitely that can also be pressed as a button)
//  4-digit, 7-segment display used as feedback for menu navigation and status
//  Real Time Clock to measure accurate time and to retain schedules when power is lost
//  Solid state relays control the irrigation pump and the valve for the backup water filling
//  Up to 3 programmable schedules with start time, water duration, duration between watering, and start day
//  A rain sensor is checked every hour and before scheduled watering. If water is detected, the pump stays off.

//  Safety Features:
//  Temperature sensor ensures the garden is not watered when it is too cold or too cold in the near future
//  Float sensor to maintain a minimum water level in the rain barrel; to ensure the pump does not run dry
//  Emergency shutdown if the pressure rises to an unsafe level (ie. blockage in the line)
//  Emergency shutdown if fill valve has been open longer than it normally takes to fill to the float sensor
//  Emergency shutdown if the float sensor activates the fill valve more than 3 minutes after watering has ended

// *-*-*-*-*-* Operation and Navigation *-*-*-*-*-*
// The current time & temperature will alternate on the display by default. If the colon is blinking when the
// time is displayed, the schedule is on. If the colon is not blinking and stays solid, the schedule is turned off.
// While the time & temperature is being displayed. The knob can be pressed to activate the control menu.
// Turn the knob to change a value, then press to confirm the value.

// *-*-*-*-*-* Menu Options *-*-*-*-*-*
// 1: Exit the control menu and return to displaying the current time.
// 2: Display schedule(s) in the format: start time, duration on, days between running, next run day (1=Mon...7=Sun).
// 3: Change schedule
//   A: "#  x"   #: Which schedule, x: How many schedules to use (up to 3).
//   B: "XXYY"   X: Starting hour, Y: Starting minute. Selecting the time will confirm & display the next schedule time. 
//   C: "#  x"   #: Which schedule, x: watering duration in seconds (~5 gallons/minute).
//   D: "#d x"   #: Which schedule, x: how many days between running the schedule. 
//   E: "#n x"   #: Which schedule, x: Which day of the week to start schedule (1=Mon...7=Sun).
// 4: Override the pump (to run the sprinkler) to either remain OFF or ON
// 5: Override the valve (to fill the barrels) to either remain OFF or ON
// 6: Turn the timer schedule OFF or ON
// 7: Reset the pump and valve overrides so they return to a scheduled operation (if schedule is turned on)
// 8: Display temperature history over the past 24 hours, 0 is the most current temperature stored

// -*-*-*-*-* Emergency Modes *-*-*-*-*-*
// If an emergency is detected, the valve and pump will remain off until the button is pressed to resume normal operation.
// During emergency mode, one of the following will interupt anything currently being displayed:
// -E-1: The valve was open > 15 continuous seconds.
//       This indicates the water is not activating the float switch in a timely manner. Ensure the hose going into
//       the tank is connected & clear, the valve is operational, the spigot is open, the tanks are free of leaks,
//       the float switch is free to move, and the switch is indeed activated when the water level reaches the sensor.
// -E-2: The pressure switch began rapidly turning on and off or the pump has run dry.
//       This indicates there is too much pressure building in the pump outlet or no water at the pump inlet.
//       Ensure the outflow hose from the pump is not kinked, the sprinkler screen/heads are clean, and there is
//       an adequate amount of water reaching the pump inlet.
// -E-3: The float switch has been activated after more than 3 minutes from when the end of a scheduled watering.
//       This indicates there may be a leak or water is being manually removed via the barrel's spigot. If the barrels
//       were allowed to fill, which could take over a minute of the valve turning on and off [to equilibriate the water
//       levels], the float switch should not activate this late. If water is being manually taken out, remember to exit
//       emergency mode in order to resume the schedule.

//  Thanks to:
//  Pedro Rodrigues (medecau@gmail.com) for QuadEncoder.h, January 2010 
//  Maurice Ribble (http://www.glacialwanderer.com/hobbyrobotics) for the RTC code

//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.

//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.

//  You should have received a copy of the GNU General Public License
//  along with this program. If not, see <http://www.gnu.org/licenses/>.

int debug = true;

#include <NewSoftSerial.h>       //  Communicate to the 7-Segment Display
#include <math.h>                //  Calculate temperature from the thermistor
#include <QuadEncoder.h>         //  Read the gray code from the rotary encoder
#include <Wire.h>                //  Communicate to the Real Time Clock modeule
#include <EEPROM.h>              //  Store variables in EEPROM (non-volitile memory)
#include <EEPROMAnything.h>      //  Allow saving and loading whole arrays/structures of variables in a single call

#define DS1307_I2C_ADDRESS 0x68  //  RTC I2C address. Hardware: VCC->5v, GND->GND, SDA->analog 4, SCL->analog 5
#define SerInToArdu 2            //  Not used (RTC)
#define SerOutFrmArdu 3          //  Pin that serial data is sent to the 7-segment display
#define pump_RelayPin 9          //  Pin that controls the pump relay
#define valve_RelayPin 7         //  Pin that controls the valve relay
#define buttonPin 2              //  Pin that senses the rotary encoder button
#define floatswitchPin 8         //  Pin that senses the float (reed) switch
#define pressureswitchPin 1      //  Pin that measures the 5v DC (USB adapter) pressure switch signal
#define ThermistorPIN 0          //  Pin that measures the thermistor resistance
#define RainPIN A3               //  Pin that senses rain
#define valvesec 10              //  How many seconds to keep the water fill valve open after PIN 8 is LOW

NewSoftSerial mySerialPort(SerInToArdu,SerOutFrmArdu);  //  Create serial channel for 7-seg display

QuadEncoder qe(5,4); //  Pins for rotary encoder       
int qe1Move = 0;     //  Rotary encoder position returned by QuadEncoder.h
int rotary = 1;      //  Stores the current rotary dial position
int rotaryLast = 1;  //  Stores the last rotary dial position

byte second, minute, hour, dayOfWeek, dayOfMonth, month, year;  // Real Time Clock
//int second=0, minute=1, hour=1, dayOfWeek=7, dayOfMonth, month, year; // uncomment when speeding up time (testing)

static unsigned long relay_timer = 0;           //  Measures the amount of time the pump is on
static unsigned long pressure_timer = 0;        //  Creates a delay before measuring pressure switch state
static unsigned long valve_timer;               //  Prevents valve from turning on prematurely when first powered on
static unsigned long lWaitMillis;               //  Handle millis() rollover for long timing, sets timers
static unsigned long blink_timer[] = {0,1};     //  Handle time/temp display timing and colon blinking

int valve_timer_act = 0; //  Makes sure valve_timer is only set once
int rain;                //  Stores the rain sensor value
int timer;               //  Identifies which schedule is currently running the pump
int counttimes;          //  Counts up to schedule.timesperday for schedule creating
int tempF;               //  Holds temerature in Fahrenheit
int tempOK;              //  Stores whether it is too cold or will be too cold in the near future
int tempHistS;           //  Selects which temperature in history to display when on menu 8
int menu[] = { 8, 1 };   //  Determines display and menu navigation

struct config_t         // Variables read/written to EEPROM
{
    int onoff;            //  schedule on (true) or off (false)
    int pumpOR;           //  Pump Override: supersedes onoff variable, on (true) or off (false)
    int valveOR;          //  Valve Override: supersedes onoff variable, on (true) or off (false)
    int numofschedules;   //  Run schedules 1 (=1), 1 and 2 (=2), or 1 and 2 and 3 (=3)
    int seconds1TOwater;  //  How many seconds to run the pump for schedule 1,2,3
    int seconds2TOwater;
    int seconds3TOwater;
    int time1hour;        //  Start hour (24 hour format)
    int time2hour;
    int time3hour;
    int time1minute;      //  Start minute
    int time2minute;
    int time3minute;
    int days1TOwater;     //  Days between watering
    int days2TOwater;
    int days3TOwater;
    int nextday1TOwater;  //  Which day of the week to initially start watering (1=Mon...7=Sun)
    int nextday2TOwater; 
    int nextday3TOwater;
    int pumptrigger;      //  Determines if a timer has switched the pump on (true) or off (false)
    int tempHist[24];     //  Stores the temperatures from the past 24 hours
    int emergency;        //  0=OK, 1=Valve open >15 sec, 2=pump pressure switch tripping, 3=valve open >3 min after watering
} schedule;

byte decToBcd(byte val) {  // Convert normal decimal numbers to binary coded decimal
    return ((val / 10 * 16) + (val%10));
}

byte bcdToDec(byte val) {  // Convert binary coded decimal to normal decimal numbers
    return ((val / 16 * 10) + (val%16));
}

int avgTemp(int t) {  // Calculates the average temperaure for the past t hours
    EEPROM_readAnything(0, schedule);
    int tempAvg = 0;
    for (int i = 0; i <= t - 1; i++) {
        tempAvg = tempAvg + schedule.tempHist[i];
    }
    tempAvg = tempAvg / t;
    return tempAvg;
}

void setDateDs1307(byte second,        // 0-59
                   byte minute,        // 0-59
                   byte hour,          // 1-23
                   byte dayOfWeek,     // 1-7
                   byte dayOfMonth,    // 1-28/29/30/31
                   byte month,         // 1-12
                   byte year)          // 0-99
{
    Wire.beginTransmission(DS1307_I2C_ADDRESS);
    Wire.send(0);
    Wire.send(decToBcd(second));    // 0 to bit 7 starts the clock
    Wire.send(decToBcd(minute));
    Wire.send(decToBcd(hour));      // If you want 12 hour am/pm you need to set
                                    // bit 6 (also need to change readDateDs1307)
    Wire.send(decToBcd(dayOfWeek));
    Wire.send(decToBcd(dayOfMonth));
    Wire.send(decToBcd(month));
    Wire.send(decToBcd(year));
    Wire.endTransmission();
}

void getDateDs1307() {  // Gets the date and time from the ds1307 RTC
    // Reset the register pointer
    Wire.beginTransmission(DS1307_I2C_ADDRESS);
    Wire.send(0);
    Wire.endTransmission();
    Wire.requestFrom(DS1307_I2C_ADDRESS, 7);
    // A few of these need masks because certain bits are control bits
    second = bcdToDec(Wire.receive() & 0x7f);
    minute = bcdToDec(Wire.receive());
    hour = bcdToDec(Wire.receive() & 0x3f); // Need to change this if 12 hour am/pm
    dayOfWeek = bcdToDec(Wire.receive());
    dayOfMonth = bcdToDec(Wire.receive());
    month = bcdToDec(Wire.receive());
    year = bcdToDec(Wire.receive());
}

double Thermister(int RawADC) { // Read thermistor and calculate temperature
   double Temp;
   Temp = log(((10240000 / RawADC) - 10000));
   Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp ))* Temp );
   Temp = Temp - 273.15;            // Convert Kelvin to Celcius
   Temp = (Temp * 9.0)/ 5.0 + 32.0; // Convert Celcius to Fahrenheit
   return Temp;
}

void rotary_encoder() {  // Read change in rotary encoder or push-button
    EEPROM_readAnything(0, schedule);       // read variables from PROGMEM
    if (qe1Move == '>' || qe1Move == '<') { // Update rotary if turn detected
        if (qe1Move == '>' && rotary <= 8) {
            if (rotary < 8) rotary++;
            else rotary = 1;
        } 
        else if (qe1Move == '<' && rotary >= 1) {
            if (rotary > 1) rotary--;
            else rotary = 8;
        }
    }
    if (rotaryLast != rotary && menu[0] != 8) {  // If rotary changes, update menu
        if (menu[0] == 12) {  // Show temperature history
	        if (qe1Move == '>') {
	            if (tempHistS < 23) tempHistS++;
		        else tempHistS = 0;
	        }
	        else {
	            if (tempHistS > 0) tempHistS--;
		        else tempHistS = 23;
	        }
	    }
        else if (menu[0] == 11) {  // Turn schedule On/Off
	        schedule.onoff = !schedule.onoff;
            EEPROM_writeAnything(0, schedule);
	    }
	    else if (menu[0] == 10) {  // Reverse state of valve
	        schedule.valveOR = !schedule.valveOR;
	        valve_timer = lWaitMillis;
            EEPROM_writeAnything(0, schedule);
	    }
	    else if (menu[0] == 9) {  // Reverse state of pump
	        schedule.pumpOR = !schedule.pumpOR;
            EEPROM_writeAnything(0, schedule);
	    }
	    else if (menu[0] == 8) {  // Schedule change: 5. What day of the week to start schedule (Mon=1...Sun=7)
	        if (counttimes == 1) {
                if (qe1Move == '>') {
	                if (schedule.nextday1TOwater < 7) schedule.nextday1TOwater++;
		            else schedule.nextday1TOwater = 1;
	            }
		        else {
		            if (schedule.nextday1TOwater > 1) schedule.nextday1TOwater--;
		            else schedule.nextday1TOwater = 7;
		        }
	        }
	        if (counttimes == 2) {
                if (qe1Move == '>') {
	                if (schedule.nextday2TOwater < 7) schedule.nextday2TOwater++;
		            else schedule.nextday2TOwater = 1;
	            }
		        else {
		            if (schedule.nextday2TOwater > 1) schedule.nextday2TOwater--;
		            else schedule.nextday2TOwater = 7;
		        }
	        }
	        if (counttimes == 3) {
                if (qe1Move == '>') {
	                if (schedule.nextday3TOwater < 7) schedule.nextday3TOwater++;
		            else schedule.nextday3TOwater = 1;
	            }
		        else {
		            if (schedule.nextday3TOwater > 1) schedule.nextday3TOwater--;
		            else schedule.nextday3TOwater = 7;
		        }
	        }
	        EEPROM_writeAnything(0, schedule);
	    }
	    else if (menu[0] == 7) {  // Schedule change: 4. How many days between running pump?
	        if (counttimes == 1) {
                if (qe1Move == '>') {
		            if (schedule.days1TOwater < 7) schedule.days1TOwater++;
		            else schedule.days1TOwater = 1;
		        }
                else {
		            if (schedule.days1TOwater > 1) schedule.days1TOwater--;
		            else schedule.days1TOwater = 7;
	            }
	        }
	        else if (counttimes == 2) {
	            if (qe1Move == '>') {
		            if (schedule.days2TOwater < 7) schedule.days2TOwater++;
		            else schedule.days2TOwater = 1;
		        }
                else {
		            if (schedule.days2TOwater > 1) schedule.days2TOwater--;
		            else schedule.days2TOwater = 7;
	            }
	        }
	        else if (counttimes == 3) {
	            if (qe1Move == '>') {
		            if (schedule.days3TOwater < 7) schedule.days3TOwater++;
		            else schedule.days3TOwater = 1;
		        }
                else {
		            if (schedule.days3TOwater > 1) schedule.days3TOwater--;
		            else schedule.days3TOwater = 7;
	            }
	        }
            EEPROM_writeAnything(0, schedule);
	    }
        else if (menu[0] == 5) {  // Schedule change: 3. How many seconds to run the pump?
	        if (counttimes == 1) {
	            if (qe1Move == '>') schedule.seconds1TOwater = schedule.seconds1TOwater + 5;
                else if (schedule.seconds1TOwater > 5) schedule.seconds1TOwater = schedule.seconds1TOwater - 5;
	        }
	        else if (counttimes == 2) {
	            if (qe1Move == '>') schedule.seconds2TOwater = schedule.seconds2TOwater + 5;
                else if (schedule.seconds2TOwater > 5) schedule.seconds2TOwater = schedule.seconds2TOwater - 5;
	        }
	        else if (counttimes == 3) {
	            if (qe1Move == '>') schedule.seconds3TOwater = schedule.seconds3TOwater + 5;
                else if (schedule.seconds3TOwater > 5) schedule.seconds3TOwater = schedule.seconds3TOwater - 5;
	        }
            EEPROM_writeAnything(0, schedule);
	    }
        else if (menu[0] == 4) {  // Schedule change: 2. What hour and minute to water?
	        if (counttimes == 1) {
                if (qe1Move == '>') {
                    if (schedule.time1minute == 30) {
                        if (schedule.time1hour == 23) schedule.time1hour = 0;
                        else schedule.time1hour++;
                        schedule.time1minute = 0;
                    }
                    else schedule.time1minute = 30;
                }
                else {
                    if (schedule.time1minute == 30) schedule.time1minute = 0;
                    else {
                        if (schedule.time1hour == 0) schedule.time1hour = 23;
                        else schedule.time1hour--;
                        schedule.time1minute = 30;
                    }
                }
            }
            else if (counttimes == 2) {
                if (qe1Move == '>') {
                    if (schedule.time2minute == 30) {
                        if (schedule.time2hour == 23) schedule.time2hour = 0;
                        else schedule.time2hour++;
                        schedule.time2minute = 0;
                    }
                    else schedule.time2minute = 30;
                }
                else {
                    if (schedule.time2minute == 30) schedule.time2minute = 0;
                    else {
                        if (schedule.time2hour == 0) schedule.time2hour = 23;
                        else schedule.time2hour--;
                        schedule.time2minute = 30;
                    }
                }
            }
	        else if (counttimes == 3) {
                if (qe1Move == '>') {
                    if (schedule.time3minute == 30) {
                        if (schedule.time3hour == 23) schedule.time3hour = 0;
                        else schedule.time3hour++;
                        schedule.time3minute = 0;
                    }
                    else schedule.time3minute = 30;
                }
                else {
                    if (schedule.time3minute == 30) schedule.time3minute = 0;
                    else {
                        if (schedule.time3hour == 0) schedule.time3hour = 23;
                        else schedule.time3hour--;
                        schedule.time3minute = 30;
                    }
                }
            }
            EEPROM_writeAnything(0, schedule);
        } 
        else if (menu[0] == 3) {  // Schedule change: 1. How many schedules of watering?
	        if (qe1Move == '>') {
	            if (schedule.numofschedules < 3) schedule.numofschedules++;
		        else schedule.numofschedules = 1;
	        }
            else if (schedule.numofschedules > 1) schedule.numofschedules--;
	        else schedule.numofschedules = 3;
            EEPROM_writeAnything(0, schedule);
        }
        else if (menu[0] == 2) {  // Display schedule
            menu[0] = 8;
        }
	    else if (menu[0] == 1) {  // Select option from main menu
	        menu[1] = rotary;
	    }
        rotaryLast = rotary;
	    if (debug) status();
    }
    if (digitalRead(buttonPin) == LOW) {  // Buton is pressed
        while (digitalRead(buttonPin) == LOW) { // Wait for button to be released to continue
            delay(1);                             // Prevents weird results if the button is held down
        }
		switch (menu[0]) {
            case 1:
		        switch (menu[1]) {
			        case 8:  // Select to view temperature history
	                    tempHistS = 0;
		                menu[0] = 12;
						break;
	                case 7:  // Reset overrides
                        schedule.pumpOR = 3;
                        schedule.valveOR = 3;
                        EEPROM_writeAnything(0, schedule);
		                rotary = 1;
		                blink_timer[0] = lWaitMillis;
		                menu[0] = 8;
                        break; 
                    case 6:  // Select schedule On/Off, go to default dispay
		                menu[0] = 11;
                        break;
	                case 5:  // Select Valve On/Off
	                    schedule.valveOR = 0;
                        EEPROM_writeAnything(0, schedule);
	                    menu[0] = 10;
                        break;
                    case 4:  // Select Pump On/Off
	                    schedule.pumpOR = 0;
                        EEPROM_writeAnything(0, schedule);
	                    menu[0] = 9;
                        break;
	                case 3:  // Select Change schedule
		                menu[0] = 3;
                        break;
	                case 2:  // Select Display schedule 
	                    menu[0] = 2;
                        break;
                    default:  // Select Resume
	                    blink_timer[0] = lWaitMillis;
                        menu[0] = 8;
                        break;
                }
				break;
	        case 2:  // On display schedule
	            rotary = 1;
	            blink_timer[0] = lWaitMillis;
	            menu[0] = 8;
                break;
            case 3:  // Change schedule: 1. Select how many schedules to water
                counttimes = 1;
	            menu[0] = 4;
                break;
            case 4:  // Change schedule: 2. Select time
                if (counttimes == schedule.numofschedules) {
	                mySerialPort.print("w");  // This plus next line turns off all dots.
                    mySerialPort.print(B00000000,BYTE);
		            counttimes = 1;
		            menu[0] = 5;
	            }
	            else counttimes++;
                break;
	        case 5:  // Change schedule: 3. Select how long to run pump for
	            if (counttimes == schedule.numofschedules) {
	  	            counttimes = 1;
	                menu[0] = 6;
	            }
	            else counttimes++;
                EEPROM_writeAnything(0, schedule);
	            break;
	        case 6:  // Change schedule: 4. Select how many days between running pump
	            if (counttimes == schedule.numofschedules) {
	                counttimes = 1;
	                menu[0] = 7;
		            if (schedule.days1TOwater == 1) {
		                if ((int)hour < schedule.time1hour ||
		                    ((int)hour == schedule.time1hour &&
			                 (int)minute <= schedule.time1minute)) schedule.nextday1TOwater = dayOfWeek;
		                else if (schedule.days1TOwater + dayOfWeek < 8) {
						    schedule.nextday1TOwater = schedule.days1TOwater + dayOfWeek;
					    }
		                else schedule.nextday1TOwater = schedule.days1TOwater + dayOfWeek - 7;
                    } 
		            else if (schedule.days1TOwater + dayOfWeek < 8) schedule.nextday1TOwater = schedule.days1TOwater + dayOfWeek;
		            else schedule.nextday1TOwater = schedule.days1TOwater + dayOfWeek - 7;

		            if (schedule.days2TOwater == 1) {
		                if ((int)hour < schedule.time2hour ||
		                    ((int)hour == schedule.time2hour &&
			                 (int)minute <= schedule.time2minute)) schedule.nextday2TOwater = dayOfWeek;
		                else if (schedule.days2TOwater + dayOfWeek < 8) {
						    schedule.nextday2TOwater = schedule.days2TOwater + dayOfWeek;
						}
		                else schedule.nextday2TOwater = schedule.days2TOwater + dayOfWeek - 7;
                    }
		            else if (schedule.days2TOwater + dayOfWeek < 8) schedule.nextday2TOwater = schedule.days2TOwater + dayOfWeek;
		            else schedule.nextday2TOwater = schedule.days2TOwater + dayOfWeek - 7;

		            if (schedule.days3TOwater == 1) {
		                if ((int)hour < schedule.time3hour ||
		                    ((int)hour == schedule.time3hour &&
			                 (int)minute <= schedule.time3minute)) schedule.nextday3TOwater = dayOfWeek;
		                else if (schedule.days3TOwater + dayOfWeek < 8) {
						    schedule.nextday3TOwater = schedule.days3TOwater + dayOfWeek;
						}
		                else schedule.nextday3TOwater = schedule.days3TOwater + dayOfWeek - 7;
                    }
		            else if (schedule.days3TOwater + dayOfWeek < 8) schedule.nextday3TOwater = schedule.days3TOwater + dayOfWeek;
		            else schedule.nextday3TOwater = schedule.days3TOwater + dayOfWeek - 7;
	            }
	            else counttimes++;
                EEPROM_writeAnything(0, schedule);
				break;
	        case 7:  // Change schedule: 5. Select the starting day of schedule
	            if (counttimes == schedule.numofschedules) {
	                schedule.onoff = 1;
		            blink_timer[0] = lWaitMillis;
	                menu[0] = 8;
		            EEPROM_writeAnything(0, schedule);
				}
	            else counttimes++;
	            break;
            case 8:  // On current time
	            if (schedule.emergency != 0) {
                    schedule.emergency = 0;
                    pressure_timer = 0;
		            valve_timer_act = 0;
                    relay_timer = lWaitMillis;
		            EEPROM_writeAnything(0, schedule);
                }
	            rotary = 1;
	            menu[0] = 1;
	            break;
	        case 9:
	        case 10:
			case 11:
			case 12:  // On change pump, valve, schedule
	            rotary = 1;
	            blink_timer[0] = lWaitMillis;
	            menu[0] = 8;
				break;
	    }
        if (debug) status();
    }
}

void seg_display() { // Update the 7-segment display
    EEPROM_readAnything(0, schedule);
    switch (menu[0]) {
        case 1:  // Main menu
            mySerialPort.print("w");  // This plus next line turns off all dots.
            mySerialPort.print(B00000000,BYTE);
            switch (menu[1]) {
                case 1:
                    mySerialPort.print("1xRE"); // Resume
                    break;
                case 2:
                    mySerialPort.print("2xDI"); // Display schedule
                    break;
                case 3:
                    mySerialPort.print("3xCH"); // Change schedule
                    break;
                case 4:
                    mySerialPort.print("4xPU"); // Pump On/Off override
                    break;
                case 5:
                    mySerialPort.print("5xH0"); // Hose Valve On/Off override
                    break;
                case 6:
                    mySerialPort.print("6xP0"); // Schedule On/Off
                    break;
	            case 7:
	                mySerialPort.print("7xR0"); // Reset overrides
	                break;
	            case 8:
	                mySerialPort.print("8xte"); // Temperature history
	                break;
            }
            break;
        case 2:  // Display schedule
  	        mySerialPort.print("w");  // This plus next line turns colon on
            mySerialPort.print(B00010000,BYTE);
	        if (schedule.time1hour < 10) mySerialPort.print("x");
	        mySerialPort.print(schedule.time1hour);
	        if (schedule.time1minute < 10) mySerialPort.print("0");
	        mySerialPort.print(schedule.time1minute);
	        delay(2000);
	        mySerialPort.print("w");  // This plus next line turns off all dots.
            mySerialPort.print(B00000000,BYTE);
	        mySerialPort.print("t");
	        if (schedule.seconds1TOwater < 100) mySerialPort.print("x");
	        mySerialPort.print(schedule.seconds1TOwater);
	        delay(2000);
	        mySerialPort.print("dbx");
	        mySerialPort.print(schedule.days1TOwater);
	        delay(2000);
	        mySerialPort.print("ndx");
	        mySerialPort.print(schedule.nextday1TOwater);
	        delay(2000);
	        mySerialPort.print("w");  // This plus next line turns colon on
            mySerialPort.print(B00010000,BYTE);
            if (schedule.numofschedules == 2 || schedule.numofschedules == 3) {
	            if (schedule.time2hour < 10) mySerialPort.print("x");
	            mySerialPort.print(schedule.time2hour);
	            if (schedule.time2minute < 10) mySerialPort.print("0");
	            mySerialPort.print(schedule.time2minute);
	            delay(2000);
	            mySerialPort.print("w");  // This plus next line turns off all dots.
                mySerialPort.print(B00000000,BYTE);
	            mySerialPort.print("t");
	            if (schedule.seconds2TOwater < 100) mySerialPort.print("x");
	            mySerialPort.print(schedule.seconds2TOwater);
	            delay(2000);
	            mySerialPort.print("dbx");
	            mySerialPort.print(schedule.days2TOwater);
	            delay(2000);
	            mySerialPort.print("ndx");
	            mySerialPort.print(schedule.nextday2TOwater);
	            delay(2000);
	        }
	        mySerialPort.print("w");  // This plus next line turns colon on
            mySerialPort.print(B00010000,BYTE);
            if (schedule.numofschedules == 3) {
	            if (schedule.time3hour < 10) mySerialPort.print("x");
	            mySerialPort.print(schedule.time3hour);
	            if (schedule.time3minute < 10) mySerialPort.print("0");
	            mySerialPort.print(schedule.time3minute);
	            delay(2000);
	            mySerialPort.print("w");  // This plus next line turns off all dots.
                mySerialPort.print(B00000000,BYTE);
	            mySerialPort.print("t");
	            if (schedule.seconds3TOwater < 100) mySerialPort.print("x");
	            mySerialPort.print(schedule.seconds3TOwater);
	            delay(2000);
	            mySerialPort.print("dbx");
	            mySerialPort.print(schedule.days3TOwater);
	            delay(2000);
	            mySerialPort.print("ndx");
	            mySerialPort.print(schedule.nextday3TOwater);
	            delay(2000);
	            mySerialPort.print("xxxx");
	        }
	        menu[0] = 8;
            break;
        case 3:  // Change schedule: 1. How many schedules to set
            mySerialPort.print("1xx");
            mySerialPort.print(schedule.numofschedules);
            break;
        case 4:  // Change schedule: 2. What times to water
  	        mySerialPort.print("w");  // This plus next line turns colon on
            mySerialPort.print(B00010000,BYTE);
            if (counttimes == 1) {
	            if (schedule.time1hour < 10) mySerialPort.print(0);
	            mySerialPort.print(schedule.time1hour);
	            if (schedule.time1minute < 10) mySerialPort.print(0);
	            mySerialPort.print(schedule.time1minute);
            }
            if (counttimes == 2) {
	            if (schedule.time2hour < 10) mySerialPort.print(0);
	            mySerialPort.print(schedule.time2hour);
	            if (schedule.time2minute < 10) mySerialPort.print(0);
	            mySerialPort.print(schedule.time2minute);
            }
	        if (counttimes == 3) {
	            if (schedule.time3hour < 10) mySerialPort.print(0);
	            mySerialPort.print(schedule.time3hour);
	            if (schedule.time3minute < 10) mySerialPort.print(0);
	            mySerialPort.print(schedule.time3minute);
            }
            break;
        case 5:  // Change schedule: 3. How many seconds to run pump
	        if (counttimes == 1) {
	            mySerialPort.print("1");
	            if (schedule.seconds1TOwater < 100) mySerialPort.print("x");
	            mySerialPort.print(schedule.seconds1TOwater);
	        }
	        else if (counttimes == 2) {
	            mySerialPort.print("2");
	            if (schedule.seconds2TOwater < 100) mySerialPort.print("x");
	            mySerialPort.print(schedule.seconds2TOwater);
	        }
	        else if (counttimes == 3) {
	            mySerialPort.print("3");
  	            if (schedule.seconds3TOwater < 100) mySerialPort.print("x");
	            mySerialPort.print(schedule.seconds3TOwater);
	        }
            break;
        case 6:  // Change schedule: 4. How many days between running particular schedule
	        if (counttimes == 1) {
	            mySerialPort.print("1dx");
	            mySerialPort.print(schedule.days1TOwater);
	        }
	        else if (counttimes == 2) {
	            mySerialPort.print("2dx");
	            mySerialPort.print(schedule.days2TOwater);
	        }
	        else if (counttimes == 3) {
	            mySerialPort.print("3dx");
	            mySerialPort.print(schedule.days3TOwater);
	        }
            break;
        case 7:  // Change schedule: 5. What day of the week to start schedule
            if (counttimes == 1) {
	            mySerialPort.print("1nx");
	            mySerialPort.print(schedule.nextday1TOwater);
	        }
	        else if (counttimes == 2) {
	            mySerialPort.print("2nx");
	            mySerialPort.print(schedule.nextday2TOwater);
	        }
	        else if (counttimes == 3) {
	            mySerialPort.print("3nx");
	            mySerialPort.print(schedule.nextday3TOwater);
	        }
            break;
        case 8:  // Display current time and temperature
            if (schedule.emergency != 0) {
	            if (schedule.emergency == 1) mySerialPort.print("-E-1"); // Emergency 1 shutdown
	            else if (schedule.emergency == 2) mySerialPort.print("-E-2"); // Emergency 2 shutdown
	            else if (schedule.emergency == 3) mySerialPort.print("-E-3"); // Emergency 3 shutdown
            }
	        else {
	            if (lWaitMillis - blink_timer[0] < 5000) { // if seconds is even
	                if ((lWaitMillis - blink_timer[0] + 1000) / 1000 % 2 == 0 && schedule.onoff == 1) {
	                    mySerialPort.print("w");  // This plus next line turns colon off
                        mySerialPort.print(B00000000,BYTE);
	                } else {
	                    mySerialPort.print("w");  // This plus next line turns colon on
                        mySerialPort.print(B00010000,BYTE);
	                }
	                if ((int)hour < 10) mySerialPort.print("x");
                    mySerialPort.print(hour, DEC);
	                if ((int)minute < 10) mySerialPort.print(0);
	                mySerialPort.print(minute, DEC);
	            } 
                else if (blink_timer[1]) {
	                mySerialPort.print("v");  // Reset display module
	                blink_timer[1] = 0;
	            }
	            else if (rain > 1020 &&
				         lWaitMillis - blink_timer[0] >= 5000 &&
						 lWaitMillis - blink_timer[0] <= 9000)
				{
	                mySerialPort.print("w");  // This plus next line turns colon off
                    mySerialPort.print(B00000000,BYTE);
		            mySerialPort.print("RAIn");
	            }
	            else {
                    mySerialPort.print("w");  // This plus next line turns off all dots, apostrophe on
                    mySerialPort.print(B00100000,BYTE);
	                if (tempF < 100 && tempF > 9) mySerialPort.print("x");
	                if (tempF < 10 && tempF > -1) mySerialPort.print("xx");
	                if (tempF < 0 && tempF > -10) mySerialPort.print("x");
	                mySerialPort.print(tempF);
	                mySerialPort.print("F");
	                if ((lWaitMillis - blink_timer[0] > 15000 && rain > 1020) ||
		                (lWaitMillis - blink_timer[0] > 9000 && rain < 1020))
	                {
	                    blink_timer[0] = lWaitMillis;
	                    blink_timer[1] = 1;
	                }
                }
	        }
            break;
	    case 9:  // Display Pump override option On/Off
	        if (schedule.pumpOR) mySerialPort.print("4xON");
	        else mySerialPort.print("4OFF");
	        break;
	    case 10:  // Display Valve override option On/Off
            if (schedule.valveOR) mySerialPort.print("5xON");
	        else mySerialPort.print("5OFF");
	        break;
	    case 11:  // Display schedule option On/Off
	        if (schedule.onoff) mySerialPort.print("6xON");
	        else mySerialPort.print("6OFF");
	        break;
	    case 12:  // Display temperature history
	        if (schedule.tempHist[tempHistS] > 99) {
	            mySerialPort.print(" ");
	            mySerialPort.print(schedule.tempHist[tempHistS]);
	        }
	        else {
	            mySerialPort.print(tempHistS);
	            if (tempHistS < 10) mySerialPort.print(" ");
	            if (schedule.tempHist[tempHistS] < 10) mySerialPort.print(" ");
	            mySerialPort.print(schedule.tempHist[tempHistS]);
	        }
	        break;
    }
}

void relay_control() { // Turn relays On/Off
    EEPROM_readAnything(0, schedule);
    if (schedule.nextday1TOwater == dayOfWeek &&
        (int)hour == schedule.time1hour &&
	    (int)minute == schedule.time1minute &&
	    second < 3 &&
	    !schedule.pumptrigger &&
	    schedule.pumpOR != 0)
    {
	    rainSense();
        relay_timer = lWaitMillis;
	    schedule.pumptrigger = true;
	    timer = 1;
	    EEPROM_writeAnything(0, schedule);
    }
    else if (schedule.nextday2TOwater == dayOfWeek &&
             (int)hour == schedule.time2hour &&
		     (int)minute == schedule.time2minute &&
		     second < 3 &&
		     !schedule.pumptrigger &&
		     schedule.pumpOR != 0 &&
		     schedule.numofschedules > 1)
    {
	    rainSense();
        relay_timer = lWaitMillis;
        schedule.pumptrigger = true;
	    timer = 2;
	    EEPROM_writeAnything(0, schedule);
    }
    else if (schedule.nextday3TOwater == dayOfWeek &&
             (int)hour == schedule.time3hour &&
		     (int)minute == schedule.time3minute &&
		     second < 3 &&
		     !schedule.pumptrigger &&
		     schedule.pumpOR != 0 &&
		     schedule.numofschedules == 3)
    {
	    rainSense();
	    relay_timer = lWaitMillis;
	    schedule.pumptrigger = true;
	    timer = 3;
	    EEPROM_writeAnything(0, schedule);
    }
    if (schedule.pumptrigger) {
        if (timer == 1 &&
	        (lWaitMillis - relay_timer) / 1000 >= schedule.seconds1TOwater)
	    {
	        pressure_timer = 0;
	        relay_timer = lWaitMillis;
	        if (schedule.days1TOwater == 1 && dayOfWeek < 7) schedule.nextday1TOwater = dayOfWeek + 1;
	        else if (schedule.days1TOwater + dayOfWeek < 8) schedule.nextday1TOwater = schedule.days1TOwater + dayOfWeek;
	        else schedule.nextday1TOwater = schedule.days1TOwater + dayOfWeek - 7;
            schedule.pumptrigger = false;
	        EEPROM_writeAnything(0, schedule);
        }
        if (timer == 2 &&
	        (lWaitMillis - relay_timer) / 1000 >= schedule.seconds2TOwater)
	    {
	        pressure_timer = 0;
	        relay_timer = lWaitMillis;
	        if (schedule.days2TOwater == 1 && dayOfWeek < 7) schedule.nextday2TOwater = dayOfWeek + 1;
	        else if (schedule.days2TOwater + dayOfWeek < 8) schedule.nextday2TOwater = schedule.days2TOwater + dayOfWeek;
	        else schedule.nextday2TOwater = schedule.days2TOwater + dayOfWeek - 7;
            schedule.pumptrigger = false;
	        EEPROM_writeAnything(0, schedule);
        }
        if (timer == 3 &&
	        (lWaitMillis - relay_timer) / 1000 >= schedule.seconds3TOwater)
	    {
	        pressure_timer = 0;
	        relay_timer = lWaitMillis;
	        if (schedule.days3TOwater == 1 && dayOfWeek < 7) schedule.nextday3TOwater = dayOfWeek + 1;
	        else if (schedule.days3TOwater + dayOfWeek < 8) schedule.nextday3TOwater = schedule.days3TOwater + dayOfWeek;
	        else schedule.nextday3TOwater = schedule.days3TOwater + dayOfWeek - 7;
            schedule.pumptrigger = false;
	        EEPROM_writeAnything(0, schedule);
        }
    }
  
    if (tempOK &&
        schedule.emergency == 0 &&
        rain < 1020 &&
        pressure_timer == 0 &&
	    (schedule.pumpOR == 1 || schedule.onoff) &&
	    (schedule.pumpOR == 1 || (schedule.pumptrigger && schedule.pumpOR == 3)))
    {
        digitalWrite(pump_RelayPin, HIGH);
        pressure_timer = lWaitMillis;
    }
    else if (pump_RelayPin == HIGH &&
	         (schedule.pumpOR == 0 ||
              (!schedule.pumptrigger && schedule.pumpOR == 3)))
    {
	    digitalWrite(pump_RelayPin, LOW);
	    pressure_timer = 0;
	    schedule.pumptrigger = false;
	    EEPROM_writeAnything(0, schedule);
    }
}

void rainSense() { //  Predict future temperature and check if it's within an acceptable range for plant tolerance
    rain = 0;
    for (int j = 1; j < 7; j++) { //  Get average of 6 rain sensor readings
        rain += (int)analogRead(RainPIN); 
        delay(25);
    }
    rain = rain / 6;
}

void tempPredict() { //  Predict future temperature and check if it's within an acceptable range for plant tolerance
    tempF = 0;
    for (int j = 1; j < 5; j++) { //  Get average of 4 temperature readings from thermistor
        tempF += (int)Thermister(analogRead(ThermistorPIN)); 
        delay(25);
    }
    tempF = tempF / 4;

    EEPROM_readAnything(0, schedule);
    if (tempF < 60 &&
        tempF < (schedule.tempHist[1] + schedule.tempHist[2] + schedule.tempHist[3]) / 3)
    {
        if (tempF < 37 ||
	        ((tempF < schedule.tempHist[2] < schedule.tempHist[4]) &&
		     (tempF - (((schedule.tempHist[3] - schedule.tempHist[2]) + (schedule.tempHist[2] -
			  schedule.tempHist[1])) / 2) * 4 < 35)) || 
		    ((tempF < avgTemp(3)) && (tempF - (2 * (avgTemp(3) - tempF)) < 35)))
        {
            tempOK = false;
        }
        else tempOK = true;
    }
    else tempOK = true;
    EEPROM_writeAnything(0, schedule);
}

void status() { // Send debug messages to serial terminal
    EEPROM_readAnything(0, schedule);
    Serial.print(hour, DEC);
    Serial.print(":");
    Serial.print(minute, DEC);
    Serial.print(":");
    Serial.print(second, DEC);
    Serial.print(" DoW:");
    Serial.print(dayOfWeek, DEC);
    Serial.print(" lWaitMi:");
    Serial.print(lWaitMillis);
    Serial.print(" relay_tmr:");
    Serial.print(relay_timer);
    Serial.print(" valve_tmr:");
    Serial.print(valve_timer);
    Serial.print(" pre_tmr:");
    Serial.print(pressure_timer);
    Serial.print(" rain:");
    Serial.print(rain);
    Serial.print(" float:");
    Serial.print(digitalRead(floatswitchPin));
    Serial.print(" pressure:");
    Serial.print(analogRead(pressureswitchPin));
    Serial.print(" PR:");
    Serial.print(digitalRead(pump_RelayPin));
    Serial.print(" VR:");
    Serial.print(digitalRead(valve_RelayPin));
    Serial.print(" RE:");
    Serial.print(char(qe1Move));
    Serial.print(" M0:");
    Serial.print(menu[0]);
    Serial.print(" M1:");
    Serial.print(menu[1]);
  
    Serial.println();
    Serial.print(" onoff:");
    Serial.print(schedule.onoff);
    Serial.print(" pOR:");
    Serial.print(schedule.pumpOR);
    Serial.print(" vOR:");
    Serial.print(schedule.valveOR);
    Serial.print(" pmptr:");
    Serial.print(schedule.pumptrigger);
    Serial.print(" tmr:");
    Serial.print(timer);
    Serial.print(" countt:");
    Serial.print(counttimes);
    Serial.print(" timespd:");
    Serial.print(schedule.numofschedules);
    Serial.print(" days1:");
    Serial.print(schedule.days1TOwater);
    Serial.print(" nxtday1:");
    Serial.print(schedule.nextday1TOwater);
    Serial.print(" t1:");
    Serial.print(schedule.time1hour);
    Serial.print(":");
    Serial.print(schedule.time1minute);
    Serial.print(":");
    Serial.print(schedule.seconds1TOwater);
    Serial.print(" days2:");
    Serial.print(schedule.days2TOwater);
    Serial.print(" nxtday2:");
    Serial.print(schedule.nextday2TOwater);
    Serial.print(" t2:");
    Serial.print(schedule.time2hour);
    Serial.print(":");
    Serial.print(schedule.time2minute);
    Serial.print(":");
    Serial.print(schedule.seconds2TOwater);
    Serial.print(" days3:");
    Serial.print(schedule.days3TOwater);
    Serial.print(" nxtday3:");
    Serial.print(schedule.nextday3TOwater);
    Serial.print(" t3:");
    Serial.print(schedule.time3hour);
    Serial.print(":");
    Serial.print(schedule.time3minute);
    Serial.print(":");
    Serial.print(schedule.seconds3TOwater);
  
    Serial.println();
    Serial.print(" tempF:");
    Serial.print(tempF);
    Serial.print(" tempAvg(2):");
    Serial.print(avgTemp(2));
    Serial.print(" tempAvg(5):");
    Serial.print(avgTemp(5));
    Serial.print(" tHist0:");
    Serial.print(schedule.tempHist[0]);
    Serial.print(" tH1:");
    Serial.print(schedule.tempHist[1]);
    Serial.print(" tH2:");
    Serial.print(schedule.tempHist[2]);
    Serial.print(" tH3:");
    Serial.print(schedule.tempHist[3]);
    Serial.print(" tH4:");
    Serial.print(schedule.tempHist[4]);
    Serial.print(" tH5:");
    Serial.print(schedule.tempHist[5]);
    Serial.print(" tH6:");
    Serial.print(schedule.tempHist[6]);
    Serial.print(" tH7:");
    Serial.print(schedule.tempHist[7]);
    Serial.print(" tH8:");
    Serial.print(schedule.tempHist[8]);
    Serial.print(" tH9:");
    Serial.print(schedule.tempHist[9]);
    Serial.print(" tH10:");
    Serial.print(schedule.tempHist[10]);
    Serial.print(" tH11:");
    Serial.print(schedule.tempHist[11]);
    Serial.println();
}

void loop() {
    EEPROM_readAnything(0, schedule);
    qe1Move = qe.hb(); //  Check rotary encoder for rotation
    rotary_encoder();  //  Act if rotary encoder or push button used
    seg_display();     //  Output to 7-segment display
  
    if ((long)(millis() - lWaitMillis) >= 0) {     //  Perform once per second to improve responsiveness
        lWaitMillis += 1000;                         //  For timer setting/counting
        getDateDs1307();                             //  Get time from RTC
        relay_control();                             //  Check if Pump/Valve relays should be ON/OFF
  
        if ((int)minute == 0 && (int)second == 0) {  //  Every hour:
            tempPredict();                             //  Obtain temperature & add to an array for history storage (24 hours)
	        rainSense();                               //  Check if it's rainng/been raining
	        for (int i = 23; i > 0; i--) schedule.tempHist[i] = schedule.tempHist[i-1];
	        schedule.tempHist[0] = tempF;
	        EEPROM_writeAnything(0, schedule);
        }
  
  // Speed up time 1 (actual) second = 1 (arduino) minute, for testing
  //if (minute < 59) minute++;
  //else {
  //  minute = 0;
  //  if (hour < 23) hour++;
  //  else {
  //    hour = 0;
  //    if (dayOfWeek < 6) dayOfWeek++;
  //    else dayOfWeek = 1;}}
  
        if (schedule.valveOR != 0 &&
	        schedule.emergency == 0 &&
	        (digitalRead(floatswitchPin) == LOW ||
	         lWaitMillis - valve_timer < valvesec*1000 ||
		     schedule.valveOR == 1))
	    {
            if (digitalRead(floatswitchPin) == LOW && !valve_timer_act) {
	            valve_timer = lWaitMillis;
		        valve_timer_act = 1;
            }
            if (lWaitMillis - valve_timer > 15000) {  //  If valve has been open > 15 seconds, emergency shut down
                schedule.emergency = 1;
	            schedule.pumptrigger = false;
	            menu[0] = 8;
	            EEPROM_writeAnything(0, schedule);
                mySerialPort.print("v");    // Reset display module
	        }
	        else digitalWrite(valve_RelayPin, HIGH);
        }
        else {
            digitalWrite(valve_RelayPin, LOW);
            valve_timer_act = 0;
        }
  
        if (!schedule.pumptrigger && lWaitMillis - relay_timer > 180000 &&
	        digitalRead(floatswitchPin) == LOW)
	    {
	        schedule.emergency = 3;
	        menu[0] = 8;
	        EEPROM_writeAnything(0, schedule);
            mySerialPort.print("v");    // Reset display module
        }
    }

    if (digitalRead(pump_RelayPin) == HIGH &&
        (analogRead(pressureswitchPin)+analogRead(pressureswitchPin))/2 < 840 &&
	     lWaitMillis - pressure_timer > 500)
    {
	    schedule.emergency = 2;
	    schedule.pumptrigger = false;
        menu[0] = 8;
        EEPROM_writeAnything(0, schedule);
        mySerialPort.print("v");    // Reset display module
    }
}

void setup() {
    Wire.begin();                
    Serial.begin(9600);          //  RTC initialize
    mySerialPort.begin(9600);    //  7-Seg display initialize
    mySerialPort.print("v");     //  Reset display module

  // Uncomment and change these values to set the clock
  //second = 45;
  //minute = 18;
  //hour = 15;
  //dayOfWeek = 6;
  //dayOfMonth = 17;
  //month = 3;
  //year = 12;
  //setDateDs1307(second, minute, hour, dayOfWeek, dayOfMonth, month, year);

  // Uncomment to change the 7-segment display brightness stored in non-volotile memory
  // mySerialPort.print("z");             //  command character to change brightness
  // mySerialPort.print(B00001100,BYTE);  //  byte of 0 to 254, with the lower being brighter

    pinMode(pump_RelayPin, OUTPUT);
    pinMode(valve_RelayPin, OUTPUT);
    pinMode(SerOutFrmArdu,OUTPUT);
    pinMode(SerInToArdu,INPUT);     //  Put in to be explicit as to data direction over serial lines

    lWaitMillis = millis() + 1000;  //  millis() rollover protection initial setup
    blink_timer[0] = lWaitMillis;
    relay_timer = lWaitMillis;
    valve_timer = lWaitMillis - valvesec * 1000;
    tempPredict();
    rainSense();
}