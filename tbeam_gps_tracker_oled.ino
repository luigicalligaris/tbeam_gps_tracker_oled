// T-Beam GPs tracker with OLED support
// Copyright 2019, Luigi Calligaris
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.


#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>


//
// Constants
//

constexpr int const OLED_WIDTH      = 128;
constexpr int const OLED_HEIGHT     =  64;

constexpr int const PIN_OLED_RESET  =   4; // OLED eset pin # (or -1 if sharing Arduino reset pin)
constexpr int const PIN_GPS_UART_TX =  12;
constexpr int const PIN_GPS_UART_RX =  15;
constexpr int const PIN_BUTTON      =  39;

constexpr unsigned long const TIME_BUTTON_STOREPOS = 3000;



//
// Globals
//

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
//Adafruit_SSD1306(uint8_t w, uint8_t h, TwoWire *twi=&Wire, int8_t rst_pin=-1, uint32_t clkDuring=400000UL, uint32_t clkAfter=100000UL);
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, PIN_OLED_RESET, 800000UL, 400000UL);

HardwareSerial GPSSerial(1);

TinyGPSPlus tGps;

unsigned long     time_button_falling    =     0;
unsigned long     time_button_rising     =     0;
bool              button_state_previous  = false;
bool              button_state_now       = false;

bool              gps_valid_fix          = false;
bool              gps_had_one_fix        = false;
bool              gps_stored_exists      = false;
double            gps_stored_lat         = 0.0; // We love Greenwich meridian
double            gps_stored_lon         = 0.0; // Go Equator!!
double            gps_stored_h           = 0.0; // The sea is always good for a swim



//
// Behavior
//

double deg_to_rad(double const deg)
{
  return deg * 3.14159265358979323846 / 180.0;
}

// DISTANCE IN METERS
double distance_2d(double const x_lat_deg, double const x_lon_deg, double const y_lat_deg, double const y_lon_deg)
{
	constexpr double const earth_radius = 6.3781e6; // in m
	
	double const delta_lat = deg_to_rad(y_lat_deg - x_lat_deg);
  double const delta_lon = deg_to_rad(y_lon_deg - x_lon_deg);
	
  double const x_lat = deg_to_rad(x_lat_deg);
  double const y_lat = deg_to_rad(y_lat_deg);
	
  double const sin_lathalf = sin(delta_lat/2.0);
  double const sin_lonhalf = sin(delta_lon/2.0);
	
  double const sin2_lathalf = sin_lathalf * sin_lathalf ;
  double const sin2_lonhalf = sin_lonhalf * sin_lonhalf ;
	
	double const a = sin2_lathalf + sin2_lonhalf * cos(x_lat) * cos(y_lat);
	
	double const c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
	
	return c * earth_radius;
}

// DISTANCE IN METERS
double distance_3d(double const x_lat_deg, double const x_lon_deg, double const x_h, double const y_lat_deg, double const y_lon_deg, double const y_h)
{
	double const dist2d = distance_2d(x_lat_deg, x_lon_deg, y_lat_deg, y_lon_deg);
	double const disth  = y_h - x_h;
	
	return sqrt(dist2d * dist2d + disth * disth);
}


void setup()
{
	pinMode(PIN_BUTTON, INPUT);
	
	
	// Initialize USB UART
	Serial.begin(115200);
	
	// Initialize GPS UART
	GPSSerial.begin(9600, SERIAL_8N1, PIN_GPS_UART_TX, PIN_GPS_UART_RX);
	GPSSerial.setTimeout(2);
	
	// SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
	if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
	{
		for(;;) // Don't proceed, loop forever
		{
			Serial.println(F("SSD1306 allocation failed"));
			display.println(F("SSD1306 allocation failed"));
			delay(2000);
		}
	}
	
	display.clearDisplay();
	display.setTextColor(WHITE);
	display.setCursor(0,0);
	display.setTextSize(1);
	display.println(F("T-Beam GPS Tracker"));
	display.println(F(""));
	display.println(F("github.com/luigicalligaris/tbeam_gps_oled"));
	display.display();
	
	delay(5000);
}


void loop()
{
	char msgstring[64];
	
	snprintf(msgstring, 63, "[Millis] %lu" , millis() ); Serial.println(msgstring);
	
	
	while (GPSSerial.available() > 0)
	{
		tGps.encode(GPSSerial.read());
	}
	
	
	if (
		tGps.location.isValid() && tGps.location.age() < 2000 &&
		tGps.altitude.isValid() && tGps.altitude.age() < 2000 &&
		tGps.hdop.isValid()     && tGps.hdop.age()     < 2000 &&
		tGps.hdop.value() <= 300
	)
	{
		gps_valid_fix   = true;
		gps_had_one_fix = true;
	}
	else
	{
		gps_valid_fix = false;
	}
	
	
	
	//
	// Position store button on GPIO39
	//
	
	button_state_previous = button_state_now;
	button_state_now      = digitalRead(PIN_BUTTON);
	
	Serial.print("[BUTTON 39]");
	
	if (button_state_previous)
		Serial.print(" Previous HI");
	else
		Serial.print(" Previous LO");
	
	if (button_state_now)
		Serial.print(" Current HI");
	else
		Serial.print(" Current LO");
	
	
	if (true == button_state_previous && false == button_state_now)
	{
		time_button_falling = millis();
		Serial.print(" --> Triggered falling");
	}
	
	if (false == button_state_previous && true == button_state_now)
	{
		time_button_rising = millis();
		Serial.print(" --> Triggered rising ");
	}
	
	if (!button_state_now && (millis() - time_button_falling > TIME_BUTTON_STOREPOS))
	{
		Serial.print(" --> LO longer than threshold");
		
		if (gps_valid_fix)
		{
			Serial.print(", storing position");
			gps_stored_exists = true;
			gps_stored_lat    = tGps.location.lat()   ;
			gps_stored_lon    = tGps.location.lng()   ;
			gps_stored_h      = tGps.altitude.meters();
		}
	}
	
	Serial.println("");
	
	
	
	snprintf(msgstring, 63, "[GPS] Loc valid %i age %i, alt valid %i age %i" , tGps.location.isValid(), tGps.location.age(), tGps.altitude.isValid(), tGps.altitude.age()); Serial.println(msgstring);
	
	snprintf(msgstring, 63, "[GPS] HDOP valid %i age %i value %i" , tGps.hdop.isValid(), tGps.hdop.age(), tGps.hdop.value()); Serial.println(msgstring);
	
	snprintf(msgstring, 63, "[GPS] Sats %i", tGps.satellites.value()); Serial.println(msgstring);
	
	//display.setTextColor(BLACK, WHITE);
	display.setTextColor(WHITE);
	display.setCursor(0,0);
	
	
	if (gps_valid_fix)
	{
		Serial.println("[GPS] Valid gps Fix");
		
		snprintf(msgstring, 63, "[GPS] Lat %f Lon %f"              , tGps.location.lat(), tGps.location.lng()                                                                       ); Serial.println(msgstring);
		snprintf(msgstring, 63, "[GPS] Date %i/%i/%i Time %i:%i:%i", tGps.date.day(), tGps.date.month(), tGps.date.year(), tGps.time.hour(), tGps.time.minute(), tGps.time.second() ); Serial.println(msgstring);
		
		
		display.clearDisplay();
		display.setTextSize(1);
		snprintf(msgstring, 63, "%02i/%02i/%04i %02i:%02i:%02i", tGps.date.day(), tGps.date.month(), tGps.date.year(), tGps.time.hour(), tGps.time.minute(), tGps.time.second() ); display.println(F(msgstring));
		snprintf(msgstring, 63, "Sats %i"             , tGps.satellites.value() ); display.println(F(msgstring));
		snprintf(msgstring, 63, "Lat %f deg"          , tGps.location.lat()     ); display.println(F(msgstring));
		snprintf(msgstring, 63, "Lon %f deg"          , tGps.location.lng()     ); display.println(F(msgstring));
		snprintf(msgstring, 63, "Alt %f m"            , tGps.altitude.meters()  ); display.println(F(msgstring));
		snprintf(msgstring, 63, "HDOP %f"             , tGps.hdop.hdop()        ); display.println(F(msgstring));
		
		if (gps_stored_exists)
		{
			double const dist3d = distance_3d(tGps.location.lat(), tGps.location.lng(), tGps.altitude.meters(), gps_stored_lat, gps_stored_lon, gps_stored_h);
			double const deltah = tGps.altitude.meters() - gps_stored_h;
			snprintf(msgstring, 63, "D3 %f m" , dist3d); display.println(F(msgstring));
			snprintf(msgstring, 63, "DH %f m" , deltah); display.println(F(msgstring));
		}
		else
		{
			snprintf(msgstring, 63, "D3 N/A"); display.println(F(msgstring));
			snprintf(msgstring, 63, "DH N/A"); display.println(F(msgstring));
		}
		
		display.display();
	}
	else
	{
		Serial.println("[GPS] Invalid GPS Fix");
		
		if (!gps_had_one_fix)
		{
			display.clearDisplay();
			
			display.setTextSize(2);
			display.println(F("No GPS fix"));
			display.display();
		}
	}
	
	Serial.println("");
	
	delay(200);
}
