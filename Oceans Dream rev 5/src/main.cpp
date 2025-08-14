// Oceans Dream Instruments 
// Revised build July 2025 rev 5     Signal K/SensESP version 2.0.0

// Now includes additional 1-wire sensor for future use
// Now includes temperature interpreter for future use
// Now includes modified RPM code
// Now includes INA219 sensor for fuel tank level

#include <WiFi.h>
#include <Adafruit_INA219.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BME280.h>  
#include <Wire.h>
#include "sensesp_onewire/onewire_temperature.h"
#include <Arduino.h>
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/analogvoltage.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/voltagedivider.h"
#include "sensesp/transforms/frequency.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/transforms/moving_average.h"

using namespace sensesp;

class TemperatureInterpreter : public CurveInterpolator {
 public:
  TemperatureInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the ohm values returned by
    // our temperature sender to degrees Kelvin
    clear_samples();
    // addSample(CurveInterpolator::Sample(knownOhmValue, knownKelvin));
    add_sample(CurveInterpolator::Sample(20, 393.15));
    add_sample(CurveInterpolator::Sample(30, 383.15));
    add_sample(CurveInterpolator::Sample(40, 373.15));
    add_sample(CurveInterpolator::Sample(55, 363.15));
    add_sample(CurveInterpolator::Sample(70, 353.15));
    add_sample(CurveInterpolator::Sample(100, 343.15));
    add_sample(CurveInterpolator::Sample(140, 333.15));
    add_sample(CurveInterpolator::Sample(200, 323.15));
    add_sample(CurveInterpolator::Sample(300, 317.15));
    add_sample(CurveInterpolator::Sample(400, 313.15)); 
  }
};

class FuelInterpreter : public CurveInterpolator {
 public:
  FuelInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate RPM to M^3/s
    clear_samples();
    // addSample(CurveInterpolator::Sample(RPM, M^3/s));
    add_sample(CurveInterpolator::Sample(600, 0.0000000694));
    add_sample(CurveInterpolator::Sample(1000, 0.000000125));
    add_sample(CurveInterpolator::Sample(1500, 0.000000222));
    add_sample(CurveInterpolator::Sample(1800, 0.000000284));
    add_sample(CurveInterpolator::Sample(2000, 0.000000347));
    add_sample(CurveInterpolator::Sample(2200, 0.000000484));
    add_sample(CurveInterpolator::Sample(2400, 0.000000620));
    add_sample(CurveInterpolator::Sample(2600, 0.000000757));
    add_sample(CurveInterpolator::Sample(2800, 0.000000893));
    add_sample(CurveInterpolator::Sample(3000, 0.00000103));
    add_sample(CurveInterpolator::Sample(3200, 0.00000124));
      
  }
};

  reactesp::ReactESP app;

    Adafruit_BME280 bme280;

    float read_temp_callback() { return (bme280.readTemperature() + 273.15);}
    float read_pressure_callback() { return (bme280.readPressure());}
    float read_humidity_callback() { return (bme280.readHumidity());}


    Adafruit_INA219 ina219;
 
    float read_current_callback() { return (ina219.getCurrent_mA() / 1000);}


// The setup function performs one-time application initialization.
void setup() {
#ifndef SERIAL_DEBUG_ENABLED

  SetupSerialDebug(115200);
#endif
             
  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("Oceans-Dream-Instruments")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi("My WiFi SSID", "my_wifi_password")
                    //->set_sk_server("192.168.10.3", 80)
                    ->enable_uptime_sensor()
                    // ->enable_ota("raspberry")
                    ->get_app();


/// Fuel Gauge Sensor ///

// Create a RepeatSensor with float output that reads the temperature
  // using the function defined above.
  auto* fuel_level =
      new RepeatSensor<float>(300000, read_current_callback);
   // e.g., 5 mins

// Send the level to the Signal K server as a Float
  fuel_level->connect_to(new SKOutputFloat("propulsion.fuelTank.level"));



/// 1-Wire Temp Sensors ///

  DallasTemperatureSensors* dts = new DallasTemperatureSensors(19); 
  
// Exhaust Temperature - /propulsion/engine/exhaustTemperature //
  auto* exhaust_temp =
      new OneWireTemperature(dts, 10000, "/Exhaust Temperature/oneWire");

  exhaust_temp->connect_to(new Linear(1.0, 0.0, "/Exhaust Temperature/linear"))
      ->connect_to(
          new SKOutputFloat("propulsion.engine.exhaustTemperature",
                             "/Exhaust Temperature/sk_path"));

// Alternator Temperature - /electrical/alternator/temperature //
  auto* alternator_temp =
      new OneWireTemperature(dts, 10000, "/Alternator Temperature/oneWire");

  alternator_temp->connect_to(new Linear(1.0, 0.0, "/Alternator Temperature/linear"))
      ->connect_to(
          new SKOutputFloat("electrical.alternator.temperature",
                             "/Alternator Temperature/sk_path"));

// Oil Temperature - /propulsion/engine/oilTemperature //
  auto* oil_temp =
      new OneWireTemperature(dts, 10000, "/Oil Temperature/oneWire");

  oil_temp->connect_to(new Linear(1.0, 0.0, "/Oil Temperature/linear"))
      ->connect_to(
          new SKOutputFloat("propulsion.engine.oilTemperature",
                             "/Oil Temperature/sk_path"));                         

// Coolant Temperature - /propulsion/engine/coolantTemperature //
  auto* coolant_temp =
      new OneWireTemperature(dts, 10000, "/Coolant Temperature/oneWire");

  coolant_temp->connect_to(new Linear(1.0, 0.0, "/Coolant Temperature/linear"))
      ->connect_to(
          new SKOutputFloat("propulsion.engine.coolantTemperature",
                             "/Coolant Temperature/sk_path"));  

/// RPM Application ///

  const char* config_path = "/sensors/engine_rpm";
  const char* config_path_calibrate = "/sensors/engine_rpm/calibrate";
  const char* config_path_skpath = "/sensors/engine_rpm/sk";

  const float multiplier = 1.0 / 11.0;
  const unsigned int read_delay = 2000;

  uint8_t pin = 16;

  auto* sensor = new DigitalInputCounter(16, INPUT_PULLUP, RISING, 2000);

  auto frequency = new Frequency(multiplier, config_path_calibrate);

  auto frequency_sk_output = new SKOutputFloat("propulsion.engine.revolutions", config_path_skpath);

  sensor
      ->connect_to(frequency)             // connect the output of sensor
                                          // to the input of Frequency()
      ->connect_to(frequency_sk_output);  // connect the output of Frequency()
                                          // to a Signal K Output as a number

  sensor->connect_to(new Frequency(multiplier, config_path_calibrate))  
  // connect the output of sensor to the input of Frequency()
          ->connect_to(new MovingAverage(2, 1.0,"/Engine RPM/movingAVG"))
          ->connect_to(new SKOutputFloat("propulsion.engine.revolutions", config_path_skpath));  
          // connect the output of Frequency() to a Signal K Output as a number

  sensor->connect_to(new Frequency(6))
  // times by 6 to go from Hz to RPM
          ->connect_to(new MovingAverage(4, 1.0,"/Engine Fuel/movingAVG")) 
          ->connect_to(new FuelInterpreter("/Engine Fuel/curve"))
          ->connect_to(new SKOutputFloat("propulsion.engine.fuel.rate", "/Engine Fuel/sk_path"));                                       

 
/// BME280 SENSOR CODE - Temp/Humidity/Pressure Sensor ///  

bme280.begin(0x76);
  // Create a RepeatSensor with float output that reads the temperature
  // using the function defined above.
  auto* engine_room_temp =
      new RepeatSensor<float>(10000, read_temp_callback);

  auto* engine_room_pressure = 
      new RepeatSensor<float>(10000, read_pressure_callback);

  auto* engine_room_humidity = 
      new RepeatSensor<float>(10000, read_humidity_callback);   

  // Send the temperature to the Signal K server as a Float
  engine_room_temp->connect_to(new SKOutputFloat("environment.engineBay.temperature"));

  engine_room_pressure->connect_to(new SKOutputFloat("environment.engineBay.pressure"));

  engine_room_humidity->connect_to(new SKOutputFloat("environment.engineBay.relativeHumidity"));

/// Engine Temp Config ///

const float Vin = 3.5;
const float R1 = 120.0;
auto* analog_input = new AnalogInput(36, 2000);

analog_input->connect_to(new AnalogVoltage(Vin, Vin))
      ->connect_to(new VoltageDividerR2(R1, Vin, "/Engine Temp/sender"))
      ->connect_to(new TemperatureInterpreter("/Engine Temp/curve"))
      ->connect_to(new Linear(1.0, 0.9, "/Engine Temp/calibrate"))
      ->connect_to(new MovingAverage(4, 1.0,"/Engine Temp/movingAVG"))
      ->connect_to(new SKOutputFloat("propulsion.engine.temperature", "/Engine Temp/sk_path"));

analog_input->connect_to(new AnalogVoltage(Vin, Vin))
      ->connect_to(new VoltageDividerR2(R1, Vin, "/Engine Temp/sender"))
      ->connect_to(new SKOutputFloat("propulsion.engine.temperature.raw"));


/// Bilge Monitor ///

auto* bilge = new DigitalInputState(17, INPUT_PULLUP, 5000);  

	auto int_to_string_function = [](int input) ->String {
		 if (input == 1) {
		   return "Water in bilge";
		 } 
		 else { // input == 0
		   return "Bilge clear";
		 }
	};

auto int_to_string_transform = new LambdaTransform<int, String>(int_to_string_function);

bilge->connect_to(int_to_string_transform)
      ->connect_to(new SKOutputString("notification.bilge"));

bilge->connect_to(new SKOutputFloat("notification.bilge.raw"));

  // Start networking, SK server connections and other SensESP internals
  sensesp_app->start();

}

void loop() { app.tick(); }