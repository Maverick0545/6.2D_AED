#include "ADC_DEAKIN.h"
#include "DHT.h"
#include "GPIO_DEAKIN.h"
#include "TIMER_DEAKIN.h"
#include <Arduino.h>

// Define constants for GPIO pin connections
#define LED1_PIN 5 // LED1 connected to digital pin 5
#define LED2_PIN 6 // LED2 connected to digital pin 6
#define DHT_PIN 4  // DHT sensor connected to digital pin 4

// Initialize components
ADC_DEAKIN adc;
DHT dht(DHT_PIN, DHT22); // Initialize DHT22 sensor for humidity and temperature
GPIO_DEAKIN gpio;
TIMER_DEAKIN timer;

// Constants for RN3440 thermistor calculations
const float R_known = 10000.0; // 10kΩ resistor value
const float V_in = 3.3; // Input voltage (3.3V for Arduino Nano 33 IoT)
const float BETA = 3950; // β-value for thermistor (adjust according to datasheet)
const float R_0 = 10000.0; // Resistance at 25°C (10kΩ)
const float T_0 = 298.15; // Reference temperature (25°C in Kelvin)

// Number of sensor readings for averaging
const int numSamples = 10;

// Arrays to store sensor samples
float temperatureSamples[numSamples];
float humiditySamples[numSamples];
int sampleIndex = 0; // Index for the current sample

// Variables to hold averaged sensor readings
float averageTemperature = 0; // Averaged temperature
float averageHumidity = 0; // Averaged humidity

// Timing variables for readings and alarms
unsigned long lastReadTime = 0;
unsigned long samplingInterval = 60000; // Sampling interval set to 1 minute
unsigned long alarmTimestamp = 0; // Timestamp for last triggered alarm
int frequency = 5; // Default sampling frequency (in seconds)

// Function to calculate thermistor resistance from ADC value
float getThermistorResistance(int adcValue) {
    float V_out = (adcValue * V_in) / 1023.0; // Convert ADC value to voltage
    if (V_out == 0) return -1; // Prevent division by zero
    return R_known * ((V_in / V_out) - 1); // Calculate thermistor resistance
}

// Function to calculate temperature in Celsius using the β-model
float convertResistanceToTemperature(float resistance) {
    if (resistance <= 0) return NAN; // Invalid resistance
    float tempKelvin = 1.0 / ((1.0 / T_0) + (1.0 / BETA) * log(resistance / R_0));
    return tempKelvin - 273.15; // Convert from Kelvin to Celsius
}

// Function to read humidity from the DHT sensor
void captureHumidityData() {
    float humidity = dht.readHumidity(); // Get humidity reading
    if (isnan(humidity)) {
        Serial.println("Error: Could not read humidity from sensor.");
        return;
    }
    humiditySamples[sampleIndex] = humidity; // Store humidity sample
}

// Function to read temperature using the thermistor
void captureTemperatureData() {
    int adcValue;
    adc.read_ADC(&adcValue); // Read ADC value
    float resistance = getThermistorResistance(adcValue); // Calculate thermistor resistance
    float temperature = convertResistanceToTemperature(resistance); // Calculate temperature

    // Adjust temperature readings for calibration
    if (temperature < 0) {
        temperature += 22; // Adjust negative readings by adding 22°C
    } else if (temperature > 0) {
        temperature -= 18; // Adjust positive readings by subtracting 18°C
    }

    // Store valid temperature readings
    if (!isnan(temperature)) {
        temperatureSamples[sampleIndex] = temperature;
    } else {
        temperatureSamples[sampleIndex] = 0; // Store default if invalid
    }
}

// Function to calculate the average temperature and humidity
void computeAverageReadings() {
    float totalTemperature = 0;
    float totalHumidity = 0;
    int validTemperatureCount = 0;

    // Sum valid temperature and humidity values
    for (int i = 0; i < numSamples; i++) {
        if (temperatureSamples[i] >= 0) {
            totalTemperature += temperatureSamples[i];
            validTemperatureCount++;
        }
        totalHumidity += humiditySamples[i];
    }

    // Calculate average values
    if (validTemperatureCount > 0) {
        averageTemperature = totalTemperature / validTemperatureCount;
    } else {
        averageTemperature = 0; // Default if no valid readings
    }
    averageHumidity = totalHumidity / numSamples;

    // Output averaged readings
    Serial.print("Avg Temp: ");
    Serial.print(averageTemperature);
    Serial.println(" °C");
    Serial.print("Avg Humidity: ");
    Serial.print(averageHumidity);
    Serial.println(" %");
}

// Function to blink an LED at a specified frequency
void blinkLEDAtFrequency(uint8_t pin, int blinkFrequency) {
    unsigned long interval = 1000 / blinkFrequency; // Calculate on/off interval
    gpio.digitalWrite(pin, HIGH); // Turn LED on
    timer.start(interval); // Start timer for LED on duration
    while (!timer.isComplete()); // Wait for timer to complete
    gpio.digitalWrite(pin, LOW); // Turn LED off
    timer.start(interval); // Start timer for LED off duration
    while (!timer.isComplete()); // Wait for timer to complete
}

// Function to check and trigger alarms based on readings
void evaluateAlarmConditions() {
    // Temperature-based alarm
    if (averageTemperature < 4) {
        Serial.println("Warning: Temp < 4°C - LED1 blinking at 10Hz");
        alarmTimestamp = millis();
        for (int i = 0; i < 15; i++) {
            blinkLEDAtFrequency(LED1_PIN, 10); // Blink LED1 at 10Hz
        }
    } else if (averageTemperature < 10) {
        Serial.println("Warning: Temp < 10°C - LED1 blinking at 1Hz");
        alarmTimestamp = millis();
        for (int i = 0; i < 15; i++) {
            blinkLEDAtFrequency(LED1_PIN, 1); // Blink LED1 at 1Hz
        }
    }

    // Humidity-based alarm
    if (averageHumidity < 30) {
        Serial.println("Warning: Humidity < 30% - LED2 ON for 15s");
        alarmTimestamp = millis();
        gpio.digitalWrite(LED2_PIN, HIGH); // Turn LED2 on
        timer.start(15000); // Keep LED2 on for 15 seconds
        while (!timer.isComplete());
        gpio.digitalWrite(LED2_PIN, LOW); // Turn LED2 off
    } else if (averageHumidity < 50) {
        Serial.println("Warning: Humidity < 50% - LED2 blinking at 0.5Hz");
        alarmTimestamp = millis();
        for (int i = 0; i < 15; i++) {
            blinkLEDAtFrequency(LED2_PIN, 2); // Blink LED2 at 0.5Hz (2s cycle)
        }
    }
}

// Function to display the serial menu and handle input
void showMenuAndHandleInput() {
    Serial.println("Menu:");
    Serial.println("a - Show current sensor readings with timestamp");
    Serial.println("b - Show recent alarm status with timestamp");
    Serial.println("Enter a number between 1 and 30 to set the sampling frequency");
    Serial.print("Your choice: ");

    while (!Serial.available()); // Wait for user input

    String input = Serial.readStringUntil('\n'); // Read input
    input.trim(); // Remove whitespace

    if (input.equalsIgnoreCase("a")) {
        // Display current sensor readings
        Serial.print("\nTemp: ");
        Serial.print(temperatureSamples[sampleIndex]);
        Serial.print(" °C, Timestamp: ");
        Serial.println(millis());

        Serial.print("Humidity: ");
        Serial.print(humiditySamples[sampleIndex]);
        Serial.print(" %, Timestamp: ");
        Serial.println(millis());
    } else if (input.equalsIgnoreCase("b")) {
        // Display last alarm states
        Serial.println("\nAlarm States:");
        Serial.print("Last temperature alarm at: ");
        Serial.println(alarmTimestamp);
        Serial.print("Last humidity alarm at: ");
        Serial.println(alarmTimestamp);
    } else {
        // Set new sampling frequency
        int freq = input.toInt();
        if (freq >= 1 && freq <= 30) {
            frequency = freq;
            samplingInterval = frequency * 1000; // Convert to milliseconds
            Serial.print("\nSampling frequency set to ");
            Serial.print(frequency);
            Serial.println(" seconds.");
        } else {
            Serial.println("\nInvalid input. Please choose 'a', 'b', or a number between 1 and 30.");
        }
    }
}

void setup() {
    Serial.begin(9600);
    while (!Serial); // Wait for the serial connection
    dht.begin(); // Initialize DHT sensor
    adc.setup_ADC('A', 3, 10, 0); // Set up ADC on pin A3
    adc.enable_ADC(); // Enable ADC

    // Configure LED pins as outputs
    gpio.pinMode(LED1_PIN, OUTPUT);
    gpio.pinMode(LED2_PIN, OUTPUT);

    timer.init(); // Initialize the timer
    Serial.println("System initialized and ready.");
}

void loop() {
    // Display the menu before reading sensor data
    showMenuAndHandleInput();

    unsigned long currentTime = millis();
    if (lastReadTime == 0 || currentTime - lastReadTime >= samplingInterval) {
        lastReadTime = currentTime;

        // Capture sensor readings and update the average values
        for (int i = 0; i < numSamples; i++) {
            captureTemperatureData(); // Read temperature
            captureHumidityData(); // Read humidity

            // Move to the next sample index
            sampleIndex = (sampleIndex + 1) % numSamples;

            // Wait for the set frequency between readings
            for (int j = 0; j < frequency; j++) {
                timer.start(1000); // Wait 1 second between samples
                while (!timer.isComplete());
            }
        }

        // Calculate averages and check for any alarms
        computeAverageReadings();
        evaluateAlarmConditions();
    }
}