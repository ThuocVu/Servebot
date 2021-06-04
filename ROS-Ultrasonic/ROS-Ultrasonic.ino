/*
* Arduino - Ultrasonnic Sensor HC-SR04
*
* Wiring: Ultrasonic Sensor -> Arduino:
* - VCC - 5VCC
* - GND - GND
* - TRIG - Pin10
* - ECHO - PIN11
*
*/

// Ultrasonic pins definition
const int Echo = 11, Trig = 10;
long duration, cm;

void setup()
{
    Serial.begin(115200);
    setupUltrasonic();
}

// Main loop
void loop()
{
    update_Ultrasonic();
}
// Setup Ultrasonic Sensor
void setupUltrasonic()
{
    // Configure the trigger pin to output mode
    pinMode(Trig, OUTPUT);
    // Configure the echo pin to input mode
    pinMode(Echo, INPUT);
}

// Update Ultrasonic Sensor
void update_Ultrasonic()
{
    digitalWrite(Trig, LOW);
    delayMicroseconds(2);
    digitalWrite(Trig, HIGH);
    delayMicroseconds(10);

    duration = pulseIn(Echo, HIGH);
    // Convert duration -> cm
    cm = durationTocm(duration);

    // Sending through serial port
    Serial.print("u");
    Serial.print("\t");
    Serial.print(cm);
    Serial.print("\n");

}

// Convert microseconds to centimeter
long durationTocm(long microseconds)
{
    return (microseconds/29)/2;
}