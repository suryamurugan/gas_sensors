
#include "esp32-mqtt.h"
#include <DHT.h>;
#include "ArduinoJson.h"


int MQGetGasPercentage(float, int);
int  MQGetPercentage(float, float);

DHT dht(4,DHT22 ); //// Initialize DHT sensor for normal 16mhz Arduino
int chk;
float hum;  //Stores humidity value
float temp; //Stores temperature value
DynamicJsonDocument doc(1024) ;
DynamicJsonDocument ht(512);
char charBuf[150] ;

#define         MQ_PIN                       (36)     //define which analog input channel you are going to use h2
#define         RL_VALUE                     (10)    //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (9.21)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                                     //which is derived from the chart in datasheet

/***********************Software Related Macros************************************/
#define         CALIBARAION_SAMPLE_TIMES     (20)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (200)   //define the time interal(in milisecond) between each samples in the
                                                     //cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in
                                                     //normal operation


#define         MG_PIN                       35     //define which analog input channel you are going to use
#define         BOOL_PIN                     (2)
#define         DC_GAIN                      (8.5)   //define the DC gain of amplifier

#define         ZERO_POINT_VOLTAGE           (0.220) //define the output of the sensor in volts when the concentration of CO2 is 400PPM
#define         REACTION_VOLTGAE             (0.020) //define the voltage drop of the sensor when move the sensor from air into 1000ppm CO2
 
/*****************************Globals***********************************************/
float           CO2Curve[3]  =  {2.602,ZERO_POINT_VOLTAGE,(REACTION_VOLTGAE/(2.602-3))};   
                                                     //two points are taken from the curve. 
                                                     //with these two points, a line is formed which is
                                                     //"approximately equivalent" to the original curve.
                                                     //data format:{ x, y, slope}; point1: (lg400, 0.324), point2: (lg4000, 0.280) 
                                                     //slope = ( reaction voltage ) / (log400 –log1000) 
/**********************Application Related Macros**********************************/
#define         GAS_H2                      (1)
#define         GAS_co                     (2)

/*****************************Globals***********************************************/
float           H2Curve[3]  =  {2.3, 0.93,-1.44};    //two points are taken from the curve in datasheet.
                                                     //data format:{ x, y, slope}; point1: (lg200,lg8.5), point2: (lg10000, lg0.03)
float           COCurve[3]  =  {2.3, 0.23, -0.49};    //two points are taken from the curve in datasheet.
                                                     //data format:{ x, y, slope}; point1: (lg200,lg1.7), point2: (lg1000, lg0.78)

float           R0           =  10;                  //Ro is initialized to 10 kilo ohms
float           Ro           =  10;                  //Ro is initialized to 10 kilo ohms


void setup() {
  
  Serial.begin(115200);
  
  pinMode(LED_BUILTIN, OUTPUT);
  dht.begin();
  
  pinMode(4, INPUT);
  Serial.print("Initializing\nCalibrating...\n");               
  Ro = MQCalibration(MQ_PIN);
  //Calibrating the sensor. Please make sure the sensor is in clean air when you perform the calibration              
  Serial.print("Calibration is done...\n");
  Serial.print("Ro=");
  Serial.print(Ro);
  Serial.print("kohm");
  Serial.print("\n");
  pinMode(BOOL_PIN, INPUT);                        //setpin to input
  digitalWrite(BOOL_PIN, HIGH);                    //turn on pullup resistors
  Serial.print("Done Init\n");                
  //setupCloudIoT();
}

unsigned long lastMillis = 0;


////////////////////////////// FUNCTION TO READ TEMP AND HUMIDITY DHT22 //////////////////////////////
String dht22()
{
 
  // Reading temperature or humidity takes about 250 milliseconds and upto 2s!
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  Serial.println(dht.readTemperature());
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) )
  {
    Serial.println(F("Failed to read from DHT sensor!"));
    return "None";
  }
  Serial.print(F("Humidity: "));
  Serial.println(h);
  Serial.print(F("Temperature: ")); 
  Serial.println(t);
  return String(String(h)+"-"+String(t)+"-");
}


////////////////////////////// FUNCTION TO READ AIRPURITY/AIRQUALITY  //////////////////////////////
float Airquality( )
{
  float ppm;
  ppm = analogRead(34);
  //Serial.print(F("airquality: "));
  //Serial.println(ppm);
  return ppm;  
}

////////////////////////////// FUNCTION TO READ H2  //////////////////////////////
int h2(){
  return MQGetGasPercentage(MQRead(MQ_PIN)/Ro,GAS_H2);
  }
////////////////////////////// FUNCTION TO READ CO  //////////////////////////////
int co(){
  return MQGetGasPercentage(CO(),GAS_co);;
  
  }




float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(1023-raw_adc)/raw_adc));
}

////////////////////////////// FUNCTION TO CALIBRATE  //////////////////////////////

float MQCalibration(int mq_pin)
{
  int i;
  float val=0;

  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) {            //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val/CALIBARAION_SAMPLE_TIMES;                   //calculate the average value

  val = val/RO_CLEAN_AIR_FACTOR;                        //divided by RO_CLEAN_AIR_FACTOR yields the Ro
                                                        //according to the chart in the datasheet

  return val;
}

////////////////////////////// FUNCTION TO READ   //////////////////////////////

float MQRead(int mq_pin)
{
  int i;
  float rs=0;

  for (i=0;i<READ_SAMPLE_TIMES;i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs/READ_SAMPLE_TIMES;

  return rs; 
}

////////////////////////////// FUNCTION TO GETGAS%  //////////////////////////////
int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{
  if ( gas_id == GAS_H2) {
  //  Serial.println("H2");
    
     return MQGetPercentage(rs_ro_ratio,H2Curve);
    
  } 
   else if( gas_id == GAS_co){
     // Serial.println("CO");
    
     return MQGetPercentage(rs_ro_ratio,COCurve);
     }
     
  

}

int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}


////////////////////////////// FUNCTION TO READ CO2  //////////////////////////////
float co2()
{
  int percentage;
  float volts;
  volts = MGRead(MG_PIN);
  percentage = MGGetPercentage(volts,CO2Curve);
  //Serial.print("CO2:");
    if (percentage == -1) {
        //Serial.print( "<400" );
    } else {
       // Serial.print(percentage);
    }
 
    //Serial.print( "ppm" );  
    //Serial.print("\n");
 
    if (digitalRead(BOOL_PIN) ){
        //Serial.print( "=====BOOL is HIGH======" );
    } else {
      //  Serial.print( "=====BOOL is LOW======" );
    }
 
    //Serial.print("\n");
 
    delay(200);
  return percentage;
}


/*****************************  MGRead *****************************************
Input:   mg_pin - analog channel
Output:  output of SEN-000007
Remarks: This function reads the output of SEN-000007
************************************************************************************/ 
float MGRead(int mg_pin)
{
    int i;
    float v=0;
 
    for (i=0;i<READ_SAMPLE_TIMES;i++) {
        v += analogRead(mg_pin);
        delay(READ_SAMPLE_INTERVAL);
    }
    v = (v/READ_SAMPLE_TIMES) *5/1024 ;
    return v;  
}
 
/*****************************  MQGetPercentage ******************************
Input:   volts   - SEN-000007 output measured in volts
         pcurve  - pointer to the curve of the target gas
Output:  ppm of the target gas
Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm) 
         of the line could be derived if y(MG-811 output) is provided. As it is a 
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic 
         value.
************************************************************************************/ 
int  MGGetPercentage(float volts, float *pcurve)
{
   if ((volts/DC_GAIN )>=ZERO_POINT_VOLTAGE) {
      return -1;
   } else { 
      return pow(10, ((volts/DC_GAIN)-pcurve[1])/pcurve[2]+pcurve[0]);
   }
}
float CO()
{  
float sensor_volt;
    float RS_gas; // Get value of RS in a GAS
    float ratio; // Get ratio RS_GAS/RS_air
    int sensorValue = analogRead(39);
    sensor_volt=(float)sensorValue/1024*5.0;
    RS_gas = (5.0-sensor_volt)/sensor_volt; // omit *RL

          /*-Replace the name "R0" with the value of R0 in the demo of First Test -*/
    ratio = RS_gas/R0;  // ratio = RS/R0
          /*-----------------------------------------------------------------------*/

    //Serial.print("sensor_volt = ");
    //Serial.println(sensor_volt);
    //Serial.print("RS_ratio = ");
    //Serial.println(RS_gas);
    //Serial.print("Rs/R0 = ");
    //Serial.println(ratio);

    delay(1000);
    return ratio;

}


void loop() {
    delay(2000);
    //Read data and store it to variables hum and temp
    //mqtt->loop();
    delay(10);  // <- fixes some issues with WiFi stability

 // if (!mqttClient->connected()) {
  ///  connect();
    
//}

   doc["h2"]= h2();
   doc["co"]= co();
   doc["co2"] = co2();
   doc["ap"] = Airquality();
   doc["humiditya"]=dht22().substring(0,5);
   doc["tempa"] = dht22().substring(6,11);
   serializeJson(doc,charBuf);
   serializeJson(doc,Serial);
   Serial.println(" ");
  


  // doc["humiditya"]=dht22().substring(0,5);
  // doc["tempa"] = dht22().substring(6,11);
  // doc["airpurity"] = Airquality();
   //doc["watertemp"] =  water_temp();//== NULL ?float(0):water_temp() ;
   //doc["ph"] = pH_value();//== NULL ?float(0):pH_value() ; 
  // publish a message roughly every second.
  //if (millis() - lastMillis > 60000) {
 //   lastMillis = millis();
    //publishTelemetry(mqttClient, "/sensors", getDefaultSensor());
   // publishTelemetry(charBuf);
   // free(charBuf);
  
}
