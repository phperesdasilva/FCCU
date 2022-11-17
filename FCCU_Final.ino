//#include <pwmWrite.h>
#include <math.h>
#include <analogWrite.h>


#include <Wire.h>
#include <Adafruit_ADS1015.h>

Adafruit_ADS1115 ads(0x48);
//Adafruit_ADS1015 ads(0x48);

TaskHandle_t Nucleo0;
TaskHandle_t Nucleo1;

#define a 80
#define sz 5
#define n 1000

int16_t adcPressao; //PRESSÃO 
int16_t adcTemp; //TEMP
int16_t adcCurrent; //CORRENTE

int ctest, ptest;

double temp;

int media=0;
int i=0;
int valores[a];

float mapeamento;

int contador = millis();
int leituraTeste;

#define girapara 27//switch rele

void releStart(){
  if(digitalRead(girapara)==HIGH){
    start();
  }
  else{
    shut();
  }
}

//INTERRUPT------------------------------------------
#define bot 34 //botao interruption
int state = 0;

void IRAM_ATTR startSol() {
  state = !state;
  if (state) {
    start();
  }
  else {
    shut();
  }
}

//PWM------------------------------------------------
const byte pwmDCDC = 13; //dcdc
const byte pwmFan = 33; //fan
const float freq = 100;
const byte resolution = 10;
int duty, dutyDCDC, dutyFan;

//Pwm pwm = Pwm();


//SETPOINT MEAN--------------------------------------

int mean = 0;
int j = 0;
int set[n];

//PID------------------------------------------------
int Ctime, Etime, Ptime = 0;
                          //PARAETROS VENTOINHA
float Kp_t = 4.0 
*10;      
float Ki_t = 0.01;
float Kd_t = 50 
*1000;
//float Ki_t = 0;
//float Kd_t = 0; 
int p = 0;
float out[sz];
float meio = 0;

float Kp_c = 0.05;   //PARAMETROS CORRENTE
float Ki_c = 0.0005;

float setPoint_t = 12000; //50 graus no teste

float setPoint_c = 35;

float output = 0;

float Cerror, Lerror, intError, CintError, derError;

float Temperatura_ideal = 0;
float Tempo_purga = 0;

float corrente = 20;

//Temperatura_ideal = (corrente*0.53)+26.01;
//Tempo_purga = 2300/corrente;

void setup() {

  //attachInterrupt(bot, startSol, RISING);

  //pinMode(bot, INPUT);

  pinMode(27, INPUT); //SWITCH
  //spare 5 (26) livre
  pinMode(12, OUTPUT); //rele eletronica
  pinMode(14, OUTPUT); //rele UDA 
  
  ads.begin();

  Serial.begin(115200);

  for (i = 0; i<a;i++) valores[i]=0;
  i=0;
  for (i = 0; i<a;i++) out[i]=0;
  i=0;

  analogWriteResolution(resolution); //Sets PWM resolution value Globaly to "Resolution"
  analogWriteFrequency(freq); //Sets PWM frequency value Globaly to "freq"
   
  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Loop1,   /* Task function. */
                    "Loop1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Nucleo0,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Loop2,   /* Task function. */
                    "Loop2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Nucleo1,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
}




void Loop1(void * pvParameters){for(;;){

  //Temperatura_ideal = (corrente*0.53)+26.01;
  //Tempo_purga = 2300/corrente;
  
  //readCurrent();
  
  //duty = adc1/28;

  /*Serial.print(duty);
  Serial.print("  ");
  Serial.println(adc1);*/
  
  //pwm.write(pwmDCDC, duty, freq, resolution, 0);

  /*dutyDCDC = PID(adcPressao, Kp_t, Ki_t, setPoint_t, 0);
   * dutyDCDC
  pwm.write(pwmDCDC, dutyDCDC, freq, resolution, 0);*/

  //spVar();

  //setPoint_t = Temperatura_ideal;
  
  //setPoint_t = 65;

  dutyFan = PID(readTempCelsiusMean(), Kp_t, Ki_t, Kd_t, setPoint_t, 0);
  //Serial.print("dutyFan: ");  Serial.print(dutyFan);   Serial.print(" ");



  analogWrite(pwmFan, dutyFan, 1023); //Sets PWM's duty cycle on pin "pwmFan" to value "dutyFan" with its max value being 1023
  
  //pwm.write(pwmFan, dutyFan, freq, resolution, 0);
  
  //testTemp();
  //test();
  //readTemp();
  //readPot();
  //testTempFix();
  //testTempFixMean();
  //readTempCelsius();

  //releStart();
  //readPot();
  //sendPWM();


  //pwm.write(pwmFan, 1000, freq, resolution, 0);
  spVar();
}}

void Loop2(void * pvParameters){for(;;){

  int set = 0;
  //Lendo O Setpoint (potenciometro)
  if (analogRead(25)<=500) setPoint_t =40;
  else if (analogRead(25)>=3500) setPoint_t =60;
  else setPoint_t = 50;
  // mean -= set[j];
  //set[j] = analogRead(25);
  //mean += set[j];
  //j++; if (j>=n) j = 0;


  ///setPoint_t = mean/(40.0*n);
}}

void loop() {} //Do not use


double PID (float input, float Kp, float Ki, float Kd, float setPoint, int inv) {
  
  Serial.print("time  ");
  Serial.print(millis());
  Serial.print("  ");
  
  Serial.print("sp  ");
  Serial.print(setPoint);
  Serial.print("  ");

  Serial.print("input ");
  Serial.print(input);
  Serial.println("  ");
/*
  Serial.print("output  ");
  Serial.print(round(output/10)); //prints both sp and input for visual calibration
  Serial.println("  ");

  Serial.print("derError x Kd  ");
  if(derError*Kd>=-1023 && derError*Kd<=1023) Serial.print(derError*Kd/10); //prints both sp and input for visual calibration
  else Serial.print("0");
  Serial.print("  ");

  Serial.print("CError x Kp  ");
  if(Cerror*Kp>=-1023 && Cerror*Kp<=1023) Serial.print(Cerror*Kp/10); //prints both sp and input for visual calibration
  else Serial.print("0");
  Serial.print("  ");*/

  /*Serial.print("intError x Ki  ");
  if(intError*Ki>=-1023 && intError*Ki<=1023)Serial.print(intError*Ki/10); //prints both sp and input for visual calibration
  else Serial.print("0");
  Serial.print("  ");

  Serial.print("  ");
  Serial.println(intError);
  */
  Ctime = millis();

  Etime = (Ctime - Ptime);

  

  if (Etime >= 100) {

    //Serial.print(Etime);
    Cerror = input - setPoint;

    /*Serial.print("Elap. Time ");
      Serial.print(Etime);
      Serial.print("  |");*/

    /*Serial.print("Error ");
    Serial.print(Cerror);
    Serial.print("  |");*/

    //intError = (Cerror + Lerror) * (Etime / 2); //Metodo1 (Não Funciona)
    
    //intError += (Cerror + Lerror) * (Etime / 2); //Metodo2 (Error pode ir ao infinito)

   
    CintError = (Cerror + Lerror) * (Etime / 2); //Metodo3 (Corrigi o Erro ir ao infinito)
     
    //if(!((intError*Kp>=1023 && output >=1023 && NexintError>=0)||(intError*Kp<=-1023 && output <=0 && NexintError<=0))) intError += NexintError; //Metodo3.1 (Funciona porem muitas vezes o erro parava de contar impropriamente)
    
    //if(intError*Kp>=1023 && output >=1023 && NexintError>=0) {}  //Metodo3.2 (Funciona, porém passa por muitos ifs)
    //else if(intError*Kp<=-1023 && output <=0 && NexintError<=0){}
    //else intError += NexintError;
    //if((outpu<=1023 || NextintError < 0) && (outpu>=0 || NextintError > 0) intError += NexintError;
    
    // if(output + CintError*Ki <=1023 && output +CintError*Ki>=0 ) intError += CintError; //Metodo3.3 (A ser Testado)
    
    if(output <1023 && output >0 ) intError += CintError; //Metodo3.4 (A ser Testado)
    

    
    
    /*if(intError >= errorRange){
    intError = errorRange;
    }

    if(intError <= -errorRange){
    intError = -errorRange;
    }*/

    /*Serial.print("i.Error ");
    Serial.print(intError);
    Serial.print("  |");*/

    derError = (Cerror - Lerror)/Etime;

    

    output = Kp * Cerror + Ki * intError + Kd * derError;

    if (output >= 1023) output = 1023;
    

    if (output <= 0) output = 0;
    

    if (inv == 1) output = 1023 - output;

    //Pequena média movel para atenuar o resultado
    meio -= out[p];
    out[p] = output;
    meio += out[p];
    p++; if (p>=sz) p = 0;
    

    

    Lerror = Cerror;
    Ptime = Ctime;
    
  
  }

  
  
  
    return round(meio/sz);
  
}

void spVar(){

  ctest = millis();

  if(ctest - ptest >= 60*1000){  
    ptest = ctest;
    if(setPoint_t == 12000){
      setPoint_t = 15000;
    }
    else{
      setPoint_t = 12000;
    }
  }
}

void test() {

  int i = 0;
  while (i < 1023) {
    duty = i;
    //Serial.println(duty);
    //            pin, duty, freq, resolution, shift
    analogWrite(pwmFan, duty, 1023); //Sets PWM's duty cycle on pin "pwmFan" to value "duty" with its max value being 1023
    //  pwm.write(pwmFan, duty, freq, resolution, 0);
    i++;
    delay(10);
  }
}

void testTemp() {

  int i = 0;
  while (i < 1023) {
    duty = i;
    //Serial.println(duty);
    //            pin, duty, freq, resolution, shift
    readTemp();
    analogWrite(pwmFan, duty, 1023); //Sets PWM's duty cycle on pin "pwmFan" to value "duty" with its max value being 1023
  //  pwm.write(pwmFan, duty, freq, resolution, 0);
    i++;
    delay(10);
  }
}

void testTempFix() {

  //Serial.println(duty);
  //            pin, duty, freq, resolution, shift
  readTemp();
  analogWrite(pwmFan, 250, 1023); //Sets PWM's duty cycle on pin "pwmFan" to value "250" with its max value being 1023
//  pwm.write(pwmFan, 250, freq, resolution, 0);
  i++;
  delay(10);
  
}

void testTempFixMean() {

  //Serial.println(duty);
  //            pin, duty, freq, resolution, shift
  readTempMean();
  analogWrite(pwmFan, 250, 1023); //Sets PWM's duty cycle on pin "pwmFan" to value "250" with its max value being 1023
  //pwm.write(pwmFan, 250, freq, resolution, 0);
  i++;
  delay(10);
  
}

/*void sendPWM(){
  adcPressao = ads.readADC_SingleEnded(0);
  duty = adcPressao/14;
  pwm.write(pwmDCDC, duty, freq, resolution, 0);
 }*/


void readPot(){
  adcPressao = ads.readADC_SingleEnded(0);

  Serial.println(adcPressao);
}

void readTemp(){
  adcTemp = ads.readADC_SingleEnded(2);
  Serial.println(adcTemp);
}


double readTempCelsius(){
  adcTemp = ads.readADC_SingleEnded(2);
  //adcTemp = (-2*(10^-11)(adcTemp^3))+(1(10^-6)(adcTemp^2))-(1.99(10^-2)*adcTemp)+162.47;
  temp = 111.35*(exp(-0.00007*adcTemp));
  //Serial.println(temp);

  return temp;
}

double readTempCelsiusMean(){
  media-=valores[i];
  
  adcTemp = ads.readADC_SingleEnded(2);
  
  valores[i] = adcTemp;
  media+=valores[i];
 
  i++;
  if(i>=a) i=0;

  temp = 111.35*(exp(-0.00007*media/a));
  //Serial.println(temp);

  return temp;
}

void readTempMean(){
  media -= valores[i];
  
  adcTemp = ads.readADC_SingleEnded(2);
  valores[i] = adcTemp;
  media += valores[i];
  
  i++;
  if(i>=a)i=0;

  
  
  //mapeamento = map(media, -500, -997, 0, 15.72);
  
  //Serial.print(media);
  //Serial.print(mapeamento); 
  //Serial.print(valores[i]);
  Serial.println(media/a);
}

void readCurrent(){
  media -= valores[i];
  
  adcCurrent = ads.readADC_SingleEnded(4);
  valores[i] = adcCurrent;
  media += valores[i];
  
  i++;
  if(i>=a)i=0;

 
  
  mapeamento = map(1.0*media/a, -500, -997, 0, 15.72);
  
  //Serial.print(media);
  //Serial.print(mapeamento); 
  //Serial.print(valores[i]);
  Serial.println();
}

void start() {
  digitalWrite(12, HIGH); //rele eletronica
  digitalWrite(14, HIGH); //UDA
  

}

void shut() {
  digitalWrite(12, LOW); //rele eletronica
  digitalWrite(14, LOW); //UDA

}
