# Workshop # 7 Automatización

Integrantes:
Brayan David Acosta Gomez 
Valentina Osorio Lopez

## Implementación de la configuración maestro-esclavo conectado vía protocolo I2C. 

### Introducción
Con la llegada del internet de las cosas y la necesidad del aprovechamiento de pines de los diferentes microcontroladores fueron unas de las razones por las cuales el protocolo de comunicación I2C con Arduino es ampliamente utilizado en multitud de sensores y actuadores en la actualidad a pesar de su antigüedad. 
![Que es el protocolo serial I2C? - YouTube](https://i.ytimg.com/vi/r5rEDutaxkA/maxresdefault.jpg)
**I2C** ( _Inter-Integrated Circuit_) es un protocolo de comunicación serial que funciona con una arquitectura maestro-esclavo (_master-slave_) como se aprecia en el gráfico se emplean únicamente dos pines para la comunicación los cuales son SDA: utilizado para el intercambio de datos y SCL: empleado como señal de reloj.

A continuación se presenta el proceso y desarrollo de una configuración maestro esclavo con dos Arduinos comunicados por protocolo I2C donde el esclavo se encargara de leer el estado de un sensor de temperatura enviando dicha información cada segundo al maestro el cual encenderá un led cada vez que dicha temperatura supere los 30 grados.

### Diagrama
Para el diseño de la arquitectura optamos por utilizar la herramienta de TinkerCad la cual nos permitió realizar el montaje basado en la configuración presentada por el docente, posterior a esto también se logro programar y simular con los componentes requeridos (los cuales seran explicados más adelante) de manera previa al montaje y ejecución del "Real-Hardware".
![](https://cdn.discordapp.com/attachments/472586238096965646/976290606945013840/unknown.png)
 
 ### Componentes y Librerías
 Los componentes y módulos utilizados fueron seleccionados teniendo en cuenta que soportarán comunicación I2C.
 - Arduino UNO (2 unidades)
 - Sensor de Temperatura LM35
 - Protoboards (2 unidades)
 - Diodo Led
 - Resistencia
 - Cables y Jumpers
 
 Para los Arduino UNO utilizados se identificaron los pines SDA y SCL los cuales corresponden a A4 (SDA), A5 (SCL); Logramos medir la lectura del sensor LM35 usando una de las entradas del ADC del microcontrolador que en este caso fue el pin A0 en el cual para poder calcular la temperatura fue necesario conocer que cada 10 mv a partir de 0v representa 1 grado Celsius y se presenta la siguiente formula **float temperatura = (analogRead(A0) * 500) / 1024** sin embargo al utilizar esta forma de capturar la temperatura se presentaban variaciones muy bruscas así que implementamos otra forma colocando una sentencia **analogReference(INTERNAL)** que convertía 1024 del ADC de 5000 mV a 1100 mV y el funcionamiento mejoro considerablemente. 

Finalmente se importo y uso La **librería**  **_Wire_** una herramienta indispensable para interactuar con el **bus I2C**  **con arduino** de forma simple e intuitiva. Esta librería se utiliza para comunicar la placa arduino con dispositivos que trabajan mediante el protocolo I2C/TWI. Se usaron algunas de las funciones principales como:

- Wire.begin()
- Wire.beginTransmission()
- Wire.write()
- Wire.endTransmission()
- Wire.onRequest()
- Wire.read()
- Wire.available()


 ### Documentación Código
 Primero se realizó la programación del Arduino que haría de maestro, siendo el encargado de recoger la información del esclavo y validar constantemente si la temperatura es igual o mayor a 30 para encender un led indicativo.

Para esto utilizamos la librería wire y se configuró como maestro.
    
    //Libreria comunicacion I2C
    #include <Wire.h>
    #define LED 13 //Define blinking LED pin
    // SETUP DEL PROYECTO
    void setup(){
      Serial.begin(9600);
      pinMode(LED, OUTPUT); //SE ESTABLECE PIN 13 COMO SALIDA
      Wire.begin(); //INICIAMOS LA COMUNICACION 
    }
    
    void loop(){
      Wire.beginTransmission(1); //INICIAMOS LA TRASMICION
      Wire.write('S');
      Wire.endTransmission();
      Wire.requestFrom(1, 1);   
      byte len = Wire.read();
      Wire.requestFrom(1, (int)len);
      String temperatura="";
      while (Wire.available()) { // LECTURA DE LOS DATOS
        char c = Wire.read();   
        temp = temperatura+c;
      }
      
      float tempeFloat = temp.toFloat();  
      Serial.println(tempeFloat);
      if(tempeFloat>=30){ //VALIDACION TEMPERATURA
        digitalWrite(LED, 0);
      }else{
        digitalWrite(LED, 1);
      }
      delay(1000);
    }
Finalmente, para la programación del Arduino como esclavo, el cual será el encargado de leer y transformar la información del sensor de temperatura para luego ser enviada al maestro. De igual forma se utiliza la librería wire para la comunicación i2c pero configurado como esclavo.
    //Libreria comunicacion I2C
    #include <Wire.h>
    bool S=false;
    int sensor = 0;
    float temperatura = 0.0;
    
    // SETUP DEL PROYECTO
    void setup(){
      Serial.begin(9600); // INICIO DE SERIAL PARA VER LOS DATOS
      Wire.begin(1); //INICIAMOS LA COMUNICACION 
      Wire.onRequest(eventoSolicitud);
      Wire.onReceive(receiveEvent); 
    }
    
    void loop(){
      sensor = analogRead(0);
      temperatura = (lectura * 500.0)/1024.0;
      Serial.println(temp);
      delay(1000);
    }
    
    void receiveEvent(int howMany){
      if( Wire.read() == 'S' ){
        S = true;
      } 
    }
    
    void eventoSolicitud() {
      if( S == true ){
        Wire.write(String(temp, 3).length());
        S = false;
      }
      else{
        Serial.println(String(temp, 3));
        Wire.println(String(temp, 3));
      }
    }


### Referencias
Carmenate, J. G. (2022, 13 enero).  _Comunicación I2C con Arduino lo mejor de 2 mundos_. Programar fácil con Arduino. https://programarfacil.com/blog/arduino-blog/comunicacion-i2c-con-arduino/amp/

Gómez, E. (2021, 6 mayo).  _Medir temperatura con sensor LM35_. Rincón Ingenieril. https://www.rinconingenieril.es/medir-temperatura-con-sensor-lm35/

_LIBRERIA Wire ARDUINO_. (2019). Techamc. http://techamc.es/ARDUINO/PROGRAMACION/LIBRERIAS/LIBRERIA_Wire_ARDUINO.htm
