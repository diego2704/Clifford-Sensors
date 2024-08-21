/* Alarma de Gas Metano

Hecho por:  http://elprofegarcia.com/
Conecta el Arduino con el Sensor de Gas MQ4 y cuando el nivel de concentracion de gas metano 
Supera un limite se activa una alarma sonora, el nivel de gas metado se puede visualizar por
el Monitor Serial.
Conexiones:
ARDUINO      MQ4      Zumbador

5V           Vcc
GND          GND
A0           A0

Pin 13                  +
GND                    GND

Componentes Comprados en: http://dinastiatecnologica.com/
*/

int valor_limite= 180;                  // Fiaja el valor limite en el que se activa la alarma    
                                        // Fije el valor despues de visualizar el nivel con el Monitor Serial
void setup() { 
  Serial.begin(9600);                   // Activa el puerto Serial a 9600 Baudios
  pinMode(9,OUTPUT);                   // Configura el Pin 13 como salida para el Zumbador
}

void loop() { 
  Serial.println(analogRead(A0));       // Envia al Serial el valor leido del Sensor MQ4 
 
  if(analogRead(A0) > valor_limite){    // Si la medida de gas metano es mayor de valor limite
       digitalWrite(9, HIGH);          // Enciende el Zumbador conectado al Pin 13
       Serial.println("corraaaaaaaaaa");       // Envia al Serial el valor leido del Sensor MQ4 

   }
   else{                                // Si es menor del valor limite apaga el Zumbador
      digitalWrite(9, LOW); 
   }
  delay (300);                          // Espera 300ms para realizar la proxima medida
}