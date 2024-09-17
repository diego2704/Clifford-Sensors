int valor_limite = 500;                 // Valor límite para activar la alarma

void setup() {
  Serial.begin(9600);                   // Inicia el monitor serial a 9600 baudios
  pinMode(9, OUTPUT);                   // Configura el pin 9 como salida para el zumbador
}

void loop() {
  int lectura_sensor = analogRead(A0);  // Lee el valor del sensor MQ-4 en el pin A0
  Serial.println(lectura_sensor);       // Envía el valor leído al monitor serial

  if (lectura_sensor > valor_limite) {  // Si el valor supera el límite establecido
    digitalWrite(9, HIGH);              // Activa el zumbador
    Serial.println("¡Alerta de gas metano!"); // Mensaje de alerta en el monitor serial
  } else {                              // Si el valor es menor que el límite
    digitalWrite(9, LOW);               // Apaga el zumbador
  }

  delay(300);                           // Espera 300 ms antes de realizar la próxima lectura
}
