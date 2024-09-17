import serial
import pynmea2

ser = serial.Serial('COM11', baudrate=9600, timeout=1)

def decode_nmea_sentence(sentence):
    try:
        # Decodificar la trama NMEA
        msg = pynmea2.parse(sentence)

        # Verificar si la trama es de tipo GGA (Fix information)
        if isinstance(msg, pynmea2.GGA):
            latitude = msg.latitude
            longitude = msg.longitude
            altitude = msg.altitude

            # Imprimir los resultados
            print(f"Latitud: {latitude}, Longitud: {longitude}, Altitud: {altitude}")

    except pynmea2.ParseError as e:
        print(f"Error al decodificar la trama NMEA: {e}")

try:
    while True:
        # Leer una línea desde el puerto serie
        sentence = ser.readline().decode('latin-1').strip()
        
        # Decodificar y procesar la trama NMEA
        decode_nmea_sentence(sentence)

except KeyboardInterrupt:
    # Manejar interrupción de teclado (Ctrl+C)
    ser.close()
    print("Programa terminado.")
