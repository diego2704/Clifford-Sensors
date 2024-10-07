import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sounddevice as sd
from pydub import AudioSegment
import speech_recognition as sr

class AudioPublisher(Node):
    def __init__(self):
        super().__init__('audio_publisher')
        self.publisher_ = self.create_publisher(String, 'audio_text', 10)
        self.sample_rate = 44100
        self.duration = 5  # Duración de la grabación en segundos
        self.recognizer = sr.Recognizer()
        self.timer = self.create_timer(6, self.timer_callback)  # Publica cada 6 segundos
        self.get_logger().info("Nodo de audio inicializado. Grabando cada 5 segundos.")

    def timer_callback(self):
        try:
            # Inicia la grabación de audio
            self.get_logger().info(f"Grabando durante {self.duration} segundos...")
            audio_data = sd.rec(int(self.sample_rate * self.duration), samplerate=self.sample_rate, channels=1, dtype='int16')
            sd.wait()  # Espera a que la grabación finalice

            # Convierte el audio a formato WAV
            audio_segment = AudioSegment(
                audio_data.tobytes(),
                frame_rate=self.sample_rate,
                sample_width=audio_data.dtype.itemsize,
                channels=1
            )
            audio_segment.export("temp.wav", format="wav")

            # Reconoce el habla desde el archivo temporal
            with sr.AudioFile("temp.wav") as source:
                audio_text = self.recognizer.record(source)
                text = self.recognizer.recognize_google(audio_text)
                self.get_logger().info(f"Texto reconocido: {text}")

                # Publica el texto en el tópico 'audio_text'
                msg = String()
                msg.data = text
                self.publisher_.publish(msg)

        except sr.UnknownValueError:
            self.get_logger().info("No se pudo entender el audio.")
        except sr.RequestError as e:
            self.get_logger().error(f"Error en la solicitud de reconocimiento de habla: {e}")
        except Exception as e:
            self.get_logger().error(f"Error inesperado: {e}")

def main(args=None):
    rclpy.init(args=args)
    audio_publisher = AudioPublisher()

    try:
        rclpy.spin(audio_publisher)
    except KeyboardInterrupt:
        audio_publisher.get_logger().info("Nodo detenido por el usuario.")
    finally:
        audio_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
