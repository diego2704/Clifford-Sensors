import customtkinter as ctk
import tkinter as tk
from tkintermapview import TkinterMapView
import cv2
import threading
import numpy as np
from PIL import Image, ImageTk

class SensorInterface(ctk.CTk):
    def __init__(self):
        super().__init__()

        # Configuración de la ventana principal
        self.title("Monitor de Sensores del Perro Robótico")
        self.geometry("800x600")

        # Tema y colores
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")

        # Sección de título principal
        title_label = ctk.CTkLabel(self, text="Monitor de Sensores", font=ctk.CTkFont(size=24, weight="bold"))
        title_label.pack(pady=10)

        # Contenedor para agrupar las secciones
        container_frame = ctk.CTkFrame(self)
        container_frame.pack(pady=10, padx=20, fill="both", expand=True)

        # Sección de sensores
        self.create_sensor_sections(container_frame)

        # Botón para abrir la cámara
        camera_button = ctk.CTkButton(self, text="Mostrar Cámara", command=self.open_camera_window)
        camera_button.pack(pady=10)

        # Botón para abrir el mapa
        map_button = ctk.CTkButton(self, text="Mostrar Mapa", command=self.open_map_window)
        map_button.pack(pady=10)

        # Sección para comandos de voz
        self.voice_command_label = ctk.CTkLabel(self, text="Comandos de Voz:", font=ctk.CTkFont(size=18))
        self.voice_command_label.pack(pady=10)

        self.voice_command_display = ctk.CTkTextbox(self, width=600, height=100)
        self.voice_command_display.pack(pady=10)

    def create_sensor_sections(self, container_frame):
        # Sección para sensores
        sensors_label = ctk.CTkLabel(container_frame, text="Datos de Sensores", font=ctk.CTkFont(size=18))
        sensors_label.pack(pady=10)

        # Ejemplo de etiquetas para sensores
        gas_sensor_label = ctk.CTkLabel(container_frame, text="Sensor de Gas: OK")
        gas_sensor_label.pack(pady=5)

        voltage_sensor_label = ctk.CTkLabel(container_frame, text="Sensor de Voltaje: 12V")
        voltage_sensor_label.pack(pady=5)

        current_sensor_label = ctk.CTkLabel(container_frame, text="Sensor de Corriente: 2A")
        current_sensor_label.pack(pady=5)

    def open_camera_window(self):
        self.camera_window = tk.Toplevel(self)
        self.camera_window.title("Cámara")
        self.camera_window.geometry("640x480")

        # Etiqueta para mostrar la imagen de la cámara
        self.camera_label = ctk.CTkLabel(self.camera_window)
        self.camera_label.pack(fill="both", expand=True)

        # Iniciar el hilo para mostrar la imagen de la cámara
        threading.Thread(target=self.show_camera_feed, daemon=True).start()

    def show_camera_feed(self):
        # Captura de video de la cámara local
        cap = cv2.VideoCapture(0)

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            # Convertir la imagen de OpenCV a formato compatible con tkinter
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = Image.fromarray(frame)
            photo = ImageTk.PhotoImage(image=img)

            # Actualizar la etiqueta de la cámara
            if hasattr(self, 'camera_label'):
                self.camera_label.configure(image=photo)
                self.camera_label.image = photo  # Mantener una referencia a la imagen

        cap.release()

    def open_map_window(self):
        self.map_window = tk.Toplevel(self)
        self.map_window.title("Mapa")
        self.map_window.geometry("800x600")

        # Widget del mapa
        map_widget = TkinterMapView(self.map_window, width=800, height=600, corner_radius=0)
        map_widget.pack(fill="both", expand=True)

        map_widget.set_tile_server("https://mt0.google.com/vt/lyrs=m&hl=en&x={x}&y={y}&z={z}&s=Ga", max_zoom=22)


# Ejecutar la aplicación
if __name__ == "__main__":
    app = SensorInterface()
    app.mainloop()
