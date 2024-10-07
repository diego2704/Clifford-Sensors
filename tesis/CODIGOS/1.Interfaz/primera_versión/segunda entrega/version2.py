import customtkinter as ctk
import tkinter as tk
from PIL import Image, ImageTk
import vtk
from vtkmodules.vtkRenderingCore import vtkRenderer, vtkRenderWindow, vtkRenderWindowInteractor
from vtkmodules.vtkIOGeometry import vtkSTLReader
from tkintermapview import TkinterMapView
import math
import vtk
import math
from PIL import Image, ImageTk

class STLViewer:
    def __init__(self, filename):
        # Cargar el archivo STL
        self.reader = vtk.vtkSTLReader()
        self.reader.SetFileName(filename)  # Reemplaza con la ruta de tu archivo STL
        self.reader.Update()

        # Crear un mapper para el archivo STL
        self.mapper = vtk.vtkPolyDataMapper()
        self.mapper.SetInputConnection(self.reader.GetOutputPort())

        # Crear un actor para el archivo STL
        self.actor = vtk.vtkActor()
        self.actor.SetMapper(self.mapper)

        # Crear un renderizador para la ventana
        self.renderer = vtk.vtkRenderer()
        self.renderer.AddActor(self.actor)
        self.renderer.SetBackground(0.1, 0.1, 0.1)

        # Crear una ventana para mostrar el renderizador
        self.render_window = vtk.vtkRenderWindow()
        self.render_window.AddRenderer(self.renderer)

        # Crear un interactor para la ventana
        self.interactor = vtk.vtkRenderWindowInteractor()
        self.interactor.SetRenderWindow(self.render_window)

    def scale_to_internal_range(self, degrees):
        """
        Escala un valor de grados (0-180) al rango interno (0-10300).
        """
        return (degrees / 180) * 10300

    def apply_rotation(self, roll, pitch, yaw):
        """
        Aplica rotaciones al modelo en función de roll, pitch y yaw en grados.
        Los valores en grados (0 a 180) se convierten al rango interno de 0 a 10300.
        """
        # Escalar los valores de grados (0-180) a su equivalente interno (0-10300)
        roll_internal = self.scale_to_internal_range(roll)
        pitch_internal = self.scale_to_internal_range(pitch)
        yaw_internal = self.scale_to_internal_range(yaw)

        # Convertir grados escalados a radianes
        roll_rad = math.radians(roll_internal)
        pitch_rad = math.radians(pitch_internal)
        yaw_rad = math.radians(yaw_internal)

        # Crear un transformador
        transform = vtk.vtkTransform()

        # Aplicar las rotaciones en el orden correcto
        transform.RotateZ(yaw_rad)   # Primero yaw
        transform.RotateY(pitch_rad)  # Luego pitch
        transform.RotateX(roll_rad)   # Por último roll

        # Aplicar la transformación al actor
        self.actor.SetUserTransform(transform)

        # Centrar el modelo (opcional)
        bounds = self.actor.GetBounds()
        center_x = (bounds[0] + bounds[1]) / 2
        center_y = (bounds[2] + bounds[3]) / 2
        center_z = (bounds[4] + bounds[5]) / 2

        self.actor.SetPosition(-center_x, -center_y, -center_z)  # Centrar el modelo en el origen
        
        # Renderizar la ventana después de la transformación
        self.render_window.Render()  # Asegúrate de renderizar la ventana después de aplicar la transformación


class RobotMonitor(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.viewer = STLViewer("POCHITA_v30.stl")  # Inicializa el visualizador STL


        # Configuración de la ventana principal
        self.title("Monitor de Sensores del Perro Robótico")
        self.geometry("1000x800")

        # Tema y colores
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")

        # Sección de título principal
        title_label = ctk.CTkLabel(self, text="Monitor de Sensores", font=ctk.CTkFont(size=24, weight="bold"))
        title_label.pack(pady=10)

        # Contenedor principal
        container_frame = ctk.CTkFrame(self)
        container_frame.pack(pady=10, padx=20, fill="both", expand=True)

        # Menú desplegable para los botones
        self.create_side_buttons(container_frame)

        # Sección del IMU
        self.create_imu(container_frame)

        # Sección de monitoreo de sensores
        self.create_sensor_sections(container_frame)

        # Sección para los sensores de fuerza
        self.create_force(container_frame)

        # Sección de comandos de voz
        self.create_voice_section()
        


    

    def create_side_buttons(self, parent):
        """Crea un menú desplegable para Cámara, Mapa y Simulación"""
        self.side_frame = ctk.CTkFrame(parent)
        self.side_frame.pack(side="top", anchor="nw", padx=(10, 0), pady=(10, 0), fill="x")

        # Botón para expandir/colapsar el menú (icono de tres rayitas)
        self.toggle_button = ctk.CTkButton(self.side_frame, text="☰", command=self.toggle_menu, width=10)
        self.toggle_button.pack(padx=10, pady=(5, 0))

        # Frame para los botones, inicialmente oculto
        self.button_frame = ctk.CTkFrame(self.side_frame)
        self.button_frame.pack(padx=10, pady=(5, 10))

        # Cargar y redimensionar la imagen
        camara_image_path = "camara-reflex-digital.png"  # Reemplaza con la ruta de tu imagen
        camara_image = Image.open(camara_image_path)
        camara_image = camara_image.resize((50, 50), Image.ANTIALIAS)  # Ajustar el tamaño de la imagen según sea necesario
        self.camara_image = ImageTk.PhotoImage(camara_image)  # Guardar una referencia para evitar que sea recolectada por el GC

        # Cargar y redimensionar la imagen
        mapa_image_path = "mapas.png"  # Reemplaza con la ruta de tu imagen
        mapa_image = Image.open(mapa_image_path)
        mapa_image = mapa_image.resize((50, 50), Image.ANTIALIAS)  # Ajustar el tamaño de la imagen según sea necesario
        self.mapa_image = ImageTk.PhotoImage(mapa_image)  # Guardar una referencia para evitar que sea recolectada por el GC

        # Cargar y redimensionar la imagen
        rviz_image_path = "rviz.png"  # Reemplaza con la ruta de tu imagen
        rviz_image = Image.open(rviz_image_path)
        rviz_image = rviz_image.resize((50, 50), Image.ANTIALIAS)  # Ajustar el tamaño de la imagen según sea necesario
        self.rviz_image = ImageTk.PhotoImage(rviz_image)  # Guardar una referencia para evitar que sea recolectada por el GC

        # Botones para abrir cámara, mapa y simulación con imágenes
        camera_button = ctk.CTkButton(self.button_frame, text="Cámara", command=self.open_camera_window, image=self.camara_image, compound="right")
        camera_button.pack(padx=30, pady=5)

        map_button = ctk.CTkButton(self.button_frame, text="Mapa", command=self.open_map_window, image=self.mapa_image, compound="right")
        map_button.pack(padx=10, pady=5)

        sim_button = ctk.CTkButton(self.button_frame, text="Simulación", command=self.open_simulation_window, image=self.rviz_image, compound="right")
        sim_button.pack(padx=10, pady=5)

        # Ocultar el marco de botones inicialmente
        self.button_frame.pack_forget()

    def toggle_menu(self):
        """Alterna la visibilidad del marco de botones"""
        if self.button_frame.winfo_ismapped():
            self.button_frame.pack_forget()
        else:
            self.button_frame.pack(padx=10, pady=(5, 10))

    def create_imu(self, parent):
        """Crea la sección del IMU"""
        imu_frame = ctk.CTkFrame(parent)
        imu_frame.pack(side="left", anchor="sw", padx=(10, 10), pady=(10, 0), fill="both", expand=True)

        imu_label = ctk.CTkLabel(imu_frame, text="IMU", font=ctk.CTkFont(size=50, weight="bold"))
        imu_label.pack(pady=10)

        # Botón para abrir la visualización del IMU
        imu_button = ctk.CTkButton(imu_frame, text="Visualizar IMU", command=self.open_imu_visualization)
        imu_button.pack(side="left", padx=20)



        # Frame para la orientación
        orientation_frame = ctk.CTkFrame(imu_frame)
        orientation_frame.pack(side=ctk.LEFT, padx=(10, 10))  # Alinea a la izquierda

        # Sensores de orientación
        self.roll_label = ctk.CTkLabel(orientation_frame, text="roll:   9.6")
        self.roll_label.pack(padx=(50),pady=(30, 30))

        self.pitch_label = ctk.CTkLabel(orientation_frame, text="pitch:   5.6")
        self.pitch_label.pack(padx=(10, 10), pady=(10, 30))

        self.yaw_label = ctk.CTkLabel(orientation_frame, text="yaw:   5.4")
        self.yaw_label.pack(padx=(10, 10), pady=(10, 30))
        # Frame para la orientación
        rate_frame = ctk.CTkFrame(imu_frame)
        rate_frame.pack(side=ctk.LEFT, padx=(10, 10))  # Alinea a la izquierda

        # Sensores de tasas
        self.rateRoll_label = ctk.CTkLabel(rate_frame, text="rateRoll:   2.5")
        self.rateRoll_label.pack(padx=(50),pady=(30, 30))

        self.ratePitch_label = ctk.CTkLabel(rate_frame, text="ratePitch:   22")
        self.ratePitch_label.pack(padx=(10, 10), pady=(10, 30))

        self.rateYaw_label = ctk.CTkLabel(rate_frame, text="rateYaw:   554")
        self.rateYaw_label.pack(padx=(10, 10) ,pady=(10, 30))


    

    
    def open_imu_visualization(self):
        # Configura el modelo 3D
        self.setup_3d_model()  # Asegúrate de que este método no reinicie el modelo

        # Aplica las rotaciones deseadas
        self.viewer.apply_rotation(90, 90, 90)  # Ajusta según lo necesario

        # Crear un nuevo interactor para la ventana de IMU
        self.viewer.interactor.SetRenderWindow(self.viewer.render_window)  # Asignar la ventana al interactor

        # Iniciar la interacción
        self.viewer.interactor.Initialize()  # Inicializar el interactor
        self.viewer.render_window.Render()  # Renderizar la ventana
        self.viewer.interactor.Start()  # Iniciar el interactor



    def create_sensor_sections(self, parent):
        """Crea la sección de sensores (Gas, Voltaje, Corriente, Touch)"""
        sensor_frame = ctk.CTkFrame(parent)
        sensor_frame.pack(side="top", fill="both", padx=(5, 10), pady=(10, 0))

        sensor_label = ctk.CTkLabel(sensor_frame, text="Datos de Sensores", font=ctk.CTkFont(size=18, weight="bold"))
        sensor_label.pack(pady=(50, 0))

        # Sensores individuales
        self.gas_sensor_label = ctk.CTkLabel(sensor_frame, text="Gas: ...")
        self.gas_sensor_label.pack(padx=400 ,pady=5)

        self.voltage_sensor_label = ctk.CTkLabel(sensor_frame, text="Voltaje: ...")
        self.voltage_sensor_label.pack(pady=5)

        self.current_sensor_label = ctk.CTkLabel(sensor_frame, text="Corriente: ...")
        self.current_sensor_label.pack(pady=5)

        self.touch_sensor_label = ctk.CTkLabel(sensor_frame, text="Touch: ...")
        self.touch_sensor_label.pack(pady=5)

    
    def create_force(self, parent):
        """Crea la sección de IMU y sensores de fuerza"""
        force_frame = ctk.CTkFrame(parent)
        force_frame.pack(side="bottom", anchor="se", fill="both", padx=5, pady=(5, 5))  # Disminuir padding

        force_label = ctk.CTkLabel(force_frame, text="Sensores de Fuerza", font=ctk.CTkFont(size=18, weight="bold"))
        force_label.pack(pady=5)  # Mantener espacio entre el título y los valores

        # Cargar y redimensionar la imagen
        image_path = "perro.png"  # Reemplaza con la ruta de tu imagen
        image = Image.open(image_path)
        image = image.resize((200, 200), Image.ANTIALIAS)  # Ajustar el tamaño de la imagen según sea necesario
        self.image = ImageTk.PhotoImage(image)  # Guardar una referencia para evitar que sea recolectada por el GC

        # Crear un contenedor para la imagen y los valores
        content_frame = ctk.CTkFrame(force_frame)
        content_frame.pack(padx=(5, 0),pady=5)  # Reducir el espacio en el contenedor

        # Crear un label para la imagen y empaquetarlo a la izquierda
        image_label = ctk.CTkLabel(content_frame, image=self.image)
        image_label.pack(side=ctk.LEFT, padx=(10))  # Menos espacio entre la imagen y los valores

        # Crear un frame para los valores y empaquetarlo a la derecha
        values_frame = ctk.CTkFrame(content_frame)
        values_frame.pack(side=ctk.RIGHT)

        # Sensores de fuerza
        self.f1_label = ctk.CTkLabel(values_frame, text="F1:    0.5")
        self.f1_label.pack(padx=50,pady=2)  # Disminuir el espacio entre etiquetas

        self.f2_label = ctk.CTkLabel(values_frame, text="F2:   0.0")
        self.f2_label.pack(pady=2)

        self.f3_label = ctk.CTkLabel(values_frame, text="F3:   0.3")
        self.f3_label.pack(pady=2)

        self.f4_label = ctk.CTkLabel(values_frame, text="F4:   0.6")
        self.f4_label.pack(pady=2)

    def create_voice_section(self):
        """Crea la sección de comandos de voz"""
        voice_frame = ctk.CTkFrame(self)
        voice_frame.pack(pady=10, padx=20, fill="both", expand=True)

        voice_label = ctk.CTkLabel(voice_frame, text="Comandos de Voz", font=ctk.CTkFont(size=18, weight="bold"))
        voice_label.pack(pady=10)

        self.voice_command_display = ctk.CTkTextbox(voice_frame, width=800, height=100)
        self.voice_command_display.pack(pady=10)

    def setup_3d_model(self):
        """Configura el modelo 3D a partir de un archivo STL"""
        # Cargar el archivo STL
        reader = vtk.vtkSTLReader()
        reader.SetFileName("cube.stl")  # Reemplaza con la ruta de tu archivo STL
        reader.Update()

        # Crear un mapper para el archivo STL
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(reader.GetOutputPort())

        # Crear un actor para el archivo STL
        self.model_actor = vtk.vtkActor()
        self.model_actor.SetMapper(mapper)

        # Crear el renderizador
        self.renderer = vtk.vtkRenderer()
        self.renderer.AddActor(self.model_actor)
        self.renderer.SetBackground(0.1, 0.1, 0.1)  # Color de fondo oscuro

        # Crear la ventana de renderizado
        self.render_window = vtk.vtkRenderWindow()
        self.render_window.AddRenderer(self.renderer)

        # Crear el interactor y asignar la ventana de renderizado
        self.vtk_interactor = vtk.vtkRenderWindowInteractor()  # Crear el interactor
        # Configurar el interactor
        self.vtk_interactor.SetRenderWindow(self.render_window)

        # Crear un manejador de interacción estilo trackball (para rotar el modelo con el mouse)
        style = vtk.vtkInteractorStyleTrackballCamera()
        self.vtk_interactor.SetInteractorStyle(style)

    def open_camera_window(self):
        """Abre una nueva ventana para mostrar la cámara"""
        camera_window = ctk.CTkToplevel(self)
        camera_window.title("Cámara en vivo")
        camera_window.geometry("640x480")

        # Aquí podrías integrar el feed de la cámara usando ROS 2 u OpenCV

    def open_map_window(self):
        """Abre una nueva ventana para mostrar el mapa en tiempo real"""
        map_window = ctk.CTkToplevel(self)
        map_window.title("Mapa en tiempo real")
        map_window.geometry("800x600")

        # Cargar un mapa usando tkintermapview
        map_widget = TkinterMapView(map_window, width=800, height=600, corner_radius=0)
        map_widget.pack(fill="both", expand=True)
        map_widget.set_position(-12.046374, -77.042793)  # Coordenadas de ejemplo, puedes reemplazarlas con las del GPS
        map_widget.set_zoom(15)

    def open_simulation_window(self):
        """Abre una nueva ventana para la simulación"""
        sim_window = ctk.CTkToplevel(self)
        sim_window.title("Simulación 3D")
        sim_window.geometry("800x600")

        # Aquí puedes cargar una simulación si tienes una, o puedes mostrar otra información relacionada.

# Ejecutar la aplicación
if __name__ == "__main__":
    app = RobotMonitor()
    viewer = STLViewer("cube.stl")  # Reemplaza con la ruta de tu archivo STL
    app.mainloop()
