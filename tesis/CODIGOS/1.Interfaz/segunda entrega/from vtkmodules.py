import vtk
import math

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
        self.render_window.Render()

    def start(self):
        """Inicia el interactor y muestra la ventana"""
        self.interactor.Initialize()
        self.interactor.Start()

if __name__ == "__main__":
    viewer = STLViewer("cube.stl")  # Reemplaza con la ruta de tu archivo STL
    
    # Aquí envías los valores de rotación en grados, entre 0 y 180.
    viewer.apply_rotation(90, 90, 90)  # Aplicar rotaciones en grados (roll, pitch, yaw)
    
    viewer.start()
