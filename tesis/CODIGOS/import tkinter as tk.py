import tkinter as tk
from tkinter import ttk

class AssemblyDiagramApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Smart Boxing Machine Assembly")
        self.geometry("600x400")

        # Etiqueta de título
        self.label = tk.Label(self, text="Assembly Diagram", font=("Arial", 18))
        self.label.pack(pady=10)

        # Canvas para dibujo del diagrama
        self.canvas = tk.Canvas(self, width=500, height=300, bg="white")
        self.canvas.pack(pady=20)

        # Dibujar componentes
        self.draw_components()

        # Botón para simular pasos
        self.next_step_button = ttk.Button(self, text="Next Step", command=self.next_step)
        self.next_step_button.pack(pady=10)

    def draw_components(self):
        # Dibujar base
        self.canvas.create_rectangle(100, 250, 400, 280, fill="lightgray", outline="black", width=2)
        self.canvas.create_text(250, 265, text="Base", font=("Arial", 10))

        # Dibujar PCB (placa base)
        self.canvas.create_rectangle(150, 180, 350, 210, fill="lightgreen", outline="black", width=2)
        self.canvas.create_text(250, 195, text="PCB", font=("Arial", 10))

        # Dibujar batería
        self.canvas.create_rectangle(180, 120, 320, 150, fill="yellow", outline="black", width=2)
        self.canvas.create_text(250, 135, text="Battery", font=("Arial", 10))

    def next_step(self):
        # Lógica para simular el siguiente paso en el ensamblaje
        # Por ejemplo, mostrar un componente adicional o una conexión
        pass

if __name__ == "__main__":
    app = AssemblyDiagramApp()
    app.mainloop()
