import cv2
from ultralytics import YOLO

def main():
    # Abre la cámara
    cap = cv2.VideoCapture('/dev/video0')
    if not cap.isOpened():
        print("No se pudo abrir la cámara")
        return

    # Carga el modelo YOLOv8n preentrenado
    yolo_model = YOLO('yolov8n.pt')

    # Establece la resolución deseada de la cámara
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    procesar_cada_n_frame = 3  # Ajusta según sea necesario
    contador_frame = 0

    while True:
        # Captura un frame de la cámara
        ret, frame = cap.read()
        if not ret:
            print("Error al capturar el frame")
            break

        # Incrementa el contador de frames
        contador_frame += 1

        # Procesa solo cada N-ésimo frame
        if contador_frame % procesar_cada_n_frame == 0:
            # Imprime el tamaño del frame antes de la detección
            print("Tamaño del frame antes de la detección:", frame.shape)

            # Realiza la detección de objetos usando YOLOv8n
            results = yolo_model(frame)

            # Imprime el tamaño del frame después de la detección
            print("Tamaño del frame después de la detección:", frame.shape)

            # Muestra el frame en la ventana
            cv2.imshow("Video", frame)

        # Comprueba si el usuario quiere salir
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    # Libera la cámara y cierra todas las ventanas
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
