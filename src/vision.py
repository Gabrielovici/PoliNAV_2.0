from ultralytics import YOLO
import cv2
import numpy as np

# Incarcam modelul o singura data, cand porneste programul
# Asigura-te ca 'best.pt' este langa main.py!
print("Incarc modelul neuronal (best.pt)...")
try:
    model = YOLO("best.pt")
except Exception as e:
    print(f"[EROARE CRITICA] Nu gasesc fisierul 'best.pt'! {e}")
    model = None


def process_camera(img_raw, res):
    """
    Converteste imaginea din CoppeliaSim in format OpenCV.
    """
    img_np = np.frombuffer(img_raw, dtype=np.uint8)
    img_np = img_np.reshape((res[1], res[0], 3))

    # Flip vertical (Coppelia vede rasturnat) si conversie RGB->BGR
    img = cv2.flip(img_np, 0)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    return img


def detect_objects(img):
    """
    Foloseste YOLOv8 pentru a detecta obiectele din imagine.
    Returneaza o lista de dictionare cu ce a vazut.
    """
    if model is None:
        return []

    # Facem predictia (conf=0.5 inseamna ca trebuie sa fie 50% sigur)
    results = model.predict(img, conf=0.5, verbose=False)

    detected_objects = []

    # Iteram prin rezultate (YOLO poate returna mai multe frame-uri, noi avem doar unul)
    for r in results:
        boxes = r.boxes

        for box in boxes:
            # Coordonate cutie (x1, y1, x2, y2)
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

            # Clasa (ID si Nume)
            cls_id = int(box.cls[0])
            cls_name = model.names[cls_id]

            # Incredere (cat de sigur e robotul)
            conf = float(box.conf[0])

            # Calculam centrul si dimensiunile pentru logica de navigare
            x, y = int(x1), int(y1)
            w, h = int(x2 - x1), int(y2 - y1)
            center_x = x + w // 2
            center_y = y + h // 2

            # Adaugam in lista pentru main.py
            obj_info = {
                'name': cls_name,  # Ex: "fotoliu", "persoana"
                'conf': conf,  # Ex: 0.85
                'box': (x, y, w, h),  # Pozitia pe ecran
                'center': (center_x, center_y),
                'area': w * h  # Cat de aproape e (aprox)
            }
            detected_objects.append(obj_info)

            # --- DESENARE PE IMAGINE (OPTIONAL) ---
            # Desenam direct aici ca sa vedem ce vede robotul
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(img, f"{cls_name} {conf:.2f}", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    return detected_objects