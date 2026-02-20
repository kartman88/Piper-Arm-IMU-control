import re
import time
import threading
import serial
import numpy as np
import pygame
from scipy.spatial.transform import Rotation as R

# -------------------------
# Parsing quaternioni UART
# -------------------------

FLOAT_RE = re.compile(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?")

def parse_quaternion(line, order="xyzw"):
    """
    Estrae 4 float da una riga e restituisce (qx,qy,qz,qw).
    order:
      - "xyzw" (default): qx qy qz qw
      - "wxyz":           qw qx qy qz
    """
    nums = FLOAT_RE.findall(line)
    if len(nums) < 4:
        return None

    vals = list(map(float, nums[:4]))

    if order == "xyzw":
        qx, qy, qz, qw = vals
    elif order == "wxyz":
        qw, qx, qy, qz = vals
    else:
        raise ValueError("order deve essere 'xyzw' o 'wxyz'")

    return (qx, qy, qz, qw)

def normalize_quat(q):
    q = np.array(q, dtype=float)
    n = np.linalg.norm(q)
    if n == 0:
        return None
    return tuple(q / n)

# -------------------------
# Condivisione dato tra thread/main
# -------------------------

class SharedQuat:
    def __init__(self):
        self.lock = threading.Lock()
        self.quat = None
        self.updated = False

    def update(self, quat):
        with self.lock:
            self.quat = quat
            self.updated = True

    def get(self):
        with self.lock:
            q = self.quat
            is_new = self.updated
            self.updated = False
        return q, is_new

# -------------------------
# Serial / Thread reader
# -------------------------

def open_serial(port="COM4", baud=115200, timeout=1):
    return serial.Serial(port, baud, timeout=timeout)

def uart_reader_thread(ser, shared, stop_event, order="xyzw"):
    """Thread che aggiorna solo l'ultimo quaternione valido."""
    while not stop_event.is_set():
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            quat = parse_quaternion(line, order=order)
            if quat is None:
                continue

            quat = normalize_quat(quat)
            if quat is None:
                continue

            shared.update(quat)

        except serial.SerialException as e:
            print(f"[UART] Serial error: {e}")
            break
        except Exception:
            pass

def start_uart_reader(ser, shared, order="xyzw"):
    stop_event = threading.Event()
    t = threading.Thread(
        target=uart_reader_thread,
        args=(ser, shared, stop_event, order),
        daemon=True
    )
    t.start()
    return stop_event, t

# -------------------------
# Cubo 3D + proiezione 2D
# -------------------------

def create_cube(size=1.0):
    """
    Ritorna vertices (8x3) e edges (lista coppie indici).
    Cubo centrato in (0,0,0).
    """
    s = size / 2.0
    vertices = np.array([
        [-s, -s, -s],
        [ s, -s, -s],
        [ s,  s, -s],
        [-s,  s, -s],
        [-s, -s,  s],
        [ s, -s,  s],
        [ s,  s,  s],
        [-s,  s,  s],
    ], dtype=float)

    edges = [
        (0,1),(1,2),(2,3),(3,0),
        (4,5),(5,6),(6,7),(7,4),
        (0,4),(1,5),(2,6),(3,7)
    ]
    return vertices, edges

def rotate_vertices(vertices, quat, frame_rot=None):
    """
    Ruota i vertici con SciPy a partire dal quaternione.
    frame_rot: opzionale rotazione di allineamento IMU->mondo.
    """
    qx, qy, qz, qw = quat
    rot = R.from_quat([qx, qy, qz, qw])
    if frame_rot is not None:
        rot = frame_rot * rot
    return rot.apply(vertices)

def project_vertices(vertices, width, height, fov=500, viewer_distance=3.0):
    """
    Proiezione prospettica semplice 3D -> 2D.
    """
    projected = []
    for x, y, z in vertices:
        z = z + viewer_distance
        if z == 0:
            z = 1e-6
        factor = fov / z
        x2d = x * factor + width / 2
        y2d = -y * factor + height / 2
        projected.append((int(x2d), int(y2d)))
    return projected

def draw_cube(screen, projected, edges, line_color=(0, 255, 255), thickness=2):
    for i, j in edges:
        pygame.draw.line(screen, line_color, projected[i], projected[j], thickness)

# -------------------------
# Pygame loop
# -------------------------

def init_pygame(width=900, height=600, title="IMU Quaternion Viewer"):
    pygame.init()
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption(title)
    clock = pygame.time.Clock()
    return screen, clock

def visualization_loop(shared, stop_event, fps=60, cube_size=1.5, frame_rot=None):
    screen, clock = init_pygame()
    width, height = screen.get_size()

    base_vertices, edges = create_cube(size=cube_size)
    last_quat = (0, 0, 0, 1)  # orientamento iniziale

    running = True
    while running and not stop_event.is_set():
        # Eventi pygame
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Se arriva un nuovo quaternione, aggiornalo
        q, is_new = shared.get()
        if is_new and q is not None:
            last_quat = q

        # Rotazione + proiezione
        rotated = rotate_vertices(base_vertices, last_quat, frame_rot=frame_rot)
        projected = project_vertices(rotated, width, height)

        # Disegno
        screen.fill((0, 0, 0))
        draw_cube(screen, projected, edges)
        pygame.display.flip()

        clock.tick(fps)

    pygame.quit()

# -------------------------
# Main
# -------------------------

def main(port="/dev/ttyUSB0", baud=115200, order="xyzw"): #COM4 Windows
    ser = open_serial(port, baud)
    shared = SharedQuat()
    stop_event, _ = start_uart_reader(ser, shared, order=order)

    try:
        # Se gli assi della IMU non coincidono con quelli del viewer,
        # qui puoi aggiungere una rotazione di correzione:
        frame_rot = None
        # esempio:
        # frame_rot = R.from_euler("x", 90, degrees=True)

        visualization_loop(shared, stop_event, fps=60, cube_size=1.5, frame_rot=frame_rot)

    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        ser.close()

if __name__ == "__main__":
    main()
