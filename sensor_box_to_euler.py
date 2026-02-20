import re
import time
import threading
import serial
import numpy as np
import sys
from scipy.spatial.transform import Rotation as R

# -------------------------
# Configurazione
# -------------------------
SERIAL_PORT = "/dev/ttyUSB0"  # Cambia con la tua porta (es. /dev/ttyUSB0 su Linux/Mac)
BAUD_RATE = 115200
QUAT_ORDER = "xyzw"   # Ordine dei dati in arrivo dalla seriale

# -------------------------
# Parsing quaternioni UART
# -------------------------

FLOAT_RE = re.compile(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?")

def parse_quaternion(line, order="xyzw"):
    """
    Estrae 4 float da una riga e restituisce (qx,qy,qz,qw).
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

def open_serial(port, baud, timeout=1):
    return serial.Serial(port, baud, timeout=timeout)

def uart_reader_thread(ser, shared, stop_event, order="xyzw"):
    """Thread che legge continuamente dalla seriale."""
    print(f"[UART] In ascolto su {ser.port} a {ser.baudrate} baud...")
    while not stop_event.is_set():
        try:
            # Legge una riga e decodifica
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                continue

            # Parsing
            quat = parse_quaternion(line, order=order)
            if quat is None:
                continue

            # Normalizzazione
            quat = normalize_quat(quat)
            if quat is None:
                continue

            # Aggiorna il dato condiviso
            shared.update(quat)

        except serial.SerialException as e:
            print(f"\n[UART] Errore Serial: {e}")
            break
        except Exception as e:
            print(f"\n[UART] Errore generico: {e}")

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
# Loop di Stampa (Senza Pygame)
# -------------------------

def print_loop(shared, stop_event):
    """Loop principale che converte e stampa."""
    print("[MAIN] Avvio stampa angoli di Eulero (CTRL+C per uscire)...")
    print(f"{'ROLL':>10} | {'PITCH':>10} | {'YAW':>10}")
    print("-" * 36)

    try:
        while not stop_event.is_set():
            # Recupera l'ultimo quaternione disponibile
            q, is_new = shared.get()
            
            if is_new and q is not None:
                # Conversione Quaternione -> Eulero (Gradi)
                # 'xyz' è la sequenza standard di rotazione (puoi cambiarla in 'zxy', etc. se necessario)
                rot = R.from_quat(q)
                euler = rot.as_euler('xyz', degrees=True)
                
                roll, pitch, yaw = euler
                
                # Stampa formattata sulla stessa riga (sovrascrittura)
                # \r riporta il cursore a inizio riga
                sys.stdout.write(f"\r{roll:10.2f} | {pitch:10.2f} | {yaw:10.2f}")
                sys.stdout.flush()
            
            # Un piccolo sleep per non saturare la CPU (non dobbiamo renderizzare grafica)
            time.sleep(0.02) 

    except KeyboardInterrupt:
        print("\n[MAIN] Interruzione rilevata.")

# -------------------------
# Main
# -------------------------

def main():
    try:
        ser = open_serial(SERIAL_PORT, BAUD_RATE)
    except serial.SerialException as e:
        print(f"Impossibile aprire la porta {SERIAL_PORT}: {e}")
        return

    shared = SharedQuat()
    stop_event, reader_thread = start_uart_reader(ser, shared, order=QUAT_ORDER)

    try:
        print_loop(shared, stop_event)
    finally:
        print("\n[MAIN] Chiusura connessione...")
        stop_event.set()
        ser.close()
        reader_thread.join(timeout=1)
        print("[MAIN] Terminato.")

if __name__ == "__main__":
    main()