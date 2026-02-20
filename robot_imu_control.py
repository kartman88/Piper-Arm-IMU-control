import time
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R

# Importiamo il modulo IMU che abbiamo creato prima
# Assicurati che il file 'imu_euler_printer.py' sia nella stessa cartella
import sensor_box_to_euler as imu_module

# Import librerie Piper
from piper_control import piper_connect, piper_interface, piper_init

# ---------------------------------------------------------
# CLASSE GESTIONE ROBOT
# ---------------------------------------------------------
class PiperArm:
    def __init__(self, can_port="can0"):
        self.can_port = can_port
        self.robot = None
        self.limits = {}
        
    def connect(self):
        """Inizializza la connessione CAN e l'interfaccia robot."""
        print(f"[ROBOT] Ricerca porte CAN: {piper_connect.find_ports()}")
        piper_connect.activate()
        print(f"[ROBOT] Porta attiva: {piper_connect.active_ports()}")
        
        self.robot = piper_interface.PiperInterface(can_port=self.can_port)
        # Salva i limiti hardware
        self.limits = self.robot.joint_limits
        print(f"[ROBOT] Limiti caricati: {self.limits}")

    def enable(self):
        """Procedura di abilitazione e reset del braccio."""
        print("\n" + "!"*50)
        print("ATTENZIONE: IL BRACCIO VERRÀ ABILITATO E POTREBBE CADERE.")
        print("Assicurati di reggerlo se necessario.")
        print("!"*50)
        input("Premi INVIO per confermare e continuare...")

        piper_init.reset_arm(
            self.robot,
            arm_controller=piper_interface.ArmController.POSITION_VELOCITY,
            move_mode=piper_interface.MoveMode.JOINT,
        )
        print("[ROBOT] Reset completato. Braccio attivo.")

    def recorded_position(self):
        current_joints = self.robot.get_joint_positions()
        current_joints[0] = 1.5
        current_joints[2] = -2.9
        
        # Invia comando
        self.robot.command_joint_positions(current_joints)

    def get_joints(self):
        return self.robot.get_joint_positions()

    def set_joint_safe(self, joint_index, joint_index_2, value_rad, value_rad_2):
        """Imposta un giunto verificando i limiti hardware."""
        # Recupera posizione attuale di tutti i giunti
        current_joints = self.robot.get_joint_positions()
        
        # Recupera limiti per questo giunto specifico
        min_limit = self.limits['min'][joint_index]
        max_limit = self.limits['max'][joint_index]
        min_limit_2 = self.limits['min'][joint_index_2]
        max_limit_2 = self.limits['max'][joint_index_2]
        
        # Clamping (sicurezza hardware)
        safe_val = max(min_limit, min(value_rad, max_limit))
        safe_val_2 = max(min_limit_2, min(value_rad_2, max_limit_2))
        
        # Aggiorna solo il giunto desiderato
        current_joints[joint_index] = safe_val
        current_joints[joint_index_2] = safe_val_2
        
        # Invia comando
        self.robot.command_joint_positions(current_joints)
        return safe_val

    def stop(self):
        # Implementa qui logiche di spegnimento se necessarie
        pass

# ---------------------------------------------------------
# CLASSE GESTIONE IMU CON TARATURA
# ---------------------------------------------------------
class IMUHandler:
    def __init__(self, port, baud=115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self.shared = None
        self.thread_stop = None
        self.zero_quat = None # Quaternione di riferimento (Tara)

    def start(self):
        """Avvia la connessione seriale e il thread di lettura."""
        try:
            self.ser = imu_module.open_serial(self.port, self.baud)
            self.shared = imu_module.SharedQuat()
            self.thread_stop, _ = imu_module.start_uart_reader(self.ser, self.shared)
            print(f"[IMU] Connesso su {self.port}")
            # Attende il primo dato valido
            print("[IMU] In attesa di dati validi...")
            while True:
                q, _ = self.shared.get()
                if q is not None:
                    break
                time.sleep(0.1)
        except Exception as e:
            print(f"[IMU] Errore critico: {e}")
            sys.exit(1)

    def calibrate(self):
        """Imposta la posizione attuale come 'Zero'."""
        print("[IMU] Calibrazione in corso... Tieni il sensore fermo.")
        time.sleep(1) # Aspetta che si stabilizzi
        q, _ = self.shared.get()
        if q is not None:
            # Salviamo la rotazione attuale come oggetto Rotation di Scipy
            self.zero_rot = R.from_quat(q)
            print("[IMU] Calibrazione completata! Questa posizione è ora 0°.")
            return True
        return False

    def get_relative_euler(self):
        """
        Restituisce (roll, pitch, yaw) relativi alla taratura iniziale.
        Usa la matematica dei quaternioni per calcolare la differenza.
        """
        q, is_new = self.shared.get()
        if q is None:
            return None, False

        current_rot = R.from_quat(q)
        
        if self.zero_rot is None:
            # Se non calibrato, ritorna assoluto
            rel_rot = current_rot
        else:
            # Calcola la rotazione relativa: R_rel = R_zero_inv * R_curr
            rel_rot = self.zero_rot.inv() * current_rot

        # Convertiamo in Eulero (gradi)
        # Usa 'zyx' per avere Yaw come primo elemento, o adatta all'orientamento del sensore
        return rel_rot.as_euler('zyx', degrees=True), is_new

    def close(self):
        if self.thread_stop:
            self.thread_stop.set()
        if self.ser:
            self.ser.close()

# ---------------------------------------------------------
# UTILS
# ---------------------------------------------------------
def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def clamp(val, min_val, max_val):
    return max(min_val, min(val, max_val))

# ---------------------------------------------------------
# MAIN
# ---------------------------------------------------------
def main():
    # --- CONFIGURAZIONE UTENTE ---
    IMU_PORT = "/dev/ttyUSB0"
    JOINT_TO_CONTROL = 0  # 0=Base, 1=Spalla, etc.
    JOINT_TO_CONTROL_2 = 2
    
    # Range di movimento dell'operatore (Gradi IMU relativi allo zero)
    # Esempio: Ruotando la mano di +/- 45 gradi...
    INPUT_RANGE_DEG = 90.0 
    
    # ...il robot si muoverà di +/- X radianti dalla posizione in cui si trova
    OUTPUT_RANGE_RAD = 1.0 
    OUTPUT_RANGE_RAD_2 = 1.5 
    
    SMOOTHING = 0.1
    # -----------------------------

    # 1. Inizializzazione Oggetti
    arm = PiperArm(can_port="can0")
    imu = IMUHandler(port=IMU_PORT)

    try:
        # 2. Setup Hardware
        arm.connect()
        arm.enable()
        arm.recorded_position()
        imu.start()

        # 3. Fase di Taratura
        print("\n" + "="*40)
        print("FASE DI TARATURA")
        print("Metti il sensore IMU nella posizione 'neutra' comoda.")
        input("Premi INVIO per TARARE lo zero...")
        if imu.calibrate():
            print("Taratura OK.")
        else:
            print("Errore taratura, riavviare.")
            return

        # Recuperiamo la posizione attuale del giunto del robot
        # La useremo come "centro" del movimento
        start_joint_pos = arm.get_joints()[JOINT_TO_CONTROL]
        smoothed_val = start_joint_pos
        start_joint_pos_2 = arm.get_joints()[JOINT_TO_CONTROL_2]
        smoothed_val_2 = start_joint_pos_2
        
        print("\n" + "="*40)
        print(f"CONTROLLO AVVIATO su Joint {JOINT_TO_CONTROL}")
        print(f"Posizione Robot Neutra: {start_joint_pos:.2f} rad")
        print(f"Muovi l'IMU tra -{INPUT_RANGE_DEG}° e +{INPUT_RANGE_DEG}°")
        print("CTRL+C per uscire.")

        # 4. Control Loop
        while True:
            euler, is_new = imu.get_relative_euler()
            
            if is_new and euler is not None:
                # euler[0] è Yaw (assumendo sequenza 'zyx' in get_relative_euler)
                # Nota: scambia indice se il tuo yaw è su un altro asse
                yaw_relative = euler[0] 
                pitch_relative = euler[2]

                # Limitiamo l'input ai gradi scelti dall'utente (es. +/- 45)
                yaw_clamped = clamp(yaw_relative, -INPUT_RANGE_DEG, INPUT_RANGE_DEG)
                pitch_clamped = clamp(pitch_relative, -INPUT_RANGE_DEG, INPUT_RANGE_DEG)
                
                # Mappiamo:
                # -INPUT_RANGE -> (StartPos - OUT_RANGE)
                # 0            -> StartPos
                # +INPUT_RANGE -> (StartPos + OUT_RANGE)
                target_offset = map_value(yaw_clamped, 
                                          -INPUT_RANGE_DEG, INPUT_RANGE_DEG, 
                                          -OUTPUT_RANGE_RAD, OUTPUT_RANGE_RAD)
                target_offset_2 = map_value(pitch_clamped, 
                                          -INPUT_RANGE_DEG, INPUT_RANGE_DEG, 
                                          -OUTPUT_RANGE_RAD_2, OUTPUT_RANGE_RAD_2)
                
                target_absolute = start_joint_pos + target_offset
                target_absolute_2 = start_joint_pos_2 + target_offset_2

                # Filtro esponenziale (Smoothing)
                smoothed_val = (smoothed_val * (1 - SMOOTHING)) + (target_absolute * SMOOTHING)
                smoothed_val_2 = (smoothed_val_2 * (1 - SMOOTHING)) + (target_absolute_2 * SMOOTHING)

                # Comando al robot (la funzione set_joint_safe controlla i limiti hardware)
                real_val = arm.set_joint_safe(JOINT_TO_CONTROL, JOINT_TO_CONTROL_2, smoothed_val, smoothed_val_2)

                # Feedback visivo
                sys.stdout.write(f"\rIMU YAW: {yaw_relative:6.1f}° | IMU PITCH: {yaw_relative:6.1f}° | Tgt: {target_absolute:5.2f} | Act: {real_val:5.2f}")
                sys.stdout.flush()

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nInterruzione utente.")
    except Exception as e:
        print(f"\nErrore imprevisto: {e}")
    finally:
        print("\nChiusura connessioni...")
        imu.close()
        # arm.stop() # Se c'è una logica di stop

if __name__ == "__main__":
    main()