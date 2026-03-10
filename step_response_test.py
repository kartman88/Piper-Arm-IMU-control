# real_step_response.py

import time
import sys
import csv
import threading

# Import Piper libraries
from piper_control import piper_connect, piper_interface, piper_init

# ---------------------------------------------------------
# ROBOT MANAGEMENT CLASS (Based on your implementation)
# ---------------------------------------------------------
class PiperArm:
    def __init__(self, can_port="can0"):
        self.can_port = can_port
        self.robot = None
        self.limits = {}
        
    def connect(self):
        """Initializes CAN connection and robot interface."""
        print(f"[ROBOT] Searching for CAN ports: {piper_connect.find_ports()}")
        piper_connect.activate()
        print(f"[ROBOT] Active port: {piper_connect.active_ports()}")
        
        self.robot = piper_interface.PiperInterface(can_port=self.can_port)
        # Save hardware limits
        self.limits = self.robot.joint_limits
        print(f"[ROBOT] Limits loaded: {self.limits}")

    def enable(self):
        """Enable and reset procedure for the arm."""
        print("\n" + "!"*50)
        print("WARNING: THE ARM WILL BE ENABLED AND MIGHT DROP/MOVE.")
        print("Make sure to hold it if necessary or clear the area.")
        print("!"*50)
        input("Press ENTER to confirm and continue...")

        piper_init.reset_arm(
            self.robot,
            arm_controller=piper_interface.ArmController.POSITION_VELOCITY,
            move_mode=piper_interface.MoveMode.JOINT,
        )
        print("[ROBOT] Reset complete. Arm is active.")

    def set_home_position(self):
        """Sets the arm to the stretched out 'home' position."""
        current_joints = self.robot.get_joint_positions()
        # Your custom home pose for balancing
        current_joints[0] = 0.0   # Base
        current_joints[2] = -2.9  # Elbow (stretched)
        
        print(f"[ROBOT] Moving to HOME position: {current_joints}")
        self.robot.command_joint_positions(current_joints)
        
        # Wait a bit for the arm to physically reach the home position
        time.sleep(3.0)

    def get_joints(self):
        return self.robot.get_joint_positions()
        
    def get_joint_velocities(self):
        return self.robot.get_joint_velocities()

    def set_joint_safe(self, joint_index, value_rad):
        """Sets a single joint verifying hardware limits."""
        current_joints = self.robot.get_joint_positions()
        
        min_limit = self.limits['min'][joint_index]
        max_limit = self.limits['max'][joint_index]
        
        # Clamping (Hardware Safety)
        safe_val = max(min_limit, min(value_rad, max_limit))
        
        current_joints[joint_index] = safe_val
        self.robot.command_joint_positions(current_joints)
        
        return safe_val

# ---------------------------------------------------------
# GLOBAL VARIABLES FOR THREADING
# ---------------------------------------------------------
start_movement = False

def wait_for_enter():
    """Runs in background waiting for ENTER to trigger the step."""
    global start_movement
    input("\n[WAITING] Press ENTER to trigger the step response...\n")
    start_movement = True

# ---------------------------------------------------------
# MAIN
# ---------------------------------------------------------
def main():
    # --- USER CONFIGURATION ---
    JOINT_TO_CONTROL = 0  # 0=Base joint
    TARGET_OFFSET_RAD = 0.25  # The physical offset we want to achieve (e.g. 1.0 * action_scale)
    RECORD_TIME_S = 3.0  # How long to record data after the step
    # -----------------------------

    arm = PiperArm(can_port="can0")

    try:
        # 1. Hardware Setup
        arm.connect()
        arm.enable()
        
        # Move to home position (-2.9 on joint 3, etc.)
        arm.set_home_position()

        # Get the actual resting position to use as our "Zero"
        start_joint_pos = arm.get_joints()[JOINT_TO_CONTROL]
        target_absolute = start_joint_pos + TARGET_OFFSET_RAD
        
        print("\n" + "="*40)
        print(f"STEP RESPONSE TEST READY")
        print(f"Target Joint: {JOINT_TO_CONTROL}")
        print(f"Initial Position: {start_joint_pos:.3f} rad")
        print(f"Target Position: {target_absolute:.3f} rad")
        
        # 2. Start background listener
        threading.Thread(target=wait_for_enter, daemon=True).start()

        # Wait loop (doing nothing but maintaining position)
        while not start_movement:
            time.sleep(0.01)

        # 3. TRIGGER! Send the command
        print("\n[INFO] COMMAND SENT! Recording data...")
        # Note: We don't use smoothing here because it's a STEP response test!
        arm.set_joint_safe(JOINT_TO_CONTROL, target_absolute)

        # 4. Data Recording Loop
        data_log = []
        start_time = time.perf_counter()
        
        while True:
            current_time = time.perf_counter() - start_time
            
            if current_time > RECORD_TIME_S:
                break
                
            # Read current physical state
            actual_pos = arm.get_joints()[JOINT_TO_CONTROL]
            actual_vel = arm.get_joint_velocities()[JOINT_TO_CONTROL]
            
            data_log.append({
                "time_s": current_time,
                "position_rad": actual_pos,
                "velocity_rad_s": actual_vel,
                "target_rad": target_absolute
            })
            
            # ~100Hz control loop
            time.sleep(0.01)
            
        print("\n[RESULT] Test finished. Saving data...")
        
        # 5. Save Data to CSV
        csv_filename = "real_step_response.csv"
        with open(csv_filename, mode='w', newline='') as file:
            writer = csv.DictWriter(file, fieldnames=["time_s", "position_rad", "velocity_rad_s", "target_rad"])
            writer.writeheader()
            writer.writerows(data_log)
            
        print(f"[SUCCESS] Data saved to {csv_filename}")
        print(f"[INFO] Final resting position was: {data_log[-1]['position_rad']:.3f} rad")
        
        # (Dopo aver salvato il CSV...)
        final_pos = data_log[-1]['position_rad']
        error = target_absolute - final_pos
        
        # --- CALCOLO DEL TEMPO DI ASSESTAMENTO (Settling Time) ---
        soglia_errore = 0.05 # Tolleranza in radianti (circa 3 gradi)
        tempo_assestamento = "Non assestato"
        
        # Scorriamo la lista al contrario. 
        # Troviamo l'ultimo istante in cui il braccio era fuori dalla posizione finale.
        for i in range(len(data_log)-1, -1, -1):
            if abs(data_log[i]['position_rad'] - final_pos) > soglia_errore:
                if i + 1 < len(data_log):
                    tempo_assestamento = f"{data_log[i+1]['time_s']:.3f} s"
                break
        # ---------------------------------------------------------
        
        print("=====================================================")
        print(f"[QUICK SUMMARY] Target Desiderato : {target_absolute:.3f} rad")
        print(f"[QUICK SUMMARY] Posizione Finale  : {final_pos:.3f} rad")
        print(f"[QUICK SUMMARY] Errore a Regime   : {error:.3f} rad")
        print(f"[QUICK SUMMARY] Tempo Assestamento: {tempo_assestamento}")
        print("=====================================================")
    except KeyboardInterrupt:
        print("\nUser interrupted.")
    except Exception as e:
        print(f"\nUnexpected error: {e}")
    finally:
        print("\nClosing connections...")
        # You can add arm.stop() or disable torque here if your SDK supports it

if __name__ == "__main__":
    main()
