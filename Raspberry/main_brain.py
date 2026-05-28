import time
import cv2

from control_brain import ControlBrain
from vision_brain import VisionBrain
from sensors import parse_sensor_data
from config import (
    CAPTURE_DELAY, DEFAULT_SPEED, STAIRS_SPEED, CLIMB_SPEED,
    MODE_IDLE, MODE_MANUAL, MODE_OBJECT, MODE_STAIRS, MODE_CLIMB_ASSIST
)


class ApexMainBrain:
    """
    Functional integration between ControlBrain and VisionBrain.

    V2 fixes:
    - Manual commands are direct and repeatable.
    - Camera commands are direct and repeatable.
    - Jack test keys are included.
    - Serial reading drains several waiting lines to reduce lag.
    """

    def __init__(self):
        self.control = ControlBrain()
        self.vision = VisionBrain()
        self.mode = MODE_MANUAL
        self.running = True

    def start(self):
        self.control.connect_all()
        if not self.vision.open_camera():
            self.shutdown()
            return
        self.control.set_speed(DEFAULT_SPEED)
        self.control.camera_center()
        self.print_start_info()
        self.run_loop()

    def print_start_info(self):
        print("[SYSTEM] Apex Modular Integrated Brain V2 Started")
        print()
        print("Modes: 0=IDLE, m=MANUAL, 1=OBJECT, 2=STAIRS, 3=CLIMB_ASSIST")
        print("Movement: f=forward, b=backward, l=left, r=right, x=stop")
        print("Camera: w=up, s=down, a=left, d=right, c=center")
        print("Jacks: g=front extend, h=front retract, n=front stop")
        print("       u=rear extend,  j=rear retract,  k=rear stop, z=stop all")
        print("Other: p=print sensors, q=quit")
        print()

    # ========================================================
    # Incoming Mega / ESP32
    # ========================================================

    def handle_incoming_mega_lines(self):
        # Drain a few available lines each loop so serial buffer does not lag.
        for _ in range(5):
            line = self.control.mega.read_line()
            if not line:
                break

            print(f"[FROM MEGA/ESP32] {line}")

            if line.startswith("DATA:"):
                data = parse_sensor_data(line)
                if data:
                    self.control.update_sensor_data(data)
                continue

            if line.startswith("MODE:"):
                new_mode = line.replace("MODE:", "").strip().upper()
                if new_mode in [MODE_IDLE, MODE_MANUAL, MODE_OBJECT, MODE_STAIRS, MODE_CLIMB_ASSIST]:
                    self.change_mode(new_mode)
                else:
                    print(f"[WARNING] Unknown mode: {new_mode}")
                continue

            if line.startswith("TARGET:"):
                new_color = line.replace("TARGET:", "").strip().lower()
                self.vision.set_target_color(new_color)
                continue

            if line.startswith("CMD:"):
                cmd = line.replace("CMD:", "").strip().upper()
                if cmd == "STOP":
                    self.control.emergency_stop()
                    self.change_mode(MODE_IDLE)
                continue

    # ========================================================
    # Mode Control
    # ========================================================

    def change_mode(self, new_mode: str):
        self.mode = new_mode
        print(f"[BRAIN] Mode changed to: {self.mode}")

        if self.mode == MODE_IDLE:
            self.control.stop()
            self.control.camera_stop()

        elif self.mode == MODE_MANUAL:
            self.control.camera_center()

        elif self.mode == MODE_OBJECT:
            self.control.set_speed(DEFAULT_SPEED)
            self.control.camera_center()

        elif self.mode == MODE_STAIRS:
            self.control.set_speed(STAIRS_SPEED)
            # Do not keep forcing camera down every frame. One time is enough when entering mode.
            self.control.camera_down()

        elif self.mode == MODE_CLIMB_ASSIST:
            self.control.set_speed(CLIMB_SPEED)
            self.control.mode_climb()
            self.control.camera_down()

    # ========================================================
    # Climb Assist
    # ========================================================

    def climb_assist_step(self, stairs):
        data = self.control.latest_sensor_data

        if not data.is_safe_angle():
            self.control.stop()
            self.control.stop_all_jacks()
            return "UNSAFE_ANGLE_STOP"

        if not stairs["stairs_found"]:
            self.control.stop()
            return "NO_STAIRS_WAITING"

        move_cmd, state = self.vision.decide_stairs_action(stairs, data)

        if state == "STAIRS_TOO_CLOSE_GO_SLOW":
            self.control.set_speed(CLIMB_SPEED)
            self.control.apply_auto_move_command("SLOW")

        elif data.pitch > 12.0:
            self.control.stop()
            self.control.front_jack_extend()
            time.sleep(0.15)
            self.control.front_jack_stop()
            state = "FRONT_JACK_ASSIST"

        elif data.pitch < -8.0:
            self.control.stop()
            self.control.rear_jack_extend()
            time.sleep(0.15)
            self.control.rear_jack_stop()
            state = "REAR_JACK_ASSIST"

        else:
            self.control.apply_auto_move_command(move_cmd)

        return state

    # ========================================================
    # Main Loop
    # ========================================================

    def run_loop(self):
        while self.running:
            self.handle_incoming_mega_lines()
            self.control.request_sensors_every_second()

            ret, frame = self.vision.read_frame()
            if not ret:
                print("[WARNING] Failed to capture frame")
                self.handle_keyboard()
                time.sleep(0.05)
                continue

            mask = None
            data = self.control.latest_sensor_data

            if self.mode == MODE_OBJECT:
                detection, mask = self.vision.detect_color_object(frame)
                move_cmd, cam_cmd, state = self.vision.decide_object_action(detection)

                self.control.apply_auto_move_command(move_cmd)
                self.control.send_camera_command(cam_cmd)

                debug_frame = self.vision.draw_object_debug(
                    frame, detection, move_cmd, cam_cmd, state,
                    self.mode, self.control.speed, data
                )

            elif self.mode == MODE_STAIRS:
                stairs, stairs_debug, edges = self.vision.detect_stairs(frame)
                move_cmd, state = self.vision.decide_stairs_action(stairs, data)

                self.control.apply_auto_move_command(move_cmd)

                debug_frame = self.vision.draw_stairs_debug(
                    stairs_debug, stairs, move_cmd, state,
                    self.mode, self.control.speed, data
                )
                mask = edges

            elif self.mode == MODE_CLIMB_ASSIST:
                stairs, stairs_debug, edges = self.vision.detect_stairs(frame)
                state = self.climb_assist_step(stairs)

                debug_frame = self.vision.draw_stairs_debug(
                    stairs_debug, stairs, "AUTO", state,
                    self.mode, self.control.speed, data
                )
                mask = edges

            else:
                debug_frame = self.vision.draw_idle_debug(
                    frame, self.mode, self.control.speed, data
                )

            cv2.imshow("Apex Rover Modular Brain V2", debug_frame)
            if mask is not None:
                cv2.imshow("Vision Mask / Edges", mask)

            self.handle_keyboard()
            time.sleep(CAPTURE_DELAY)

        self.shutdown()

    # ========================================================
    # Keyboard
    # ========================================================

    def handle_keyboard(self):
        key = cv2.waitKey(1) & 0xFF
        if key == 255:
            return

        # Modes
        if key == ord("q"):
            self.running = False
        elif key == ord("0"):
            self.change_mode(MODE_IDLE)
        elif key == ord("m"):
            self.change_mode(MODE_MANUAL)
        elif key == ord("1"):
            self.change_mode(MODE_OBJECT)
        elif key == ord("2"):
            self.change_mode(MODE_STAIRS)
        elif key == ord("3"):
            self.change_mode(MODE_CLIMB_ASSIST)

        # Manual movement - direct and repeatable
        elif key == ord("f"):
            self.control.forward()
        elif key == ord("b"):
            self.control.backward()
        elif key == ord("l"):
            self.control.left()
        elif key == ord("r"):
            self.control.right()
        elif key == ord("x"):
            self.control.stop()

        # Camera - direct and repeatable. UP/DOWN inversion fixed in ControlBrain.
        elif key == ord("w"):
            self.control.camera_up()
        elif key == ord("s"):
            self.control.camera_down()
        elif key == ord("a"):
            self.control.camera_left()
        elif key == ord("d"):
            self.control.camera_right()
        elif key == ord("c"):
            self.control.camera_center()

        # Jacks - direct and repeatable
        elif key == ord("g"):
            self.control.front_jack_extend()
        elif key == ord("h"):
            self.control.front_jack_retract()
        elif key == ord("n"):
            self.control.front_jack_stop()
        elif key == ord("u"):
            self.control.rear_jack_extend()
        elif key == ord("j"):
            self.control.rear_jack_retract()
        elif key == ord("k"):
            self.control.rear_jack_stop()
        elif key == ord("z"):
            self.control.stop_all_jacks()

        elif key == ord("p"):
            self.control.print_sensor_data()

    def shutdown(self):
        print("[SYSTEM] Shutting down modular brain V2")
        try:
            self.control.shutdown()
        except Exception:
            pass
        try:
            self.vision.close()
        except Exception:
            pass


def main():
    brain = ApexMainBrain()
    try:
        brain.start()
    except KeyboardInterrupt:
        print()
        brain.shutdown()
    except Exception as e:
        print("[ERROR] Fatal main brain error")
        print(e)
        brain.shutdown()


if __name__ == "__main__":
    main()
