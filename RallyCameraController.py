import subprocess

class RallyCameraController:
    MAX_PAN_STEPS = 4480    # 90 deg pan
    MAX_TILT_STEPS = 1920   # full tilt travel

    def __init__(
        self, 
        camera_heading: float = 0.0, 
        is_angled_back_45: bool = True, 
        camera_node: str = "/dev/video2"
    ):
        self.camera_heading = camera_heading
        self.is_angled_back_45 = is_angled_back_45
        self.camera_node = camera_node
        
        # assuming centre is 0,0
        self.current_pan_deg = 0.0
        self.current_tilt_deg = 0.0
        
        self.reset_home()
    
    def reset_home(self) -> None:
        """Homes camera to 0, 0."""
        cmd = [
            "v4l2-ctl", "-d", self.camera_node,
            "--set-ctrl=pan_reset=1,tilt_reset=1"
        ]
        try:
            subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.current_pan_deg = 0.0
            self.current_tilt_deg = 0.0
            print("[INFO] Camera successfully homed to (0°, 0°).")
        except subprocess.CalledProcessError as e:
            print(f"[WARN] Failed to trigger home reset: {e}")

    def zero_software_state(self) -> None:
        """Resets internal position tracking variables without triggering physical homing."""
        self.current_pan_deg = 0.0
        self.current_tilt_deg = 0.0

    def point_at_target(self, target_pan: float, target_tilt: float) -> None:
        """Calculates relative delta and executes combined axis movement."""
        # calc degrees needed to face aircraft
        delta_pan = target_pan - self.current_pan_deg
        delta_tilt = target_tilt - self.current_tilt_deg

        # convert to steps
        rel_pan_steps = int((delta_pan / 90.0) * self.MAX_PAN_STEPS)
        # tilt is inverted
        rel_tilt_steps = -int((delta_tilt / 90.0) * self.MAX_TILT_STEPS)

        # prevent hard hitting end stops
        rel_pan_steps = max(-4480, min(4480, rel_pan_steps))
        rel_tilt_steps = max(-1920, min(1920, rel_tilt_steps))

        # ignore tiny jitter
        if abs(rel_pan_steps) > 30 or abs(rel_tilt_steps) > 30:
            cmd = [
                "v4l2-ctl", "-d", self.camera_node,
                f"--set-ctrl=pan_relative={rel_pan_steps},tilt_relative={rel_tilt_steps}"
            ]
            try:
                subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                
                # set state only after successful command
                self.current_pan_deg += (rel_pan_steps / self.MAX_PAN_STEPS) * 90.0
                self.current_tilt_deg -= (rel_tilt_steps / self.MAX_TILT_STEPS) * 90.0
                print(f"[INFO] Camera pointing to az: {target_pan}, el: {target_pan}")
            except subprocess.CalledProcessError:
                print("[WARN] Could not angle camera")
                pass
