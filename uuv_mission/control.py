# uuv_mission/control.py
class PDController:
    """
    Simple proportional–derivative (PD) feedback controller.
    Computes control action u = KP * error + KD * (error - previous_error)
    """

    def __init__(self, KP: float = 0.15, KD: float = 0.6):
        self.KP = KP
        self.KD = KD
        self.previous_error = 0.0

    def __call__(self, reference: float, measurement: float) -> float:
        """Return control action for the current timestep."""
        error = reference - measurement
        derivative = error - self.previous_error
        self.previous_error = error
        u = self.KP * error + self.KD * derivative
        return u
