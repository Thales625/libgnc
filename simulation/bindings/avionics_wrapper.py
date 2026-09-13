import numpy as np
from . import libavionics

class AvionicsSim:
    def __init__(self, interval:float):
        self.core = libavionics.Avionics()

        # time
        self.delay = interval # sec
        self._last_t = 0.0

    def update(self, t:float) -> None:
        if t - self._last_t >= self.delay:
            self.core.update()
            self._last_t = t

if __name__ == "__main__":
    avionics = AvionicsSim(0.01)

    print("--- FlightState Attributes ---")

    for name in sorted(dir(avionics.core)):
        if not name.startswith("_"):
            try:
                value = getattr(avionics.core, name)
                print(f"\t{name}: {type(value).__name__}")
            except Exception:
                print(f"\t{name}")

    print("------------------------------")