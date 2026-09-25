class Avionics:
    def __init__(self, interval:float, libavionics=None):
        self.core = libavionics.Avionics()

        # time
        self.delay = interval # sec
        self._last_t = 0.0

    def update(self, t:float) -> None:
        if t - self._last_t >= self.delay:
            self.core.update()
            self._last_t = t