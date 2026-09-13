import numpy as np

class Earth:
    gravity = np.array([0., 0., -9.80665])
    g = float(np.linalg.norm(gravity))

    @staticmethod
    def density(position:np.ndarray) -> float:
        altitude = position[2]
        return 1.225 * ((2.71828 ** (-altitude / 8500.)) if altitude > 0. else 1.)

    @staticmethod
    def pressure(position:np.ndarray) -> float:
        altitude = position[2]
        return 101325. * (1. - altitude/44330.)**5.255

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    import numpy as np

    print(Earth.gravity)

    h_arr = np.linspace(0, 100000, 1000)

    plt.plot(h_arr, [Earth.density(np.array([0., 0., h])) for h in h_arr])
    plt.show()