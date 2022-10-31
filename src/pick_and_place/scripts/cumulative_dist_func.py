import matplotlib.pyplot as plt
import numpy.typing as npt
from numpy import linalg
import numpy as np
import math

lamba_arr = [1.0, 2.5, 5.0]


def get_phi(lambda_arg: float) -> float:
    num = lambda_arg
    denom = 1.0-math.exp(-2.0*lambda_arg)
    return num/denom


def cumulative_dist_func(lambda_arg: float, z: float) -> float:
    phi = get_phi(lambda_arg)
    num = phi*(1.0-math.exp(-lambda_arg*z))
    denom = lambda_arg
    return num/denom


def get_sigma_arr(z_arr: list, lambda_arg: float) -> list:
    sigma_arr = []
    for z in z_arr:
        dist = cumulative_dist_func(lambda_arg, z)
        sigma_arr.append(dist)
    return sigma_arr


def cumulative_inverse(sigma: float, lambda_arg: float) -> float:
    phi = get_phi(lambda_arg)
    phi_inv = 1.0/phi
    num = math.log(1.0-sigma*lambda_arg*phi_inv)
    denom = lambda_arg
    return -1.0*num/denom


def get_omega(z: float) -> float:
    return math.sqrt(2.0*z)


def get_sigma(vrand: npt.ArrayLike, vfield: npt.ArrayLike) -> npt.ArrayLike:
    return 0.25*(linalg(np.subtract(vrand-vfield))**2)


def get_alpha_beta(omega: float, vrand: npt.ArrayLike, vfield: npt.ArrayLike):


def plot_cumulative_dist_func():
    z_arr = np.arange(0, 5, 0.1, dtype=float)
    sigma_arr = get_sigma_arr(z_arr, lamba_arr[0])
    plt.plot(z_arr, sigma_arr, 'ro')
    sigma_arr = get_sigma_arr(z_arr, lamba_arr[1])
    plt.plot(z_arr, sigma_arr, 'bo')
    sigma_arr = get_sigma_arr(z_arr, lamba_arr[2])
    plt.plot(z_arr, sigma_arr, 'go')
    plt.legend(["lambda "+str(lamba_arr[0]), "lambda " +
                str(lamba_arr[1]), "lambda "+str(lamba_arr[2])])

    plt.show()
