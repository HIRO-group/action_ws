import matplotlib.pyplot as plt
import numpy.typing as npt
from numpy import linalg
import numpy as np
import math
import random

lamba_arr = [0.5, 1.0, 2.5]


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


def get_z(sigma: float, lambda_arg: float) -> float:
    phi = get_phi(lambda_arg)
    phi_inv = 1.0/phi
    num = math.log(1.0-sigma*lambda_arg*phi_inv)
    denom = lambda_arg
    return -1.0*num/denom


def get_z_arr(sigma_arr: list, lambda_arg: float) -> list:
    z_arr = []
    for sigma in sigma_arr:
        dist = get_z(sigma, lambda_arg)
        z_arr.append(dist)
    return z_arr


def get_omega(z: float) -> float:
    return math.sqrt(2.0*z)


def get_sigma(vrand: npt.ArrayLike, vfield: npt.ArrayLike) -> npt.ArrayLike:
    return 0.25*(linalg.norm(np.subtract(vrand, vfield)**2))


def get_alpha_beta(omega: float, vrand: npt.ArrayLike, vfield: npt.ArrayLike):
    w2 = omega*omega
    c = np.dot(vrand, vfield)
    cc_1 = c*c-1.0
    root = math.sqrt(cc_1*w2*(w2-4.0))
    beta = -1.0*root/(2.0*cc_1)
    sign = -1.0 if beta < 0.0 else 1.0
    beta *= sign
    alpha = (sign*c*root+cc_1*(2.0-w2))/(2.0*cc_1)
    # print("omega: ", omega)
    # print("alpha: ", alpha)
    # print("beta: ", beta)
    return [alpha, beta]


def get_sample_origin() -> npt.ArrayLike:
    origin = np.array([random.uniform(0, 11), random.uniform(0, 11)])
    return origin


def get_vfield(goal: npt.ArrayLike, start: npt.ArrayLike) -> npt.ArrayLike:
    vec = np.subtract(goal, start)
    vec = vec/np.linalg.norm(vec)
    return vec


def get_vrand() -> npt.ArrayLike:
    vrand = np.array([random.random(), random.random()])
    vrand = vrand/np.linalg.norm(vrand)
    return vrand


def get_vnew(vrand: npt.ArrayLike, vfield: npt.ArrayLike):
    sigma = get_sigma(vrand, vfield)
    z = get_z(sigma, lamba_arr[0])
    # print("z: ", z)
    omega = get_omega(z)
    [alpha, beta] = get_alpha_beta(omega, vrand, vfield)
    vnew = alpha*vfield + beta*vrand
    vnew = vnew/np.linalg.norm(vnew)
    return vnew


def get_exponential_alpha(step: int) -> float:
    # c = 100
    # alpha = step/(step+c)

    c = -0.005
    alpha = 1.0-math.exp(c*step)

    return alpha


def get_exp_alpha_arr(step_arr: npt.ArrayLike) -> npt.ArrayLike:
    alpha_arr = []
    for step in step_arr:
        alpha = get_exponential_alpha(step)
        alpha_arr.append(alpha)
    return alpha_arr


def plot_exp_alpha():
    step_arr = np.arange(1, 1000, 1, dtype=float)
    sigma_arr = get_exp_alpha_arr(step_arr)
    plt.plot(step_arr, sigma_arr, 'b-')
    plt.show()


def plot_cumulative_dist_func():
    z_arr = np.arange(0, 10, 0.1, dtype=float)
    sigma_arr = get_sigma_arr(z_arr, lamba_arr[0])
    plt.plot(z_arr, sigma_arr, 'ro')
    sigma_arr = get_sigma_arr(z_arr, lamba_arr[1])
    plt.plot(z_arr, sigma_arr, 'bo')
    sigma_arr = get_sigma_arr(z_arr, lamba_arr[2])
    plt.plot(z_arr, sigma_arr, 'go')
    plt.legend(["lambda "+str(lamba_arr[0]), "lambda " +
                str(lamba_arr[1]), "lambda "+str(lamba_arr[2])])
    plt.show()


def plot_z_func():
    z_arr = np.arange(0, 1, 0.1, dtype=float)
    sigma_arr = get_z_arr(z_arr, lamba_arr[0])
    plt.plot(z_arr, sigma_arr, 'ro')
    sigma_arr = get_z_arr(z_arr, lamba_arr[1])
    plt.plot(z_arr, sigma_arr, 'bo')
    sigma_arr = get_z_arr(z_arr, lamba_arr[2])
    plt.plot(z_arr, sigma_arr, 'go')
    plt.legend(["lambda "+str(lamba_arr[0]), "lambda " +
                str(lamba_arr[1]), "lambda "+str(lamba_arr[2])])
    plt.show()


def plot_direction_vectors():
    goal = np.array([5, 5])
    num_samples = 60

    vrand_arr = np.zeros((num_samples, 2))
    vfield_arr = np.zeros((num_samples, 2))
    vnew_arr = np.zeros((num_samples, 2))
    origin_arr = np.zeros((num_samples, 2))

    for i in range(num_samples):
        origin = get_sample_origin()
        vrand = get_vrand()
        vfield = get_vfield(goal, origin)
        vnew = get_vnew(vrand, vfield)
        vrand_arr[i] = vrand
        vfield_arr[i] = vfield
        vnew_arr[i] = vnew
        origin_arr[i] = origin

    plt.quiver(origin_arr[:, 0], origin_arr[:, 1],
               vfield_arr[:, 0], vfield_arr[:, 1], color='b', width=0.005, headwidth=2)

    plt.quiver(origin_arr[:, 0], origin_arr[:, 1],
               vrand_arr[:, 0], vrand_arr[:, 1], color='k', width=0.005, headwidth=2)

    plt.quiver(origin_arr[:, 0], origin_arr[:, 1],
               vnew_arr[:, 0], vnew_arr[:, 1], color='r', width=0.005, headwidth=2)

    plt.show()


# plot_cumulative_dist_func()
# plot_z_func()
# plot_direction_vectors()
plot_exp_alpha()
