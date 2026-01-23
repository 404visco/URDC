import random
import numpy as np
import matplotlib.pyplot as plt
random.seed(1)
x= np.linspace(0,100,200)
noise = np.random.randn(len(x))
# print(x)
# print(noise)
# print(x)
# print(noise)
# def f(x):
#     return x**2
# def f_noise(x):
#     return [f(i)+noise[i]for i in range(x)]
# # print(f_noise(2))
# print(f(np.arange(0,3)))
start= 0
end=3
def f(x):
    return x**2
xi = np.arange(start, end)
plt.scatter([0,1,2],xi)