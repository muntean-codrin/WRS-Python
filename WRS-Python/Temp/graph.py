import numpy as np
import matplotlib.pyplot as plt

def f(x):
    return x**2*np.exp(-x**2)
x = np.linspace ( start = 0.    # lower limit
                , stop = 3      # upper limit
                , num = 51      # generate 51 points between 0 and 3
                )
y = f(x)    # This is already vectorized, that is, y will be a vector!
plt.plot(x, y)

def g(x):
    return x*np.exp(-x)
xx = np.arange  ( start = 0.
                , stop = 3.
                , step = 0.05
                ) # generate points between start and stop with distances of step apart from each other
yy = g(xx)
plt.plot( xx
        , yy
        , 'r-'  # plot with the color red, as line
        )

plt.show()