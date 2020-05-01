#!/usr/bin/env python3

import sys
import numpy as np
import matplotlib.pyplot as plt 
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import griddata
from matplotlib.ticker import LinearLocator, FormatStrFormatter
from matplotlib import cm

def plots_2D(x, y, x_name, y_name):

    plt.scatter(x, y, label= "stars", color= "green", marker= "*", s=30) 
    plt.xlabel(x_name) 
    plt.ylabel(y_name) 
    plt.title(x_name + " vs " + y_name)
    plt.show()

    plt.scatter(x, y)
    plt.xlabel(x_name)
    plt.ylabel(y_name) 
    plt.title(x_name + " vs " + y_name) 
    plt.plot(x, y, color='red')
    plt.show()

    x = np.array(x)
    y = np.array(y)
    plt.xlabel(x_name)
    plt.ylabel(y_name) 
    plt.title(x_name + " vs " + y_name) 
    plt.plot(x, y, 'o')
    m, b = np.polyfit(x, y, 1)
    plt.plot(x, m*x + b)
    plt.show()

def plot_3d(x, y, z, x_name, y_name, z_name):

    grid_x, grid_y = np.mgrid[min(x):max(x):100j, min(y):max(y):100j]
    grid_z = griddata((x, y), z, (grid_x, grid_y), method='cubic')

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    surf = ax.plot_surface(grid_x, grid_y, grid_z, cmap=cm.get_cmap("coolwarm"))
    ax.set_xlabel(x_name)
    ax.set_ylabel(y_name)
    ax.set_zlabel(z_name)
    plt.title("Robots vs time vs iterations")
    plt.show()

def initilize(entries):
    num_robots = []
    iterations = []
    elapsed_time = []

    for entry in entries:
        entry_values = entry.split(" ")
        num_robots.append(int(entry_values[0]))
        iterations.append(int(entry_values[1]))
        elapsed_time.append(float(entry_values[2]))
    
    return num_robots, iterations, elapsed_time

def main():
    
    if len(sys.argv) != 2:
        print("Usage: generate_plots.py <INPUT_FILE>")
        sys.exit(2)

    file_name = str(sys.argv[1])
    f = open(file_name, "rb")
    entries = f.read().decode("utf-8").split('\r\n')

    num_robots, iterations, elapsed_time = initilize(entries)
    inf_robots = num_robots[0]
    sup_robots = num_robots[len(num_robots) - 1]
    
    plots_2D(num_robots, elapsed_time, "robots", "time (s)")
    plots_2D(num_robots, iterations, "robots", "iter")
    plot_3d(tuple(num_robots), tuple(iterations), tuple(elapsed_time), "robots","iterations","time (s)")
    

if __name__ == "__main__":
    main()