#!/usr/bin/env python3

import robots
from os import system
import sys

def main():
    
    if len(sys.argv) != 4:
        print("Usage: generate_data.py <INPUT_FILE> <lim_inf_robots> <lim_sup_robots")
        sys.exit(2)

    file_name = str(sys.argv[1])
    inf_robots = int(sys.argv[2])
    sup_robots = int(sys.argv[3])
    
    for i in range(inf_robots, sup_robots):
        system("python robots.py " + file_name + " 1 " + str(i))

if __name__ == "__main__":
    main()