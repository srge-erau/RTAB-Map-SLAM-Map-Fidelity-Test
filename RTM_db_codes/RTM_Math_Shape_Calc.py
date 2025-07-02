import math

def find_possible_shapes(size):
    for i in range(200, 2000):
        if size % i == 0:
            h, w = i, size // i
            print(f"Possible shape: {h}x{w}")
            
find_possible_shapes(234248)