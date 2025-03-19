import numpy as np

def generate_distance_matrix(n, minDis = 10, maxDis = 100, filename="data/distance_matrix.txt"):
    distance_matrix = np.random.randint(minDis, maxDis, size=(n, n))
    np.fill_diagonal(distance_matrix, 0) 
    
    with open(filename, "w") as f:
        f.write(f"{n}\n")  
        for row in distance_matrix:
            f.write(" ".join(map(str, row)) + "\n")

    print(f"Distance matrix saved to {filename}")

generate_distance_matrix(10)
