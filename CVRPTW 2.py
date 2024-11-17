from mip import Model, xsum, minimize, BINARY, INTEGER
import math
import numpy as np
import matplotlib.pyplot as plt

# Parámetros del problema (simplificados)
n_clients = 5  # número de clientes
n_vehicles = 3  # aumentado el número de vehículos
vehicle_capacity = 60  # aumentada la capacidad

# Coordenadas simplificadas
locations = [(0, 0), (10, 10), (15, 15), (5, 15), (20, 10), (15, 5)]

# Ventanas de tiempo más amplias y flexibles
time_windows = [(0, 50), (10, 20), (25, 40), (20, 40), (15, 30), (30, 40)]

# Demandas reducidas
demands = [0, 3, 4, 5, 6, 3]  # cliente 0 es el depósito

# Cálculo de distancias
distances = [[math.dist(locations[i], locations[j]) for j in range(n_clients + 1)] for i in range(n_clients + 1)]

# Visualización
plt.figure(figsize=(10, 10))
x = [li[0] for li in locations]
y = [li[1] for li in locations]
plt.scatter(x, y)
for i, (xi, yi) in enumerate(locations):
    plt.annotate(f'Node {i}', (xi, yi))
plt.title("Ubicación de clientes y depósito")
#plt.show()

# Modelo
model = Model()

# Variables
x = [[[model.add_var(var_type=BINARY) for j in range(n_clients + 1)] for i in range(n_clients + 1)] for k in range(n_vehicles)]
t = [model.add_var() for i in range(n_clients + 1)]

# Función objetivo
model.objective = minimize(
    xsum(distances[i][j] * x[k][i][j] 
         for k in range(n_vehicles) 
         for i in range(n_clients + 1) 
         for j in range(n_clients + 1))
)

# Restricciones básicas
# Cada cliente debe ser visitado una vez
for i in range(1, n_clients + 1):
    model += xsum(x[k][i][j] 
                 for k in range(n_vehicles) 
                 for j in range(n_clients + 1) 
                 if i != j) == 1

# Salida del depósito
for k in range(n_vehicles):
    model += xsum(x[k][0][j] for j in range(1, n_clients + 1)) <= 1
    model += xsum(x[k][i][0] for i in range(1, n_clients + 1)) <= 1

# Conservación de flujo
for k in range(n_vehicles):
    for h in range(n_clients + 1):
        model += xsum(x[k][i][h] for i in range(n_clients + 1) if i != h) == \
                xsum(x[k][h][j] for j in range(n_clients + 1) if j != h)

# Capacidad
for k in range(n_vehicles):
    model += xsum(demands[i] * xsum(x[k][i][j] for j in range(n_clients + 1) if i != j)
                 for i in range(1, n_clients + 1)) <= vehicle_capacity

# Tiempo simplificado
M = 100
service_time = 1

for i in range(n_clients + 1):
    model += time_windows[i][0] <= t[i]
    model += t[i] <= time_windows[i][1]

for k in range(n_vehicles):
    for i in range(n_clients + 1):
        for j in range(1, n_clients + 1):
            if i != j:
                model += t[j] >= t[i] + service_time + distances[i][j] - M * (1 - x[k][i][j])

# Optimización con tiempo límite
model.optimize(max_seconds=300)

# Resultados
if model.num_solutions:
    print(f"Valor objetivo: {model.objective_value:.2f}")
    
    for k in range(n_vehicles):
        route = []
        current = 0
        route_load = 0
        
        while True:
            route.append(current)
            next_node = -1
            
            for j in range(n_clients + 1):
                if current != j and x[k][current][j].x >= 0.99:
                    next_node = j
                    if j != 0:
                        route_load += demands[j]
                    break
            
            if next_node == -1 or next_node == 0:
                if next_node == 0:
                    route.append(0)
                break
            current = next_node
        
        if len(route) > 2:  # Solo muestra rutas no vacías
            print(f"\nVehículo {k+1}:")
            print(f"Ruta: {' -> '.join(map(str, route))}")
            print(f"Carga: {route_load}")
            print(f"Tiempos de llegada: {[round(t[i].x, 2) for i in route]}")
            route_length = sum(distances[route[i]][route[i+1]] for i in range(len(route)-1))
            print(f"Longitud de ruta: {route_length:.2f}")
else:
    print("No se encontró solución")