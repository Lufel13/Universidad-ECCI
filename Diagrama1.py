import matplotlib.pyplot as plt
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def create_data_model():
    data = {}
    data['matriz_distancias'] = [
        [0, 548, 776, 696, 582, 274, 502, 194, 308, 194, 536, 502, 388, 354, 468, 776, 662],
        [548, 0, 684, 308, 194, 502, 730, 354, 696, 742, 1084, 594, 480, 674, 1016, 868, 1210],
        [776, 684, 0, 992, 878, 502, 274, 810, 468, 742, 400, 1278, 1164, 1130, 788, 1552, 754],
        [696, 308, 992, 0, 114, 650, 878, 502, 844, 890, 1232, 514, 628, 822, 1164, 560, 1358],
        [582, 194, 878, 114, 0, 536, 764, 388, 730, 776, 1118, 400, 514, 708, 1050, 674, 1244],
        [274, 502, 502, 650, 536, 0, 228, 308, 194, 240, 582, 776, 662, 628, 514, 1050, 708],
        [502, 730, 274, 878, 764, 228, 0, 536, 194, 468, 354, 1004, 890, 856, 514, 1278, 480],
        [194, 354, 810, 502, 388, 308, 536, 0, 342, 388, 730, 468, 354, 320, 662, 742, 856],
        [308, 696, 468, 844, 730, 194, 194, 342, 0, 274, 388, 810, 696, 662, 320, 1084, 514],
        [194, 742, 742, 890, 776, 240, 468, 388, 274, 0, 342, 536, 422, 388, 274, 810, 468],
        [536, 1084, 400, 1232, 1118, 582, 354, 730, 388, 342, 0, 878, 764, 730, 388, 1152, 354],
        [502, 594, 1278, 514, 400, 776, 1004, 468, 810, 536, 878, 0, 114, 308, 650, 274, 844],
        [388, 480, 1164, 628, 514, 662, 890, 354, 696, 422, 764, 114, 0, 194, 536, 388, 730],
        [354, 674, 1130, 822, 708, 628, 856, 320, 662, 388, 730, 308, 194, 0, 342, 422, 536],
        [468, 1016, 788, 1164, 1050, 514, 514, 662, 320, 274, 388, 650, 536, 342, 0, 764, 194],
        [776, 868, 1552, 560, 674, 1050, 1278, 742, 1084, 810, 1152, 274, 388, 422, 764, 0, 798],
        [662, 1210, 754, 1358, 1244, 708, 480, 856, 514, 468, 354, 844, 730, 536, 194, 798, 0],
    ]
    data['num_vehiculos'] = 4
    data['deposito'] = 0
    return data

def print_solution(data, manager, routing, solution):
    total_distance = 0
    for vehicle_id in range(data['num_vehiculos']):
        index = routing.Start(vehicle_id)
        plan_output = 'Ruta para el vehiculo {}:\n'.format(vehicle_id)
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += '{}->'.format(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
        plan_output += '{}\n'.format(manager.IndexToNode(index))
        plan_output += 'Distancia de la ruta: {}m\n'.format(route_distance)
        print(plan_output)
        total_distance += route_distance
    print('Distancia total de todas las rutas: {}m'.format(total_distance))

import networkx as nx
import matplotlib.pyplot as plt

def plot_routes(data, manager, routing, solution):
    G = nx.DiGraph()  # Crear un grafo dirigido
    num_nodes = len(data['matriz_distancias'])

    # Añadir nodos al grafo
    for i in range(num_nodes):
        G.add_node(i)

    # Añadir arcos solo para las rutas de los vehículos
    for vehicle_id in range(data['num_vehiculos']):
        index = routing.Start(vehicle_id)
        route_nodes = [manager.IndexToNode(index)]
        while not routing.IsEnd(index):
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_nodes.append(manager.IndexToNode(index))
        
        # Añadir arcos de la ruta del vehículo al grafo
        for i in range(len(route_nodes) - 1):
            from_node = route_nodes[i]
            to_node = route_nodes[i + 1]
            G.add_edge(from_node, to_node, weight=data['matriz_distancias'][from_node][to_node])

    pos = nx.spring_layout(G)  # Posición de los nodos en la gráfica
    plt.figure(figsize=(12, 8))

    # Dibujar el grafo sin las etiquetas de los bordes
    nx.draw(G, pos, with_labels=True, node_size=500, node_color='lightblue', font_size=10, font_weight='bold', arrows=True, edge_color='gray')
    
    # Traza las rutas de cada vehículo
    for vehicle_id in range(data['num_vehiculos']):
        index = routing.Start(vehicle_id)
        route_nodes = [manager.IndexToNode(index)]
        while not routing.IsEnd(index):
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_nodes.append(manager.IndexToNode(index))
        route_positions = [pos[node] for node in route_nodes]  # Obtener posiciones de los nodos en la ruta
        route_x, route_y = zip(*route_positions)  # Separar las coordenadas x e y
        plt.plot(route_x, route_y, marker='o', linestyle='-', label=f'Vehículo {vehicle_id}')

    plt.legend()
    plt.title('Rutas de los Vehículos')
    plt.show()


def main():
    # Invocar la data de entrada
    data = create_data_model()

    # Crea el administrador del índice de rutas
    manager = pywrapcp.RoutingIndexManager(len(data['matriz_distancias']), data['num_vehiculos'], data['deposito'])

    # Crea el modelo de enrutamiento
    routing = pywrapcp.RoutingModel(manager)

    # Crea y registra una devolución de llamada de distancia
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['matriz_distancias'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define el costo de cada arco
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Añade la dimensión de distancia
    dimension_name = 'Distancia'
    routing.AddDimension(
        transit_callback_index,
        0,  # Sin holgura
        3000,  # Distancia máxima de viaje del vehículo
        True,  # Iniciar el acumulador en cero
        dimension_name)
    distance_dimension = routing.GetDimensionOrDie(dimension_name)
    distance_dimension.SetGlobalSpanCostCoefficient(100)

    # Configurar los parámetros de búsqueda.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solucionador del problema
    solution = routing.SolveWithParameters(search_parameters)

    # Imprimir la solución en la consola
    if solution:
        print_solution(data, manager, routing, solution)
        plot_routes(data, manager, routing, solution)  # Llamar a la función para graficar las rutas
    else:
        print('No se encuentra solución !')

if __name__ == '__main__':
    main()
