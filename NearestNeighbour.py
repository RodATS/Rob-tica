import math

# Coordenadas de los puntos
puntos = {
    'A': (0, 0),
    'B': (3, 4),
    'C': (6, 8),
    'D': (9, 12),
    'E': (12, 16)
}

# Conexiones o aristas entre puntos
conexiones = {
    'A': ['B', 'C'],
    'B': ['A', 'D'],
    'C': ['A', 'D', 'E'],
    'D': ['B', 'C', 'E'],
    'E': ['C', 'D']
}

# Función para calcular la distancia entre dos puntos
def distancia(punto1, punto2):
    x1, y1 = punto1
    x2, y2 = punto2
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

# Algoritmo Nearest Neighbor
def nearest_neighbor(punto_inicio):
    camino = [punto_inicio]
    puntos_visitados = set([punto_inicio])

    while len(puntos_visitados) < len(puntos):
        punto_actual = camino[-1]
        punto_mas_cercano = None
        distancia_minima = float('inf')

        for punto in conexiones[punto_actual]:
            if punto not in puntos_visitados:
                distancia_actual = distancia(puntos[punto_actual], puntos[punto])
                if distancia_actual < distancia_minima:
                    distancia_minima = distancia_actual
                    punto_mas_cercano = punto

        camino.append(punto_mas_cercano)
        puntos_visitados.add(punto_mas_cercano)

    return camino

# Imprimir el camino óptimo
punto_inicio = 'A'
camino_optimo = nearest_neighbor(punto_inicio)
print("Camino óptimo:", camino_optimo)

# Imprimir las distancias entre cada par de puntos en el camino óptimo
distancias = []
for i in range(len(camino_optimo) - 1):
    punto1 = puntos[camino_optimo[i]]
    punto2 = puntos[camino_optimo[i + 1]]
    distancias.append(distancia(punto1, punto2))

print("Distancias:", distancias)

# Imprimir la distancia total del camino óptimo
distancia_total = sum(distancias)
print("Distancia total:", distancia_total)
