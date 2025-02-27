import numpy as np

def calcular_probabilidad_goles(promedio_goles, partidos=10):
    probabilidades = []
    for partido in range(partidos):
        goles = np.random.poisson(promedio_goles)
        probabilidades.append(goles)
    return probabilidades

# Ejemplo de uso
promedio_goles = 2.5  # Promedio de goles por partido
probabilidades = calcular_probabilidad_goles(promedio_goles)

for i, goles in enumerate(probabilidades, 1):
    print(f"Partido {i}: {goles} goles")