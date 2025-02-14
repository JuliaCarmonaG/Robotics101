import math

def calcular_perimetro_circulo(radio):
    return 2 * math.pi * radio

# Ejemplo de uso
radio = 5
perimetro = calcular_perimetro_circulo(radio)
print(f"El perímetro del círculo con radio {radio} es {perimetro}")

def calcular_area_circulo(radio):
    return math.pi * radio ** 2

def calcular_volumen_esfera(radio):
    return (4/3) * math.pi * radio ** 3

# Ejemplo de uso
area = calcular_area_circulo(radio)
volumen = calcular_volumen_esfera(radio)
print(f"El área del círculo con radio {radio} es {area}")
print(f"El volumen de la esfera con radio {radio} es {volumen}")