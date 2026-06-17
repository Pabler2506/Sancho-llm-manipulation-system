import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import os

# Cargar datos mapa
ruta_mapa_png = os.path.expanduser('~/tfg_ws/src/contexto/map.png')
resolucion = 0.01    
origen_x = -5.525           
origen_y = -5.425            

ref_cocina = (-4.53, -4.06)
ref_dormitorio = (-1.65, 2.55)
ref_bano = (-1.66, -0.61)

test_cocina = ([-4.35, -4.39, -4.42, -4.74, -4.74], 
               [-4.16, -3.87, -4.38, -4.33, -4.17])

test_dormitorio = ([-1.69, -1.60, -1.48, -1.80, -1.51, -1.80], 
                   [2.55, 2.50, 2.42, 2.53, 2.67, 2.36])

test_bano = ([-1.53, -1.49, -1.48, -1.45, -1.49, -1.48], 
             [-0.44, -0.49, -0.49, -0.51, -0.48, -0.43])

plt.figure(figsize=(12, 8))
plt.style.use('seaborn-v0_8-whitegrid')

# Cargar la imagen del mapa
img = mpimg.imread(ruta_mapa_png)

# Calcular las dimensiones reales del mapa en metros
alto_pixeles, ancho_pixeles = img.shape[:2]
max_x = origen_x + (ancho_pixeles * resolucion)
max_y = origen_y + (alto_pixeles * resolucion)

# Dibujar el mapa como imagen de fondo
plt.imshow(img, extent=[origen_x, max_x, origen_y, max_y], origin='upper', cmap='gray', alpha=0.6)

# Graficar Referencias )
plt.scatter(*ref_cocina, color='red', marker='*', s=300, edgecolors='black', label='Ref. Cocina')
plt.scatter(*ref_dormitorio, color='blue', marker='*', s=300, edgecolors='black', label='Ref. Dormitorio')
plt.scatter(*ref_bano, color='green', marker='*', s=300, edgecolors='black', label='Ref. Cuarto de Baño')

# Graficar Llegadas 
plt.scatter(test_cocina[0], test_cocina[1], color='red', marker='o', s=70, alpha=0.8, edgecolors='black', label='Llegadas Cocina')
plt.scatter(test_dormitorio[0], test_dormitorio[1], color='blue', marker='o', s=70, alpha=0.8, edgecolors='black', label='Llegadas Dormitorio')
plt.scatter(test_bano[0], test_bano[1], color='green', marker='o', s=70, alpha=0.8, edgecolors='black', label='Llegadas Cuarto de Baño')

plt.title('Dispersión Posicional del Sistema de Navegación Nav2', fontsize=22, pad=15)
plt.xlabel('Coordenada X (metros)', fontsize=20)
plt.ylabel('Coordenada Y (metros)', fontsize=20)

# Limitar los ejes visualmente
plt.xlim(-6, 0)
plt.ylim(-5, 4)

plt.axis('equal')
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
plt.grid(True, linestyle='--', alpha=0.5)

plt.tight_layout()
plt.show()