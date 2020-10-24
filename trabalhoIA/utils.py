import random
import numpy as np

def map_Generator(available_nodes, density=0.5, weights_range=(0, 100), cartesian=True):
    """
    Gera uma lista de conexoes com pesos no intervalo 'weights_range'
    baseado numa lista de ids em 'available_nodes', a variavel 'density'
    especifica o quao conectados os nos vao estar.
    """
    map_data = set({})

    for _ in range(int(len(available_nodes) * density)):
        nodes = random.sample(available_nodes, 2)

        if not cartesian:
            rand_weight = random.randint(weights_range[0], weights_range[1])
        else:
            cords1 = tuple(np.random.randint(weights_range[0], weights_range[1]+1, 2))
            cords2 = tuple(np.random.randint(weights_range[0], weights_range[1]+1, 2))
            rand_weight = (cords1, cords2)
        
        print(rand_weight)
        map_data.add(
            (
                nodes[0],
                nodes[1],
                rand_weight
            )
        )

    print(f"Mapa gerado com {int(len(available_nodes) * density)} conexoes")
    return map_data
