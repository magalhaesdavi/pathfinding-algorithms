import random


def map_Generator(available_nodes, density=0.5, weights_range=(0, 100)):
    """
    Gera uma lista de conexoes com pesos no intervalo 'weights_range'
    baseado numa lista de ids em 'available_nodes', a variavel 'density'
    especifica o quao conectados os nos vao estar.
    """
    map = set({})

    for _ in range(int(len(available_nodes) * density)):

        nodes = random.sample(available_nodes, 2)
        map.add(
            (
                nodes[0],
                nodes[1],
                random.randint(weights_range[0], weights_range[1])
            )
        )

    print(f"Mapa gerado com {int(len(available_nodes) * density)} conexoes")
    return map
