# Trabalho de IA

Neste trabalho analisamos o desempenho dos principais algoritmos de busca da literatura no problema de mapas.

O problema de mapas consiste em encontrar o caminhos mais curto (com menor custo) entre dos pontos de um mapas.

Foi implementado uma classe de grafo para a representação dos mapas, além de um gerador de instâncias para a realização dos experimentos.

## Algoritmos

Os algortimos implementados foram:

- Irrevogável;

- Backtracking;

- Busca em Largura;

- Busca em Profundide;

- Busca Ordenada;

- A\*;

- IDA\*

## Execução

Para rodar executar o programa acessar o terminal na pasta do projeto e executa o comando:

`bashpython main.py`

e o programa irá rodar todos os algoritmos de busca 10 vezes para cada instância, sendo essas de tamanho 25, 50, 100 e 200. Os resultados serão armazenados em "results.csv" na pasta "outputs".
