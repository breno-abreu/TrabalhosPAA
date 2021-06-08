from matplotlib import pyplot as plt
import sys

plt.style.use('fast')

nome_arquivo = 'output.txt';

if len(sys.argv) == 2:
    nome_arquivo = sys.argv[1]

arquivo = open(nome_arquivo, 'r')

quantidade = []
forca_bruta = []
divisao_conquista = []

for linha in arquivo:
    campos = linha.split()
    quantidade.append(int(campos[0]))
    forca_bruta.append(float(campos[1]))
    divisao_conquista.append(float(campos[2]))

arquivo.close()

plt.plot(quantidade, forca_bruta, label='Força Bruta')
plt.plot(quantidade, divisao_conquista, label='Divisão e Conquista')

plt.xlabel('Tamanho do Vetor')
plt.ylabel('Tempo (s)')
plt.title('Comparação Entre Força Bruta e Divisão e Conquista')
plt.legend()
plt.grid(True)

plt.show()
