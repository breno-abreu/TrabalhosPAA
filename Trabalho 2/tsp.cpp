// Universidade Tecnológica Federal do Paraná
// Projeto e Análise de Algoritmos
// Turma: S73
// Segundo Semestre de 2020
// Data: 2021-05-09
// Aluno: Breno Moura de Abreu
// RA: 1561286

#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <climits>
#include <chrono>
#include <string>

using namespace std;

struct Ponto
{
    /* Estrutura que representa um ponto no plano cartesiano com coordenadas (x, y) */
    int x;
    int y;
};

class MinHeap
{
    /* Classe que representa uma estrutura de dados Heap e realiza seus principais métodos */

    private:
    struct No
    {
        /* Estrutura que representa um nó no heap que armazena informações relativas a um vértice no grafo */

        // Indica o valor identificador de um grafo
        int id;         

        // Indica o peso atual do vértice. Utilizado na execução do algoritmo de Prim
        double peso;

        // Indica se um nó já foi ou não visitado no algoritmo de Prim
        bool visitado;
    };

    // Lista que contém a fila de prioridade para indicar qual o próximo vértice a ser visitado
    // no algoritmo de Prim.
    vector<No*> filaPrioridade;

    // Estrutura auxiliar que contém informações sobre todos os vértices do grafo, que também existem
    // na lista 'filaPrioridade' em seu momento inicial. Usada para garantir 
    // acesso direto e, portanto, rápido ao peso e id de um nó, sem precisar
    // percorrer a lista 'filaPrioridade' em uma busca. 
    // Lista contém todos os vértices em todo seu período de existência,
    // diferente da lista 'filaPrioridade' que retira nós ao decorrer da execução do algoritmo.
    vector<No*> listaAux;

    public:
    MinHeap(){}

    ~MinHeap(){limpar();}

    void limpar()
    {
        /* Limpa as listas contendo ponteiros para nós, e libera memória alocada */

        listaAux.clear();

        for(auto no : filaPrioridade)
            delete no;
        
        filaPrioridade.clear();
    }

    void inserir(int id, double peso)
    {
        /* Insere um novo nó na fila de prioridade e na lista auxiliar */

        No* no = new No;
        no->id = id;
        no->peso = peso;
        no->visitado = false;
        filaPrioridade.push_back(no);
        listaAux.push_back(no);
        heapifyBaixo(filaPrioridade.size() - 1);
    }

    int pop()
    {
        /* Retira o primeiro elemento, excluindo-o do heap 
         * Esse algoritmo tem complexidade O(log n) devido ao método 'heapifyCima' */

        if(!filaPrioridade.empty())
        {
            // Encontra o id do primeiro elemento do heap
            int id = filaPrioridade[0]->id;

            filaPrioridade[0]->visitado = true;
            int ultimo = filaPrioridade.size() - 1;

            // Permuta a posição do primeiro e o último elementos do heap
            No* aux = filaPrioridade[ultimo];
            filaPrioridade[ultimo] = filaPrioridade[0];
            filaPrioridade[0] = aux;

            // Exclui o último elemento do heap
            filaPrioridade.erase(filaPrioridade.begin() + ultimo);

            // Posiciona corretamente o elemento permutado anteriormente,
            // que agora se encontra na primeira posição do heap
            heapifyCima(0, filaPrioridade.size());

            // Retorna o valor de id do primeiro elemento da lista
            return id;
        }

        return -1;
    }

    void heapifyBaixo(int indice)
    {
        /* Ao inserir um novo nó é necessário posicioná-lo corretamente no heap.
         * Para tal, adiciona-o na última posição no heap, e compara-o com seu pai.
         * Caso seja menor, troca de posição com seu pai, e repete o algoritmo recursivamente
         * a partir da nova posição. 
         * Esse algoritmo tem complexidade O(log n) */ 
        
        int pai = (indice - 1) / 2;
        if(pai >= 0 && filaPrioridade[indice]->peso < filaPrioridade[pai]->peso)
        {
            No* aux = filaPrioridade[indice];
            filaPrioridade[indice] = filaPrioridade[pai];
            filaPrioridade[pai] = aux;
            heapifyBaixo(pai);
        }
    }

    void heapifyCima(int indice, int tamanho)
    {
        /* Posiciona corretamente um elemento percorrendo o heap de cima para baixo.
         * Algoritmo usado para posicionar elementos corretamente em um heap MÍNIMO. 
         * A complexidade desse algoritmo é O(log n). */

        int menor = indice;
        int dir = 2 * indice + 2;
        int esq = 2 * indice + 1;

        // Encontra o nó à direita e à esquerda e compara com o valor do nó de menor valor
        if(dir < tamanho && filaPrioridade[dir]->peso < filaPrioridade[menor]->peso)
            menor = dir;
        
        if(esq < tamanho && filaPrioridade[esq]->peso < filaPrioridade[menor]->peso)
            menor = esq;
        
        // Caso o menor valor esteja à direita ou à esquerda, permuta o nó atual
        // com o nó de menor valor, e repete o processo recursivamente 
        // a partir da nova posição do nó.
        if(menor != indice)
        {
            No* aux = filaPrioridade[indice];
            filaPrioridade[indice] = filaPrioridade[menor];
            filaPrioridade[menor] = aux;

            heapifyCima(menor, tamanho);
        }
    }

    /*void maxHeapifyCima(int indice, int tamanho)
    {
         Posiciona corretamente um elemento percorrendo o heap de cima para baixo.
         * Algoritmo usado para posicionar elementos corretamente em um heap MÁXIMO. 
         * A complexidade desse algoritmo é O(log n). 

        int menor = indice;
        int dir = 2 * indice + 2;
        int esq = 2 * indice + 1;

        // Encontra o nó à direita e à esquerda e compara com o valor do nó com maior valor.
        if(dir < tamanho && filaPrioridade[dir]->peso > filaPrioridade[menor]->peso)
            menor = dir;
        
        if(esq < tamanho && filaPrioridade[esq]->peso > filaPrioridade[menor]->peso)
            menor = esq;
        
        // Caso o maior valor esteja à direita ou à esquerda, permuta o nó atual
        // com o nó de maior valor, e repete o processo recursivamente 
        // a partir da nova posição do nó.
        if(menor != indice)
        {
            No* aux = filaPrioridade[indice];
            filaPrioridade[indice] = filaPrioridade[menor];
            filaPrioridade[menor] = aux;

            maxHeapifyCima(menor, tamanho);
        }
    }

    void ordenar()
    {
         Ordena um heap que tenha tido um nó alterado
         * Algoritmo possui complexidade O(n log n),
         * pois executa n vezes o algoritmo maxHeapifyCima, que tem complexidade O(log n)

        int tamanho = filaPrioridade.size();
        while(tamanho > 1)
        {
            // Posiciona o primeiro elemento do heap na posição correta.
            // Deixando o nó com maior valor na primeira posição
            maxHeapifyCima(0, tamanho);

            // Permuta o primeiro elemento com o último
            No* aux = filaPrioridade[0];
            filaPrioridade[0] = filaPrioridade[tamanho - 1];
            filaPrioridade[tamanho - 1] = aux;

            // Com os últimos elementos do heap já ordenados, 
            // essa variável irá impedir que estes sejam modificados até
            // o fim da execução do algoritmo.
            tamanho--;
        }
    }*/

    void alterarNo(int id, double peso)
    {
        /* Altera o peso de um nó com determinado id */

        // Acesa o nó na lista auxiliar imediatamente, sem uso de algoritmos de busca.
        // Apenas funciona se o id e seu index forem iguais.
        // Atualiza a posição do nó. 

        listaAux[id]->peso = peso;
        int indice = 0;

        // Procura o índice do nó desejado
        for(int i = 0; i < filaPrioridade.size(); i++)
        {
            if(id == filaPrioridade[i]->id)
                indice = i;
        }

        // Como o peso novo sempre é menor que o peso antigo, é apenas necessário
        // executar o heapifyBaixo.
        heapifyBaixo(indice);
    }

    void imprimirNos()
    {
        /* Imprime todos os nós presentes na fila de prioridade */

        for(auto no : filaPrioridade)
            cout << no->id << ":" << no->peso << "  ";

        cout << endl;
    }

    bool vazio()
    {
        /* Retorna se a fila de prioridade está vazia ou não */

        return filaPrioridade.empty();
    }

    double getPeso(int id)
    {
        /* Retorna o peso de um determinado nó */

        // Acesa o nó na lista auxiliar imediatamente, sem uso de algoritmos de busca.
        // Apenas funciona se o id e seu index forem iguais.
        if(listaAux[id]->visitado == false)
            return listaAux[id]->peso;

        return -1;
    }
};

class Grafo
{
    /* Estrutura que representa um grafo e todos os seus métodos
     * incluindo a execução do algoritmo de Prim e da busca em profundidade */

    private:
    struct Vizinho
    {
        /* Estrutura que representa um vizinho de um vértice */

        // Identificador do vizinho
        int id;  

        // Distância entre o vértice e esse vizinho
        double distancia;
    };

    class Vertice
    {
        /* Estrutura que representa um vértice do grafo */

        public:

        // Identificador do vértice
        int id;

        // Coordenada X do vértice
        int x;

        // Coordenada Y do vértice
        int y;

        // Identificador do vértice predecessor de um vértice, usado no algoritmo de Prim
        int idPi;

        // Indica se o vértice foi, ou não, visitado em algum algoritmo
        bool visitado;

        // Lista de vizinhos de um vértice. Lista de adjacência
        vector<Vizinho*> vizinhos;

        Vertice(int _id, int _x, int _y)
        {
            /* Cria um novo vértice */

            id = _id;
            x = _x;
            y = _y;
        }

        void adicionarVizinho(int id, double distancia)
        {
            /* Adiciona um vizinho, recebendo seu id e a distância com o vértice */
            
            Vizinho* nova = new Vizinho;
            nova->id = id;
            nova->distancia = distancia;
            vizinhos.push_back(nova);
        }
    };

    // Indica a quantidade de vértices num grafo.
    // Indica o ID de um novo grafo
    int qtdVertices;

    // Lista de vértices presentes em um grafo
    vector<Vertice*> vertices;

    // Lista de ids do resultado da busca em profundidade
    vector<int> resDFS;

    
    public:
    Grafo(vector<Ponto*> pontos, char opcao)
    {
        /* Cria um grafo com o uso de uma lista de pontos.
         * Opção 'c' cria um grafo completo */

        qtdVertices = 0;

        for(auto ponto : pontos)
            adicionarVertice(ponto->x, ponto->y);

        if(opcao == 'c')
            criarGrafoCompleto(pontos);
    }

    Grafo(){qtdVertices = 0;}

    ~Grafo(){limparGrafo();}

    void limparGrafo()
    {
        /* Limpa a lista de vértices e a lista de vizinhos para cada vértice
         * liberando memória alocada */

        for(auto vertice : vertices)
        {
            for(auto vizinho : vertice->vizinhos)
                delete vizinho;
                
            delete vertice;
        }
        vertices.clear();
    }

    void adicionarVertice(int x=0, int y=0)
    {
        /* Adiciona um vértice no grafo */

        Vertice* novo = new Vertice(qtdVertices, x, y);
        vertices.push_back(novo);
        qtdVertices++;
    }

    double calcularDistancia(Vertice* a, Vertice* b)
    {
        /* Calcula a distância euclidiana entre dois vértices */

        return sqrt(pow(a->x - b->x, 2) + pow(a->y - b->y, 2));
    }

    void criarGrafoCompleto(vector<Ponto*> pontos)
    {
        /* Cria um grafo completo, criando arestas entre cada vértice para todos os demais
         * Complexidade é O(n^2) sendo n o número total de vértices
         * pois apresenta um laço que percorre todos os vértices dentro de 
         * outro laço que percorre todos os vértices para criar todas as arestas */

        // Complexidade O(n)
        for(int i = 0; i < vertices.size() - 1; i++)
            // Complexidade O(n)
            for(int j = i + 1; j < vertices.size(); j++)
                adicionarAresta(i, j);
    }

    void imprimirVertices()
    {
        /* Imprime o id de um vértice, o id de todos os seus vizinhos e a distância entre eles */

        for(auto vertice : vertices)
        {
            cout << vertice->id << ":: ";
            for(auto vizinho : vertice->vizinhos)
            {
                cout << fixed << setprecision(5) << vizinho->id << ": " << vizinho->distancia << " \t| ";
            }
            cout << endl;
        }
    }

    void adicionarAresta(int a, int b)
    {
        /* Adiciona uma aresta não direcionada entre dois vértices
         * com id 'a' e id 'b', e encontra o peso da aresta como a distância
         * euclidiana entre os vértices */

        if(a >= 0 && a < vertices.size() && b >= 0 && b < vertices.size())
        {
            double dist = calcularDistancia(vertices[a], vertices[b]);
            vertices[a]->adicionarVizinho(vertices[b]->id, dist);
            vertices[b]->adicionarVizinho(vertices[a]->id, dist);
        }
    }

    void adicionarAresta(int a, int b, double peso)
    {
        /* Adiciona uma aresta não direcionada entre dois vértices
         * com id 'a' e id 'b', e adiciona um peso que não necessariamente é a
         * distância euclidiana entre os vértices */

        if(a >= 0 && a < vertices.size() && b >= 0 && b < vertices.size())
        {
            vertices[a]->adicionarVizinho(vertices[b]->id, peso);
            vertices[b]->adicionarVizinho(vertices[a]->id, peso);
        }
    }

    Grafo execPrim()
    {
        /* Executa o algoritmo de Prim para encontrar a Árvore Geradora Mínima de um grafo
         * Possui complexidade (A log V), em que A -> número de arestas e V -> número de vértices 
         * Caso fosse utilizado um heap de Fibonacci, a compelexidade poderia ser melhorada para
         * O(E + log V), o que não foi o caso neste projeto em que um simples heap binário foi utilizado. */

        MinHeap heap;

        // Insere todos os vértices em um heap com peso "infinito". 
        // Complexidade O(V log V), pois para cada vértice executa o algoritmo de 
        // inserção do heap, que possui complexidade O(log V)
        for(auto vertice : vertices)
        {
            heap.inserir(vertice->id, INT_MAX);
            vertice->idPi = -1;
        }
            
        int id = 0;

        // Altera o peso do primeiro nó para 0. Complexidade O(log V)
        heap.alterarNo(id, 0);

        // Para cada vértice atualiza o peso de seus vizinhos caso o peso
        // seja menor que o valor atual contido em um vizinho
        // Este laço possui complexidade O(V)
        while(!heap.vazio())
        {
            //heap.imprimirNos();

            // Para cada vértice executa o algoritmo que retira o primeiro nó
            // de uma fila de prioridade, portanto, possui compelxidade O(V log V)
            id = heap.pop();

            // Percorre todos os vizinhos de um vértice, ou seja, todas as arestas,
            // portanto possui complexidade O(E)
            for(auto vizinho : vertices[id]->vizinhos)
            {
                if(heap.getPeso(vizinho->id) > vizinho->distancia)
                {
                    // Ao se alterar uma aresta, utiliza-se um algoritmo com complexidade
                    // O(log V), pois utiliza a fila de prioridade.
                    // Como esse algoritmo será executado para todas as arestas, 
                    // sua complexidade total é O(E log V)
                    heap.alterarNo(vizinho->id, vizinho->distancia);
                    vertices[vizinho->id]->idPi = id;
                }
            }
        }

        // A complexidade de tempo do algoritmo é O(E log V) pois é um grafo conexo

        // Cria a árvore geradora mínima resultante da execução do algoritmo de Prim
        Grafo prim = Grafo();

        // Adiciona os vértices na árvore
        for(auto vertice : vertices)
            prim.adicionarVertice(vertice->x, vertice->y);

        // Adiciona as arestas na árvore
        for(auto vertice : vertices)
            prim.adicionarAresta(vertice->id, vertice->idPi);
            
        // Retorna a árvore resultante da execução do algoritmo de Prim sobre um grafo
        return prim;
    }

    void imprimirResPrim()
    {
        /* Imprime todos os vértices, e para cada vértice, seu predecessor */

        for(auto vertice : vertices)
            cout << vertice->id << ":" << vertice->idPi << "  ";
        
        cout << endl;
    }

    void execBuscaEmProfundidade()
    {
        /* Executa a busca em profundidade no grafo
         * O algoritmo tem complexidade O(V + A)
         * sendo V -> número de vértices; e A -> número de arestas
         * O algoritmo percorre todos os vértices uma vez, e a partir de um vértice
         * percorre todos os seus vizinhos. Com isso, temos que a complexidade
         * de percorrer todas as arestas é O(V). Então visita-se todos os seus
         * vizinhos apenas uma vez. Uma visita para cada aresta.
         * Como há dois vértices que compartilham a mesma aresta o algoritmo
         * irá percorrer todas as arestas duas vezes, concluíndo a complexidade O(2A)
         * que é igual a O(A). Conclui-se que que o algoritmo irá percorrer
         * todos os vértices mais todas as arestas do grafo, logo possui 
         * complexidade O(V + A). */

        resDFS.clear();

        // Define para todos os vértices que estes não foram visitados ainda
        for(auto vertice : vertices)
            vertice->visitado = false;

        // Percorre todas as arestas. Complexidade O(V), sendo V -> número total de vértices
        for(auto vertice : vertices)
        {
            if(vertice->visitado == false)
                DFS(vertice->id);
        }
    }

    void DFS(int id)
    {
        /* Com o id de um vértice recebido, executa a busca em profundidade
         * chamando recursivamente a função para os vizinhos do vértice */

        // Encontra o vértice com id recebido
        Vertice* v = vertices[id];
        v->visitado = true;

        // Adiciona o vértice na lista do resultado da busca em profundidade
        // de acordo com a ordem de visita
        resDFS.push_back(v->id);

        // Encontra vizinhos do vértice que não foram visitados e executa a busca
        // em profundidade recursivamente.
        // Ao realizar para todos os vértices, a compelxidade é O(A)
        // sendo A -> número total de arestas
        for(auto vizinho : v->vizinhos)
        {
            if(vertices[vizinho->id]->visitado == false)
                DFS(vizinho->id);
        }
    }

    void imprimirResDFS()
    {
        /* Imprime o resultado da busca em profundidade */

        if(resDFS.empty() == false)
        {
            for(auto res : resDFS)
                cout << res + 1 << " ";

            cout << endl;
        }
        else
            cout << "DFS não foi executado!" << endl;
    }

    void escreverAGM()
    {
        /* Escreve a Árvore Geradora Mínima em um arquivo de texto */

        ofstream Arquivo("tree.txt");

        for(auto vertice : vertices)
            vertice->visitado = false;

        // Percorre todos os vértices e seus vizinhos, escrevendo sua coordenada no arquivo
        // caso ainda não tenham sido visitados
        for(auto vertice : vertices)
        {
            vertice->visitado = true;
            for(auto vizinho : vertice->vizinhos)
            {
                if(vertices[vizinho->id]->visitado == false)
                {
                    Arquivo << vertice->x << " " << vertice->y << endl;
                    Arquivo << vertices[vizinho->id]->x << " " << vertices[vizinho->id]->y << endl;
                    //Arquivo << endl;
                }
            } 
        }
        Arquivo.close();
    }

    void escreverCiclo()
    {
        /* Escreve a resposta da busca em profundidade em um arquivo de texto */

        ofstream Arquivo("cycle.txt");
            
        for(auto res : resDFS)
            Arquivo << vertices[res]->x << " " << vertices[res]->y << endl;

        // Adiciona as coordenadas do primeiro nó para completar o ciclo
        Arquivo << vertices[0]->x << " " << vertices[0]->y;

        Arquivo.close();
    }

    double getTotal(vector<int> ciclo)
    {
        /* Percorre o grafo e soma as distâncias entre os vértices
         * na ordem em que estão na lista 'ciclo' de ids recebida */

        double total = 0;

        // Adiciona o id do nó inical para completar o ciclo
        ciclo.push_back(0);

        // Percorre todos os vértices, e soma a distância entre o vértice atual
        // e o próximo vértice na lista
        for(int i = 0; i < ciclo.size() - 1; i++)
        {
            for(auto vizinho : vertices[ciclo[i]]->vizinhos)
            {
                if(vizinho->id == ciclo[i + 1])
                    total += vizinho->distancia;
            }
        }

        // Retorna a soma total das distâncias entre todos os vértices 
        // de acordo com a lista recebida
        return total;
    }

    vector<int> getResDFS()
    {
        /* Retorna uma lista com o resultado encontrado na busca em profundidade
         * na ordem de visita do algoritmo */

        return resDFS;
    }
};

vector<Ponto*> criarListaDePontos(string nomeArquivo)
{
    /* A partir de um arquivo de texto localizado em 'nomeArquivo',
     * cria uma lista de pontos que se tornarão vértices em um grafo */

    ifstream arquivo(nomeArquivo);
    string texto = "";
    string valor = "";

    getline(arquivo, texto);
    int qtdPontos = stoi(texto);

    vector<Ponto*> pontos;

    int i = 0;

    while(getline(arquivo, texto))
    {
        i = 0;
        valor = "";

        while(i < texto.length() && texto[i] != ' ')
            valor += texto[i++];

        i++;
        Ponto *p = new Ponto;
        p->x = stof(valor);
        valor = "";

        while(i < texto.length())
            valor += texto[i++];
        
        p->y = stod(valor);
        pontos.push_back(p);
    }

    arquivo.close();
    return pontos;
}

int main(int argc, char *argv[])
{
    if(argc == 2)
    {
        string arquivo = argv[1];
        using clock = chrono::system_clock;
        using sec = chrono::duration<double>;
        double tempo = 0;

        // Inicia o relógio
        auto antes = clock::now();

        // Cria uma lista de pontos
        vector<Ponto*> pontos = criarListaDePontos(arquivo);

        // Envia a lista de pontos para o grafo, que irá transformá-las
        // em vértices e criar um grafo completo, isto é, todos os vértices são
        // interligados entre si por arestas.
        Grafo grafo = Grafo(pontos, 'c');

        // Executa o algoritmo de Prim no grafo, e envia o resultado, isto é, 
        // a árvore geradora mínima que será representada por um novo grafo.
        Grafo prim = grafo.execPrim();

        // Escreve o resultado da árvore geradora mínima em um arquivo de texto
        prim.escreverAGM();

        // Executa a busca em profundidade no grafo contendo a AGM, 
        // encontrando o ciclo entre os vértices.
        prim.execBuscaEmProfundidade();

        // Escreve o ciclo encontrado em um arquivo de texto
        prim.escreverCiclo();

        // Para o relógio e transforma sua saída para segundos
        auto duracao = clock::now() - antes;
        tempo = chrono::duration_cast<chrono::nanoseconds>(duracao).count();
        tempo /= 1000000000;

        // Imprime o tempo de execução e o valor da soma das arestas do ciclo
        cout << fixed << setprecision(6) << tempo << " " << grafo.getTotal(prim.getResDFS()) << endl;
    }
    else
        cout << "[COMANDO ERRADO] defina o arquivo de entrada" << endl;
    
}