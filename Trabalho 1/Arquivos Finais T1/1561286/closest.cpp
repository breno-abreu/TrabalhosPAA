#include <iostream>
#include <cmath>
#include <cfloat>
#include <fstream>
#include <string>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <chrono>

using namespace std;

/* Estrutura para um ponto contendo suas coordenadas */
struct Ponto
{
    double x;
    double y;
};

/* Estrutura para um par de pontos e a distância entre eles */
struct ParPontos
{
    Ponto p1;
    Ponto p2;
    double dist;
};

/* Estrutura contendo os pares de pontos encontrados nos dois algoritmos e seus tempos de execução */
struct Resultados
{
    ParPontos resForcaBruta;
    ParPontos resDivisaoConquista;
    double tempoForcaBruta;
    double tempoDivisaoConquista;
};

ParPontos forcaBruta(vector<Ponto*> pontos)
{
    /* Algoritmo que encontra os dois pontos mais próximos através de força bruta */
    double dist = 0;
    ParPontos resultado;
    resultado.dist = DBL_MAX;

    /* Compara cada ponto com todos os demais pontos */
    for(int i = 0; i < pontos.size() - 1; i++)
    {
        for(int j = i + 1; j < pontos.size(); j++)
        {
            dist = sqrt(pow(pontos.at(i)->x - pontos.at(j)->x, 2) + 
                        pow(pontos.at(i)->y - pontos.at(j)->y, 2));
            
            if(dist < resultado.dist)
            {
                resultado.dist = dist;
                resultado.p1 = *pontos.at(i);
                resultado.p2 = *pontos.at(j);
            }
        }
    }

    /* Retorna o par de pontos mais próximos */
    return resultado;
}

bool compararPorX(const Ponto* a, const Ponto* b)
{
    /* Função para tornar possível ordenar uma lista de pontos através da coordenada X */
    return a->x < b->x;
}

bool compararPorY(const Ponto* a, const Ponto* b)
{
    /* Função para tornar possível ordenar uma lista de pontos através da coordenada Y */
    return a->y < b->y;
}

void limparVetor(vector<Ponto*> *v)
{
    /* Deleta todos os ponteiros em uma lista de ponteiros para um Ponto e limpa a lista */
    for(auto no : *v)
        delete no;

    v->clear();
}

void popularSubVetoresY(int meio, double xLinhaVertical, vector<Ponto*> *pontosY,
                        vector<Ponto*> *pontosYE, vector<Ponto*> *pontosYD)
                       
{
    /* Ao receber uma lista de pontos ordenada em Y divide a lista no meio
     * populando a lista da esquerda e a lista da direita */
    int aux = 0;

        for(auto no : *pontosY)
        {
            Ponto *p = new Ponto;
            p->x = no->x;
            p->y = no->y;

            /* Caso o ponto esteja na parte esquerda da linha vertical divisória
             * inclui-o na lista da esquerda, caso contrário, na lista da direita. 
             * Como a lista principal já está ordenada, as listas populadas também estarão */
            if(no->x <= xLinhaVertical && aux <= meio)
            {
                pontosYE->push_back(p);
                aux++;
            }
            else
                pontosYD->push_back(p);            
        }
}   

void popularSubVetoresX(int meio, vector<Ponto*> *pontosX,
                        vector<Ponto*> *pontosXE, vector<Ponto*> *pontosXD)
{
    /* Ao receber uma lista ordenada em X, divide a lista no meio populando 
     * a lista da esquerda e da direita */
    int cont = 0;

        for(auto no : *pontosX)
        { 
            Ponto *p = new Ponto;
            p->x = no->x;
            p->y = no->y;

            /* Caso o ponto esteja na parte esquerda da linha vertical divisória
             * inclui-o na lista da esquerda, caso contrário, na lista da direita. 
             * Como a lista principal já está ordenada, as listas populadas também estarão */
            if(cont <= meio)
                pontosXE->push_back(p);
            else
                pontosXD->push_back(p);

            cont++;
        }
}

void popularSubVetorFaixa(double xLinhaVertical, double dist, 
                          vector<Ponto*> *pontosY, vector<Ponto*> *naFaixa)
{
    /* Ao receber uma lista de pontos ordenados em Y, encontra todos os 
     * pontos que estão a uma distância máxima 'dist' da linha vertical divisória
     * e inclui-os na lista 'naFaixa' */
    for(auto no : *pontosY)
    {
        if(abs(no->x - xLinhaVertical) < dist)
        {
            Ponto *p = new Ponto;
            p->x = no->x;
            p->y = no->y;
            naFaixa->push_back(p);
        }
    }
}

ParPontos menorPontoNaFaixa(vector<Ponto*> *naFaixa)
{
    /* Encontra a menor distância entre dois pontos que estão separados
     * pela linha vertical divisória */
    double dist = 0;
    ParPontos resFaixa;
    resFaixa.dist = DBL_MAX;

    for(int i = 0; i < naFaixa->size(); i++)
    {  
        /* Dois pontos separados por uma distância menor que a distância máxima
         * entre a linha divisória estão a, no máximo, 7 posições de distância
         * logo, é necessário apenas comparar um ponto com os 7 pontos subsequentes */
        for(int j = 1; j <= 7 && i + j < naFaixa->size(); j++)
        {
            dist = sqrt(pow(naFaixa->at(i)->x - naFaixa->at(i + j)->x, 2) + 
                        pow(naFaixa->at(i)->y - naFaixa->at(i + j)->y, 2));
                
            if(dist < resFaixa.dist)
            {
                resFaixa.dist = dist;
                resFaixa.p1 = *naFaixa->at(i);
                resFaixa.p2 = *naFaixa->at(i + j);
            }
        }
    }

    return resFaixa;
}

ParPontos divisaoConquista(vector<Ponto*> pontosX, vector<Ponto*> pontosY)
{
    if(pontosX.size() <= 3)
        /* Caso a lista tenha três ou menos de três elementos, resolve por força bruta */
        return forcaBruta(pontosX);
    
    else
    {
        int meio = (pontosX.size() + 1) / 2 - 1;
        double xLinhaVertical = (pontosX.at(meio)->x + pontosX.at(meio + 1)->x) / 2;

        /* Cria duas listas, uma que recebe a primeira metade dos elementos
         * em 'pontosY' e outra que recebe a segunda metade de acordo com a 
         * posição dos pontos em relação à faixa vertical divisória */
        vector<Ponto*> pontosYE;
        vector<Ponto*> pontosYD;
        popularSubVetoresY(meio, xLinhaVertical, &pontosY, &pontosYE, &pontosYD);

        /* Cria duas listas, uma que recebe a primeira metade dos elementos
         * em 'pontosX' e outra que recebe a segunda metade */
        vector<Ponto*> pontosXE;
        vector<Ponto*> pontosXD;
        popularSubVetoresX(meio, &pontosX, &pontosXE, &pontosXD);

        /* Envia as listas recursivamente para encontrar o par de pontos
         * com menor distância na parte esquerda e direita da faixa divisória */
        ParPontos resEsq = divisaoConquista(pontosXE, pontosYE);
        ParPontos resDir = divisaoConquista(pontosXD, pontosYD);

        /* Encontra o par de pontos com menor distância entre o menor ponto
         * encontrado na lista da direita e da esquerda */
        ParPontos minEsqDir = resEsq.dist < resDir.dist ? resEsq : resDir;

        /* Encontra todos os pontos que estão a uma distância menor que 'minEsqDir' da faixa divisória */
        vector<Ponto*> naFaixa;
        popularSubVetorFaixa(xLinhaVertical, minEsqDir.dist, &pontosY, &naFaixa);

        /* Encontra o par de pontos com menor ditância entre pontos separados pela faixa vertical divisória */
        ParPontos resFaixa = menorPontoNaFaixa(&naFaixa);

        /* Limpa todos os vetores, liberando memória alocada */
        limparVetor(&pontosXE);
        limparVetor(&pontosXD);
        limparVetor(&pontosYE);
        limparVetor(&pontosYD);
        limparVetor(&naFaixa);

        /* Retorna o par de pontos com a menor distância */
        return minEsqDir.dist < resFaixa.dist ? minEsqDir : resFaixa;
    }
}

/*ParPontos divisaoConquista(vector<Ponto*> pontos)
{
    sort(pontos.begin(), pontos.end(), compararPorX);
    vector<Ponto*> pontosX = pontos;

    sort(pontos.begin(), pontos.end(), compararPorY);
    vector<Ponto*> pontosY = pontos;

    return divisaoConquistaRec(pontosX, pontosY);
}*/

int getQtdPontos(string nomeArquivo)
{
    /* Encontra a quantidade de pontos presentes em um arquivo de texto gerado
     * pelo programa 'genpoint' */
    ifstream arquivo(nomeArquivo);
    string texto = "";
    getline(arquivo, texto);
    return stoi(texto);
}

vector<Ponto*> popularMatriz(string nomeArquivo)
{
    /* Ao receber o nome de um arquivo de texto gerado pelo programa 'genpoint', 
     * lê todos os pontos escritos no arquivo e cria estruturas do tipo Ponto, 
     * incluindo-os em uma lista de ponteiros para pontos */
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

Resultados executarAlgoritmos(string nomeArquivo)
{
    /* Executa os dois algoritmos, encontra seus resultados e seus tempos de execução */
    using clock = chrono::system_clock;
    using sec = chrono::duration<double>;
    vector<Ponto*> pontos;
    ParPontos resultadoDC;
    ParPontos resultadoFB;
    double tempoFB = 0;
    double tempoDC = 0;

    /* Cria uma lista de pontos baseado nos pontos presentes no arquivo de texto
     * gerado pelo programa 'genpoint' */
    pontos = popularMatriz(nomeArquivo);

    /* Ordena a lista de pontos de acordo com a coordenada x dos pontos */
    sort(pontos.begin(), pontos.end(), compararPorX);
    vector<Ponto*> pontosX = pontos;

    /* Ordena a lista de pontos de acordo com a coordenada y dos pontos */
    sort(pontos.begin(), pontos.end(), compararPorY);
    vector<Ponto*> pontosY = pontos;

    /* Executa o algoritmo de força bruta e registra seu tempo de execução em segundos */
    auto antes = clock::now();
    resultadoFB = forcaBruta(pontos);
    auto duracao = clock::now() - antes;
    tempoFB = chrono::duration_cast<chrono::nanoseconds>(duracao).count();
    tempoFB /= 1000000000;

    /* Executa o algoritmo de divisão e conquista e registra seu tempo de execução em segundos */
    antes = clock::now();
    resultadoDC = divisaoConquista(pontosX, pontosY);
    duracao = clock::now() - antes;
    tempoDC = chrono::duration_cast<chrono::nanoseconds>(duracao).count();
    tempoDC /= 1000000000;

    /* Limpa a lista principal, liberando memória alocada */
    limparVetor(&pontos);

    /* Registra os resultados obtidos */
    Resultados resultadoFinal;
    resultadoFinal.resForcaBruta = resultadoFB;
    resultadoFinal.resDivisaoConquista = resultadoDC;
    resultadoFinal.tempoForcaBruta = tempoFB;
    resultadoFinal.tempoDivisaoConquista = tempoDC;

    /* Retorna os resultados obtidos */
    return resultadoFinal;
}

string realizarNTestes(int n, string tipo, string nomeArquivo)
{
    /* Função auxiliar para gerar o resultado referente à diferentes números de entradas
     * processados por diferentes métodos. Registra seus resultados em uma string
     * para ser escrita em um arquivo de texto. 
     * Esta função é utilizada para auxiliar a geração de gráficos criados através
     * de um script em Python */
    string resultado = "";
    string comando = "";
    int quantidade = 0;
    Resultados res;

    if(tipo == "p")
    {
        /* Cria arquivos gerados pelo programa 'genpoint' n vezes com 2^n entradas
         * e executa os dois algoritmos registrando seus resultados em uma string */
        for(int i = 1; i <= n; i++)
        {
            quantidade = pow(2, i);
            cout << "2^" << i << endl;
            comando = "./gp " + to_string(quantidade);
            system(comando.c_str());

            res = executarAlgoritmos(nomeArquivo);
            resultado += "2^" + to_string(i) + " " + to_string(res.tempoForcaBruta) + 
                     " " + to_string(res.tempoDivisaoConquista) + "\n";
        }
    }
    
    else if(tipo == "d")
    {
        /* Cria arquivos gerados pelo programa 'genpoint' n vezes com entradas que variam
         * em 10 em relação à quantidade de pontos do último arquivo gerado
         * e executa os dois algoritmos registrando seus resultados em uma string */
        for(int i = 0; i < n; i++)
        {
            quantidade += 10;
            cout << quantidade << endl;
            comando = "./gp " + to_string(quantidade);
            system(comando.c_str());

            res = executarAlgoritmos(nomeArquivo);
            resultado += to_string(quantidade) + " " + to_string(res.tempoForcaBruta) + 
                     " " + to_string(res.tempoDivisaoConquista) + "\n";
        }
    }

    else if(tipo == "u")
    {
        /* Cria arquivos gerados pelo programa 'genpoint' n vezes com entradas que variam
         * em 1 em relação à quantidade de pontos do último arquivo gerado, além disso, 
         * executa cada conjunto de pontos 1000 vezes para encontrar sua média de tempo
         * e executa os dois algoritmos registrando seus resultados em uma string */
        double mediaFB = 0;
        double mediaDC = 0;
        double totalFB = 0;
        double totalDC = 0;

        for(int i = 0; i < n; i++)
        {
            quantidade += 1;
            cout << quantidade << endl;
            comando = "./gp " + to_string(quantidade);
            system(comando.c_str());
            mediaFB = 0;
            mediaDC = 0;
            totalFB = 0;
            totalDC = 0;

            for(int i = 0; i < 1000; i++)
            {
               res = executarAlgoritmos(nomeArquivo);
               totalFB += res.tempoForcaBruta;
                totalDC += res.tempoDivisaoConquista;
            }

            res.tempoForcaBruta = totalFB / 1000;
            res.tempoDivisaoConquista = totalDC / 1000;
        
            resultado += to_string(quantidade) + " " + to_string(res.tempoForcaBruta) + 
                        " " + to_string(res.tempoDivisaoConquista) + "\n";
        }
    }

    else
        cout << "Tipo Inválido" << endl;

    return resultado;
}

void escreverResultados(string resultados, string nomeArquivo="output.txt")
{
    /* Escreve o resultado encontrado, registrado em uma string, em um arquivo de texto
     * O arquivo gerado possibilita a criação de gráficos criados através de um script em Python */
    ofstream arquivo(nomeArquivo);
    arquivo << resultados;
    arquivo.close();
}

int main(int argc, char *argv[])
{
    if(argc == 2)
    {   
        /* Executa o modo principal do programa, mostrando o resultado
         * da execução dos dois algortimos: os pontos encontrados, a distância entre eles
         * e o tempo de execução. */
        string nomeArquivo = argv[1];
        Resultados res = executarAlgoritmos(nomeArquivo);
        cout << fixed << setprecision(7)
        << res.tempoForcaBruta << " " << res.resForcaBruta.dist
        << " " << res.resForcaBruta.p1.x << " " << res.resForcaBruta.p1.y
        << " " << res.resForcaBruta.p2.x << " " << res.resForcaBruta.p2.y
        << " " << res.tempoDivisaoConquista << " " << res.resDivisaoConquista.dist
        << " " << res.resDivisaoConquista.p1.x << " " << res.resDivisaoConquista.p1.y
        << " " << res.resDivisaoConquista.p2.x << " " << res.resDivisaoConquista.p2.y << endl;
    }
    
    else if(argc == 5 && argv[2] > 0)
    {
        /* Caso requerido pelo usuário, executa algum dos modos alternativos
         * de execução do programa, usado para gerar gráficos através de
         * um script em Python */
        string nomeArquivoEntrada = argv[1];
        string nomeArquivoSaida = argv[2];
        int qtd = atoi(argv[3]);
        string tipo = argv[4];
        string res = realizarNTestes(qtd, tipo, nomeArquivoEntrada);
        escreverResultados(res, nomeArquivoSaida);
    }
    else
    {
        /* Caso os argumentos tenham sido inseridos incorretamente, mostra uma mensagem de erro */
        cout << "Erro" << endl
        << "Argumentos devem ser no formato: "<< endl
        << "./closest <nome do arquivo de entrada> ou" << endl
        << "./closest <nome do arquivo de entrada> <nome do arquivo de saida> <quantidade de iterações> <tipo do cálculo>" << endl;
    }
}