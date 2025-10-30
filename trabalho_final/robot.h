#ifndef ROBOT_H
#define ROBOT_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

// Constants
#define PI 3.14159265358979323846
#define NUM_ESTADOS_X 11
#define NUM_ESTADOS_Y 11
#define NUM_ACOES_V 11
#define NUM_ACOES_ROT 6
#define NUM_ACOES (NUM_ACOES_V * NUM_ACOES_ROT)

// Estrutura para armazenar vetores 2D
typedef struct {
    double x;
    double y;
} Vector2D;

// Estrutura para armazenar vetores 3D
typedef struct {
    double x;
    double y;
    double theta;
} Vector3D;

// Estrutura para armazenar estado discreto
typedef struct {
    int idx_x;
    int idx_y;
} EstadoDiscreto;

// Estrutura para resultado de estados discretos
typedef struct {
    EstadoDiscreto estado;
    Vector2D erros_norm;
    double dist_alvo;
} ResultadoEstadoDiscreto;

// Estrutura para armazenar histórico de treinamento
typedef struct {
    int episodio;
    int passos;
} HistoricoItem;

// Estrutura principal do robô
typedef struct {
    // Características do robô
    double d;           // distância à frente do robô
    double vmax;        // velocidade máxima de translação
    double rotmax;      // velocidade máxima de rotação

    // Parâmetros da simulação
    double h;           // passo de integração
    int caso;           // 1: atualiza erro direto, 2: atualiza pose e recalcula erro

    // Hiperparâmetros do Q-Learning
    double alfa;                    // Taxa de aprendizado
    double gama;                    // Fator de desconto
    double epsilon;                 // Epsilon atual
    double epsilon_inicial;
    double epsilon_min;
    double taxa_decaimento;
    int num_episodios;
    int passos_por_episodio;
    double dist_tolerancia;

    // Discretização
    double faixas_x[NUM_ESTADOS_X];
    double faixas_y[NUM_ESTADOS_Y];
    int centro_x;
    int centro_y;
    int num_estados_x;
    int num_estados_y;

    // Ações
    double v_valores[NUM_ACOES_V];
    double rot_valores[NUM_ACOES_ROT];
    int num_acoes_v;
    int num_acoes_rot;
    int num_acoes;

    // Tabela Q - array 3D: [x][y][acao]
    double ***Q_table;
} RobotQLearning;

// Funções de inicialização e destruição
RobotQLearning* criar_robot(int caso);
void destruir_robot(RobotQLearning* robot);
void alocar_tabela_q(RobotQLearning* robot);
void liberar_tabela_q(RobotQLearning* robot);

// Funções auxiliares de matemática
double rand_uniform(double min, double max);
double fmod_positive(double x, double y);
int max_int(int a, int b);
int min_int(int a, int b);
double max_double(double a, double b);
double min_double(double a, double b);

// Funções de cálculo de estados
ResultadoEstadoDiscreto calc_estados_discretos(RobotQLearning* robot, Vector3D erro_bruto);

// Funções de ação
int escolher_acao(RobotQLearning* robot, EstadoDiscreto estado);
void mapear_acao_para_velocidades(RobotQLearning* robot, int acao, Vector2D erros_norm,
                                   double* v, double* rot);

// Funções de cálculo de erro e simulação
Vector3D calcular_erro_inicial(RobotQLearning* robot, Vector2D destino, Vector3D pose);
Vector3D simular_acao_caso1(RobotQLearning* robot, Vector3D Er, Vector2D U);
void simular_acao_caso2(RobotQLearning* robot, Vector3D* pose, Vector2D destino,
                        Vector2D U, Vector3D* Er_novo);

// Funções de recompensa e atualização
double calcular_recompensa(RobotQLearning* robot, EstadoDiscreto estado,
                          EstadoDiscreto estado_proximo);
void atualizar_tabela_q(RobotQLearning* robot, EstadoDiscreto estado, int acao,
                       double recompensa, EstadoDiscreto estado_proximo);

// Função de treinamento
HistoricoItem* treinar(RobotQLearning* robot, int* tamanho_historico);

// Funções de salvamento
void salvar_tabela_q_texto(RobotQLearning* robot, const char* nome_arquivo);

// Função de teste
typedef struct {
    EstadoDiscreto* trajetoria;
    int tamanho;
    int sucesso;
} ResultadoTeste;

ResultadoTeste testar_politica(RobotQLearning* robot, int max_passos);
void liberar_resultado_teste(ResultadoTeste* resultado);

#endif // ROBOT_H
