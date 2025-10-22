#ifndef ROBOT_QLEARNING_H
#define ROBOT_QLEARNING_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

typedef struct {
    double *data;
    int x_size;
    int y_size;
    int a_size;
} QTable;

typedef struct {
    double d;
    double vmax;
    double rotmax;
    double h;
    int caso;

    double alfa;
    double gama;
    double epsilon_inicial;
    double epsilon_min;
    double taxa_decaimento;
    double epsilon;

    int num_episodios;
    int passos_por_episodio;
    double dist_tolerancia;

    double *faixas_x;
    double *faixas_y;
    int centro_x;
    int centro_y;
    int num_estados_x;
    int num_estados_y;

    double *v_valores;
    double *rot_valores;
    int num_acoes_v;
    int num_acoes_rot;
    int num_acoes;

    QTable Q_table;
} RobotQLearningV2;

typedef struct {
    char **formatos_suportados;
    int num_formatos;
} TabelaQManager;

// ====== RobotQLearningV2 ======
RobotQLearningV2 *robot_create(int caso);
void robot_free(RobotQLearningV2 *r);

void robot_init_qtable(RobotQLearningV2 *r);
void robot_train(RobotQLearningV2 *r);
void robot_test_policy(RobotQLearningV2 *r);

void robot_calc_estados_discretos(RobotQLearningV2 *r, double erro_bruto[3],
                                  int estado_discreto[2], double erros_norm[2], double *dist_alvo);
int robot_escolher_acao(RobotQLearningV2 *r, int estado_discreto[2]);
void robot_mapear_acao_para_velocidades(RobotQLearningV2 *r, int acao,
                                        double erros_norm[2], double *v, double *rot);
void robot_atualizar_tabela_q(RobotQLearningV2 *r, int estado_discreto[2], int acao,
                              double recompensa, int estado_proximo_discreto[2]);

void robot_calcular_erro_inicial(RobotQLearningV2 *r, double destino[2], double pose[3], double Er[3]);
void robot_simular_acao_caso1(RobotQLearningV2 *r, double Er[3], double U[2], double Er_novo[3]);
void robot_simular_acao_caso2(RobotQLearningV2 *r, double pose[3], double destino[2],
                              double U[2], double Er_novo[3], double pose_novo[3]);
double robot_calcular_recompensa(RobotQLearningV2 *r, int estado_discreto[2], int estado_proximo_discreto[2]);

// ====== TabelaQManager ======
TabelaQManager *manager_create();
void manager_free(TabelaQManager *m);
void manager_salvar_tabela_completa(TabelaQManager *m, RobotQLearningV2 *robot, const char *nome_base);
void manager_salvar_texto_scilab(TabelaQManager *m, RobotQLearningV2 *robot, const char *arquivo);
void manager_salvar_binario(TabelaQManager *m, RobotQLearningV2 *robot, const char *arquivo);

#endif
