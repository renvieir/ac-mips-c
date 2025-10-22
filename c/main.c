#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "robot_qlearning.h"

int main(void) {
    srand((unsigned int) time(NULL));

    printf("=== Iniciando treinamento do RobotQLearningV2 ===\n");

    int caso = 2;  // mesmo comportamento do Python: define o tipo de simulação
    RobotQLearningV2 *robot = robot_create(caso);
    TabelaQManager *manager = manager_create();

    printf("Treinando agente...\n");
    robot_train(robot);
    printf("Treinamento concluído.\n");

    printf("Salvando tabelas Q...\n");
    manager_salvar_tabela_completa(manager, robot, "qtable_resultados");
    printf("Arquivos 'qtable_resultados.dat' e 'qtable_resultados.bin' gerados.\n");

    printf("Testando política aprendida...\n");
    robot_test_policy(robot);
    printf("Teste concluído.\n");

    manager_free(manager);
    robot_free(robot);

    printf("=== Fim da execução ===\n");
    return 0;
}
