#include "robot.h"

int main() {
    // Inicializa o gerador de números aleatórios
    srand((unsigned int)time(NULL));

    // Marca o tempo de início
    clock_t inicio = clock();

    // Cria e inicializa o robô (Caso 1)
    RobotQLearning* robot = criar_robot(1);

    // Treina o robô
    int tamanho_historico;
    HistoricoItem* historico = treinar(robot, &tamanho_historico);

    // Salva a tabela Q
    salvar_tabela_q_texto(robot, "meu_robo_treinado.dat");

    // Testa a política aprendida
    ResultadoTeste resultado_teste = testar_politica(robot, 100);

    // Limpeza
    liberar_resultado_teste(&resultado_teste);
    free(historico);
    destruir_robot(robot);

    // Marca o tempo de fim
    clock_t fim = clock();

    // Calcula o tempo de execução
    double tempo_execucao = ((double)(fim - inicio)) / CLOCKS_PER_SEC;

    // Exibe apenas o tempo de execução
    printf("Tempo de execução: %.3f segundos\n", tempo_execucao);

    return 0;
}
