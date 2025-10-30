#include "robot.h"

int main() {
    // Inicializa o gerador de números aleatórios
    srand((unsigned int)time(NULL));

    printf("=================================================\n");
    printf("   Treinamento de Robô com Q-Learning em C\n");
    printf("=================================================\n\n");

    // Cria e inicializa o robô (Caso 1)
    printf("Criando robô com caso = 1...\n");
    RobotQLearning* robot = criar_robot(1);

    printf("Robô criado com sucesso!\n");
    printf("Parâmetros:\n");
    printf("  - d: %.2f m\n", robot->d);
    printf("  - vmax: %.2f m/s\n", robot->vmax);
    printf("  - rotmax: %.4f rad/s\n", robot->rotmax);
    printf("  - Alfa (taxa de aprendizado): %.2f\n", robot->alfa);
    printf("  - Gama (fator de desconto): %.2f\n", robot->gama);
    printf("  - Epsilon inicial: %.2f\n", robot->epsilon_inicial);
    printf("  - Número de episódios: %d\n", robot->num_episodios);
    printf("  - Passos por episódio: %d\n", robot->passos_por_episodio);
    printf("  - Dimensões da tabela Q: %dx%dx%d\n\n",
           robot->num_estados_x, robot->num_estados_y, robot->num_acoes);

    // Treina o robô
    printf("Iniciando treinamento...\n");
    printf("=================================================\n");

    int tamanho_historico;
    HistoricoItem* historico = treinar(robot, &tamanho_historico);

    printf("=================================================\n");
    printf("Treinamento concluído!\n\n");

    // Estatísticas do treinamento
    int soma_passos = 0;
    int min_passos = historico[0].passos;
    int max_passos = historico[0].passos;

    for (int i = 0; i < tamanho_historico; i++) {
        soma_passos += historico[i].passos;
        if (historico[i].passos < min_passos) min_passos = historico[i].passos;
        if (historico[i].passos > max_passos) max_passos = historico[i].passos;
    }

    double media_passos = (double)soma_passos / tamanho_historico;

    printf("Estatísticas do treinamento:\n");
    printf("  - Média de passos: %.2f\n", media_passos);
    printf("  - Mínimo de passos: %d\n", min_passos);
    printf("  - Máximo de passos: %d\n", max_passos);
    printf("  - Epsilon final: %.4f\n\n", robot->epsilon);

    // Mostra os últimos 10 episódios
    printf("Últimos 10 episódios:\n");
    for (int i = max_int(0, tamanho_historico - 10); i < tamanho_historico; i++) {
        printf("  Episódio %d: %d passos\n", historico[i].episodio, historico[i].passos);
    }
    printf("\n");

    // Salva a tabela Q
    printf("Salvando tabela Q...\n");
    salvar_tabela_q_texto(robot, "meu_robo_treinado.dat");
    printf("\n");

    // Testa a política aprendida
    printf("Testando política aprendida...\n");
    ResultadoTeste resultado_teste = testar_politica(robot, 100);

    printf("Resultado do teste:\n");
    printf("  - Sucesso: %s\n", resultado_teste.sucesso ? "Sim" : "Não");
    printf("  - Passos utilizados: %d\n", resultado_teste.tamanho);

    if (resultado_teste.tamanho > 0) {
        printf("  - Trajetória (primeiros 20 estados):\n");
        int max_mostrar = min_int(resultado_teste.tamanho, 20);
        for (int i = 0; i < max_mostrar; i++) {
            printf("    Passo %d: Estado (%d, %d)\n",
                   i + 1,
                   resultado_teste.trajetoria[i].idx_x,
                   resultado_teste.trajetoria[i].idx_y);
        }
        if (resultado_teste.tamanho > 20) {
            printf("    ... (%d estados restantes)\n", resultado_teste.tamanho - 20);
        }
    }
    printf("\n");

    // Informações sobre a tabela Q
    printf("Analisando tabela Q...\n");
    int valores_nao_zero = 0;
    double soma_valores = 0.0;
    double max_q = robot->Q_table[0][0][0];
    double min_q = robot->Q_table[0][0][0];

    for (int x = 0; x < robot->num_estados_x; x++) {
        for (int y = 0; y < robot->num_estados_y; y++) {
            for (int a = 0; a < robot->num_acoes; a++) {
                double valor = robot->Q_table[x][y][a];
                if (fabs(valor) > 1e-10) {
                    valores_nao_zero++;
                    soma_valores += valor;
                }
                if (valor > max_q) max_q = valor;
                if (valor < min_q) min_q = valor;
            }
        }
    }

    int total_valores = robot->num_estados_x * robot->num_estados_y * robot->num_acoes;
    double densidade = (valores_nao_zero * 100.0) / total_valores;
    double media_valores = valores_nao_zero > 0 ? soma_valores / valores_nao_zero : 0.0;

    printf("Estatísticas da tabela Q:\n");
    printf("  - Total de valores: %d\n", total_valores);
    printf("  - Valores não-zero: %d\n", valores_nao_zero);
    printf("  - Densidade: %.2f%%\n", densidade);
    printf("  - Valor Q máximo: %.4f\n", max_q);
    printf("  - Valor Q mínimo: %.4f\n", min_q);
    printf("  - Média dos valores não-zero: %.4f\n", media_valores);
    printf("\n");

    // Verifica o estado objetivo
    int centro_x_idx = robot->centro_x - 1;
    int centro_y_idx = robot->centro_y - 1;
    printf("Estado objetivo (centro): (%d, %d)\n", robot->centro_x, robot->centro_y);
    printf("Valores Q no estado objetivo:\n");
    for (int a = 0; a < min_int(10, robot->num_acoes); a++) {
        printf("  Ação %d: %.4f\n", a + 1, robot->Q_table[centro_x_idx][centro_y_idx][a]);
    }
    if (robot->num_acoes > 10) {
        printf("  ... (%d ações restantes)\n", robot->num_acoes - 10);
    }
    printf("\n");

    // Limpeza
    printf("Liberando memória...\n");
    liberar_resultado_teste(&resultado_teste);
    free(historico);
    destruir_robot(robot);

    printf("=================================================\n");
    printf("Programa concluído com sucesso!\n");
    printf("=================================================\n");

    return 0;
}
