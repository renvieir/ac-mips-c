#include "robot.h"
#include <string.h>
#include <stdint.h>

// Fast random number generator state (XORShift)
static uint32_t xorshift_state = 0;

static inline uint32_t xorshift32() {
    xorshift_state ^= xorshift_state << 13;
    xorshift_state ^= xorshift_state >> 17;
    xorshift_state ^= xorshift_state << 5;
    return xorshift_state;
}

// Funções auxiliares de matemática
double rand_uniform(double min, double max) {
    // Usa XORShift para geração mais rápida de números aleatórios
    if (xorshift_state == 0) {
        xorshift_state = (uint32_t)time(NULL);
    }
    return min + (max - min) * (xorshift32() / (double)UINT32_MAX);
}

double fmod_positive(double x, double y) {
    double result = fmod(x, y);
    return result < 0 ? result + y : result;
}

// Aloca a tabela Q (array 3D)
void alocar_tabela_q(RobotQLearning* robot) {
    robot->Q_table = (double***)malloc(robot->num_estados_x * sizeof(double**));
    for (int i = 0; i < robot->num_estados_x; i++) {
        robot->Q_table[i] = (double**)malloc(robot->num_estados_y * sizeof(double*));
        for (int j = 0; j < robot->num_estados_y; j++) {
            robot->Q_table[i][j] = (double*)calloc(robot->num_acoes, sizeof(double));
        }
    }
}

// Libera a tabela Q
void liberar_tabela_q(RobotQLearning* robot) {
    if (robot->Q_table != NULL) {
        for (int i = 0; i < robot->num_estados_x; i++) {
            for (int j = 0; j < robot->num_estados_y; j++) {
                free(robot->Q_table[i][j]);
            }
            free(robot->Q_table[i]);
        }
        free(robot->Q_table);
        robot->Q_table = NULL;
    }
}

// Cria e inicializa o robô
RobotQLearning* criar_robot(int caso) {
    RobotQLearning* robot = (RobotQLearning*)malloc(sizeof(RobotQLearning));

    // Características do robô
    robot->d = 0.5;
    robot->vmax = 0.5;
    robot->rotmax = PI / 8.0;

    // Parâmetros da simulação
    robot->h = 0.01;
    robot->caso = caso;

    // Hiperparâmetros do Q-Learning
    robot->alfa = 0.1;
    robot->gama = 0.9;
    robot->epsilon_inicial = 1.0;
    robot->epsilon_min = 0.05;
    robot->taxa_decaimento = 0.999;
    robot->num_episodios = 5000;
    robot->passos_por_episodio = 200;
    robot->dist_tolerancia = 0.2;
    robot->epsilon = robot->epsilon_inicial;

    // Definição das faixas de discretização
    double faixas_temp_x[] = {-1, -0.5, -0.25, -0.125, -0.0625, 0,
                              0.0625, 0.125, 0.25, 0.5, 1};
    double faixas_temp_y[] = {-1, -0.5, -0.25, -0.125, -0.0625, 0,
                              0.0625, 0.125, 0.25, 0.5, 1};
    memcpy(robot->faixas_x, faixas_temp_x, sizeof(faixas_temp_x));
    memcpy(robot->faixas_y, faixas_temp_y, sizeof(faixas_temp_y));

    // Encontra os índices do centro (erro zero)
    robot->centro_x = 0;
    robot->centro_y = 0;
    for (int i = 0; i < NUM_ESTADOS_X; i++) {
        if (robot->faixas_x[i] == 0) robot->centro_x = i + 1; // 1-based
        if (robot->faixas_y[i] == 0) robot->centro_y = i + 1; // 1-based
    }

    // Conjunto de ações (velocidades discretas)
    double v_mult[] = {0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.4, 0.5, 0.75, 1.0};
    double rot_mult[] = {0, 0.05, 0.1, 0.2, 0.5, 1.0};

    for (int i = 0; i < NUM_ACOES_V; i++) {
        robot->v_valores[i] = v_mult[i] * robot->vmax;
    }
    for (int i = 0; i < NUM_ACOES_ROT; i++) {
        robot->rot_valores[i] = rot_mult[i] * robot->rotmax;
    }

    robot->num_acoes_v = NUM_ACOES_V;
    robot->num_acoes_rot = NUM_ACOES_ROT;
    robot->num_acoes = NUM_ACOES;

    // Tamanho da tabela Q
    robot->num_estados_x = NUM_ESTADOS_X;
    robot->num_estados_y = NUM_ESTADOS_Y;

    // Inicializa a tabela Q
    robot->Q_table = NULL;
    alocar_tabela_q(robot);

    return robot;
}

// Destroi o robô e libera memória
void destruir_robot(RobotQLearning* robot) {
    if (robot != NULL) {
        liberar_tabela_q(robot);
        free(robot);
    }
}

// Calcula os estados discretos a partir dos erros brutos
ResultadoEstadoDiscreto calc_estados_discretos(RobotQLearning* robot, Vector3D erro_bruto) {
    ResultadoEstadoDiscreto resultado;

    // Distância ao objetivo usando hipot (mais rápido e preciso)
    resultado.dist_alvo = hypot(erro_bruto.x, erro_bruto.y);

    // Evita divisão por zero
    if (resultado.dist_alvo < 0.01) {
        resultado.erros_norm.x = 0;
        resultado.erros_norm.y = 0;
    } else {
        double inv_dist = 1.0 / resultado.dist_alvo;  // Uma divisão em vez de duas
        resultado.erros_norm.x = erro_bruto.x * inv_dist;
        resultado.erros_norm.y = erro_bruto.y * inv_dist;
    }

    // Discretização otimizada: busca reversa (geralmente mais rápida para valores próximos do fim)
    int idx_x = NUM_ESTADOS_X;
    for (int i = NUM_ESTADOS_X - 1; i >= 0; i--) {
        if (resultado.erros_norm.x > robot->faixas_x[i]) {
            idx_x = i + 2;
            break;
        }
    }
    if (idx_x > NUM_ESTADOS_X) idx_x = 1;

    int idx_y = NUM_ESTADOS_Y;
    for (int i = NUM_ESTADOS_Y - 1; i >= 0; i--) {
        if (resultado.erros_norm.y > robot->faixas_y[i]) {
            idx_y = i + 2;
            break;
        }
    }
    if (idx_y > NUM_ESTADOS_Y) idx_y = 1;

    // Clamp aos limites
    idx_x = max_int(1, min_int(idx_x, robot->num_estados_x));
    idx_y = max_int(1, min_int(idx_y, robot->num_estados_y));

    resultado.estado.idx_x = idx_x;
    resultado.estado.idx_y = idx_y;

    return resultado;
}

// Escolhe uma ação usando estratégia epsilon-greedy
int escolher_acao(RobotQLearning* robot, EstadoDiscreto estado) {
    if (rand_uniform(0, 1) < robot->epsilon) {
        // Exploração: escolhe uma ação aleatória
        return (int)(rand_uniform(0, robot->num_acoes - 0.001)) + 1; // 1-based
    } else {
        // Explotação: escolhe a melhor ação da tabela Q
        int idx_x = estado.idx_x - 1; // Converte para 0-based
        int idx_y = estado.idx_y - 1;

        double max_q = robot->Q_table[idx_x][idx_y][0];
        int melhor_acao = 0;

        for (int a = 1; a < robot->num_acoes; a++) {
            if (robot->Q_table[idx_x][idx_y][a] > max_q) {
                max_q = robot->Q_table[idx_x][idx_y][a];
                melhor_acao = a;
            }
        }

        return melhor_acao + 1; // Retorna 1-based
    }
}

// Mapeia a ação escolhida para as velocidades (v e rot)
void mapear_acao_para_velocidades(RobotQLearning* robot, int acao, Vector2D erros_norm,
                                   double* v, double* rot) {
    int acao_idx = acao - 1; // Converte para 0-based
    int idx_v = acao_idx / robot->num_acoes_rot;
    int idx_rot = acao_idx % robot->num_acoes_rot;

    *v = robot->v_valores[idx_v] * erros_norm.x;
    *rot = robot->rot_valores[idx_rot] * erros_norm.y;
}

// Calcula o erro inicial no frame do robô
Vector3D calcular_erro_inicial(RobotQLearning* robot, Vector2D destino, Vector3D pose) {
    // Erro de posicionamento no sistema fixo
    double Ex = destino.x - pose.x;
    double Ey = destino.y - pose.y;

    // Matriz de rotação inversa
    double cs = cos(pose.theta);
    double ss = sin(pose.theta);

    // Erro no frame do robô
    Vector3D Er;
    Er.x = cs * Ex + ss * Ey - robot->d;
    Er.y = -ss * Ex + cs * Ey;
    Er.theta = pose.theta;

    return Er;
}

// Caso 1: Calcula a velocidade do erro no frame do robô
Vector3D simular_acao_caso1(RobotQLearning* robot, Vector3D Er, Vector2D U) {
    Vector3D Er_novo = Er;

    // Pré-calcula valores constantes no loop
    const double h_Ux = robot->h * (-U.x);
    const double h_Uy = robot->h * U.y;
    const double h_Uy_d = h_Uy * robot->d;
    const double two_pi = 2.0 * PI;

    // Integração numérica otimizada
    for (int i = 0; i < 100; i++) {
        // Modelo cinemático (equação 39 do PDF)
        double Er_pt_x = h_Ux + Er_novo.y * h_Uy;
        double Er_pt_y = -Er_novo.x * h_Uy - h_Uy_d;

        Er_novo.x += Er_pt_x;
        Er_novo.y += Er_pt_y;
        Er_novo.theta += h_Uy;

        // Normaliza theta apenas no final para evitar fmod repetido
        if (i == 99) {
            Er_novo.theta = fmod_positive(Er_novo.theta, two_pi);
        }
    }

    return Er_novo;
}

// Caso 2: Atualiza a pose do robô e recalcula os erros
void simular_acao_caso2(RobotQLearning* robot, Vector3D* pose, Vector2D destino,
                        Vector2D U, Vector3D* Er_novo) {
    Vector3D pose_novo = *pose;

    // Pré-calcula valores constantes
    const double h_Ux = robot->h * U.x;
    const double h_Uy = robot->h * U.y;
    const double two_pi = 2.0 * PI;

    // Integração da pose do robô - otimizada
    for (int i = 0; i < 100; i++) {
        double cos_theta = cos(pose_novo.theta);
        double sin_theta = sin(pose_novo.theta);

        pose_novo.x += h_Ux * cos_theta;
        pose_novo.y += h_Ux * sin_theta;
        pose_novo.theta += h_Uy;

        // Normaliza theta apenas no final
        if (i == 99) {
            pose_novo.theta = fmod_positive(pose_novo.theta, two_pi);
        }
    }

    // Atualiza a pose
    *pose = pose_novo;

    // Recalcula o erro no novo estado
    *Er_novo = calcular_erro_inicial(robot, destino, pose_novo);
}

// Calcula a recompensa baseada no estado atual e próximo
double calcular_recompensa(RobotQLearning* robot, EstadoDiscreto estado,
                          EstadoDiscreto estado_proximo) {
    // Penalidade de tempo
    double recompensa = -1.0;

    // Grande recompensa por chegar ao centro (alvo)
    if (estado.idx_x == robot->centro_x && estado.idx_y == robot->centro_y) {
        recompensa = 100.0;
    }

    // Penalidade por se afastar do alvo
    if ((estado_proximo.idx_x + estado_proximo.idx_y) > (estado.idx_x + estado.idx_y)) {
        recompensa = -5.0;
    }

    return recompensa;
}

// Atualiza a tabela Q usando a equação de Bellman
void atualizar_tabela_q(RobotQLearning* robot, EstadoDiscreto estado, int acao,
                       double recompensa, EstadoDiscreto estado_proximo) {
    // Índices (ajustados para 0-based)
    int idx_x = estado.idx_x - 1;
    int idx_y = estado.idx_y - 1;
    int idx_x_prox = estado_proximo.idx_x - 1;
    int idx_y_prox = estado_proximo.idx_y - 1;
    int acao_idx = acao - 1;

    // Usa ponteiros para evitar indexação repetida
    double* Q_atual = robot->Q_table[idx_x][idx_y];
    double* Q_proximo = robot->Q_table[idx_x_prox][idx_y_prox];

    double Q_s_a = Q_atual[acao_idx];

    // Encontra o Q máximo do próximo estado (otimizado)
    double Q_max_proximo = Q_proximo[0];
    for (int a = 1; a < robot->num_acoes; a++) {
        if (Q_proximo[a] > Q_max_proximo) {
            Q_max_proximo = Q_proximo[a];
        }
    }

    // Atualiza a tabela Q
    Q_atual[acao_idx] = Q_s_a + robot->alfa * (recompensa + robot->gama * Q_max_proximo - Q_s_a);
}

// Executa o treinamento do Q-Learning
HistoricoItem* treinar(RobotQLearning* robot, int* tamanho_historico) {
    HistoricoItem* historico = (HistoricoItem*)malloc(robot->num_episodios * sizeof(HistoricoItem));
    *tamanho_historico = robot->num_episodios;

    for (int i = 0; i < robot->num_episodios; i++) {
        // Preparação do episódio
        Vector2D destino;
        destino.x = rand_uniform(-10, 10);
        destino.y = rand_uniform(-10, 10);

        Vector3D pose = {0, 0, 0};

        // Calcula o erro inicial
        Vector3D Er = calcular_erro_inicial(robot, destino, pose);

        // Calcula o estado inicial
        ResultadoEstadoDiscreto resultado = calc_estados_discretos(robot, Er);
        EstadoDiscreto estado_discreto = resultado.estado;
        Vector2D erros_norm = resultado.erros_norm;

        int j;
        for (j = 0; j < robot->passos_por_episodio; j++) {
            // Escolhe a ação
            int acao = escolher_acao(robot, estado_discreto);

            // Mapeia a ação para velocidades
            double v, rot;
            mapear_acao_para_velocidades(robot, acao, erros_norm, &v, &rot);
            Vector2D U = {v, rot};

            // Aplica a ação (simulação)
            Vector3D Er_novo;
            if (robot->caso == 1) {
                Er_novo = simular_acao_caso1(robot, Er, U);
            } else {
                simular_acao_caso2(robot, &pose, destino, U, &Er_novo);
            }

            // Calcula o próximo estado
            resultado = calc_estados_discretos(robot, Er_novo);
            EstadoDiscreto estado_proximo_discreto = resultado.estado;
            erros_norm = resultado.erros_norm;

            // Calcula a recompensa
            double recompensa = calcular_recompensa(robot, estado_discreto, estado_proximo_discreto);

            // Atualiza a tabela Q
            atualizar_tabela_q(robot, estado_discreto, acao, recompensa, estado_proximo_discreto);

            // Atualiza o estado
            Er = Er_novo;
            estado_discreto = estado_proximo_discreto;

            // Verifica se o episódio terminou
            if (recompensa > 0 || j == robot->passos_por_episodio - 1) {
                break;
            }
        }

        // Registra o progresso
        historico[i].episodio = i + 1;
        historico[i].passos = j + 1;

        // Decaimento do epsilon
        robot->epsilon = max_double(robot->epsilon_min, robot->epsilon * robot->taxa_decaimento);
    }

    return historico;
}

// Salva a tabela Q em formato texto compatível com Scilab
void salvar_tabela_q_texto(RobotQLearning* robot, const char* nome_arquivo) {
    FILE* f = fopen(nome_arquivo, "w");
    if (f == NULL) {
        printf("Erro ao abrir arquivo %s\n", nome_arquivo);
        return;
    }

    // Cabeçalho informativo
    fprintf(f, "// Tabela Q treinada - Formato compatível com Scilab\n");
    fprintf(f, "// Gerado automaticamente pelo C\n");
    fprintf(f, "// Dimensões: %d x %d x %d\n",
            robot->num_estados_x, robot->num_estados_y, robot->num_acoes);
    fprintf(f, "//\n");

    // Parâmetros importantes
    fprintf(f, "// PARÂMETROS DO TREINAMENTO:\n");
    fprintf(f, "// d = %.2f\n", robot->d);
    fprintf(f, "// vmax = %.2f\n", robot->vmax);
    fprintf(f, "// rotmax = %.6f\n", robot->rotmax);
    fprintf(f, "// h = %.4f\n", robot->h);
    fprintf(f, "// caso = %d\n", robot->caso);
    fprintf(f, "// alfa = %.2f\n", robot->alfa);
    fprintf(f, "// gama = %.2f\n", robot->gama);
    fprintf(f, "// epsilon_final = %.4f\n", robot->epsilon);
    fprintf(f, "// num_episodios = %d\n", robot->num_episodios);
    fprintf(f, "// passos_por_episodio = %d\n", robot->passos_por_episodio);
    fprintf(f, "// dist_tolerancia = %.2f\n", robot->dist_tolerancia);
    fprintf(f, "//\n");

    // Discretização
    fprintf(f, "// DISCRETIZAÇÃO:\n");
    fprintf(f, "// faixas_x = [");
    for (int i = 0; i < robot->num_estados_x; i++) {
        fprintf(f, "%.4f", robot->faixas_x[i]);
        if (i < robot->num_estados_x - 1) fprintf(f, ", ");
    }
    fprintf(f, "]\n");

    fprintf(f, "// faixas_y = [");
    for (int i = 0; i < robot->num_estados_y; i++) {
        fprintf(f, "%.4f", robot->faixas_y[i]);
        if (i < robot->num_estados_y - 1) fprintf(f, ", ");
    }
    fprintf(f, "]\n");
    fprintf(f, "//\n");

    // Início dos dados da tabela Q
    fprintf(f, "// TABELA Q (formato: Q_table(x, y, acao) = valor):\n");
    fprintf(f, "Q_table = zeros(%d, %d, %d);\n\n",
            robot->num_estados_x, robot->num_estados_y, robot->num_acoes);

    // Salva apenas valores não-zero para eficiência
    int valores_nao_zero = 0;
    int total_valores = robot->num_estados_x * robot->num_estados_y * robot->num_acoes;

    for (int x = 0; x < robot->num_estados_x; x++) {
        for (int y = 0; y < robot->num_estados_y; y++) {
            for (int a = 0; a < robot->num_acoes; a++) {
                double valor = robot->Q_table[x][y][a];
                if (fabs(valor) > 1e-10) {
                    fprintf(f, "Q_table(%d, %d, %d) = %.8f;\n", x + 1, y + 1, a + 1, valor);
                    valores_nao_zero++;
                }
            }
        }
    }

    fprintf(f, "\n// Total de valores não-zero: %d\n", valores_nao_zero);
    fprintf(f, "// Densidade da tabela: %.2f%%\n",
            (valores_nao_zero * 100.0) / total_valores);

    fclose(f);
}

// Testa a política aprendida sem exploração
ResultadoTeste testar_politica(RobotQLearning* robot, int max_passos) {
    ResultadoTeste resultado;
    resultado.trajetoria = (EstadoDiscreto*)malloc(max_passos * sizeof(EstadoDiscreto));
    resultado.tamanho = 0;
    resultado.sucesso = 0;

    // Configuração inicial
    Vector2D destino = {4.0, 3.0};
    Vector3D pose = {0.0, 0.0, 0.0};
    Vector3D Er = calcular_erro_inicial(robot, destino, pose);

    double epsilon_original = robot->epsilon;
    robot->epsilon = 0.0; // Sem exploração

    for (int passo = 0; passo < max_passos; passo++) {
        ResultadoEstadoDiscreto res = calc_estados_discretos(robot, Er);
        EstadoDiscreto estado_discreto = res.estado;
        Vector2D erros_norm = res.erros_norm;

        resultado.trajetoria[passo] = estado_discreto;
        resultado.tamanho++;

        // Verifica se chegou ao objetivo
        if (estado_discreto.idx_x == robot->centro_x &&
            estado_discreto.idx_y == robot->centro_y) {
            resultado.sucesso = 1;
            robot->epsilon = epsilon_original;
            return resultado;
        }

        // Escolhe ação e executa
        int acao = escolher_acao(robot, estado_discreto);
        double v, rot;
        mapear_acao_para_velocidades(robot, acao, erros_norm, &v, &rot);
        Vector2D U = {v, rot};

        if (robot->caso == 1) {
            Er = simular_acao_caso1(robot, Er, U);
        } else {
            simular_acao_caso2(robot, &pose, destino, U, &Er);
        }
    }

    robot->epsilon = epsilon_original;
    return resultado;
}

// Libera a memória do resultado de teste
void liberar_resultado_teste(ResultadoTeste* resultado) {
    if (resultado->trajetoria != NULL) {
        free(resultado->trajetoria);
        resultado->trajetoria = NULL;
    }
}
