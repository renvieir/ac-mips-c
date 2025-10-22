#include "robot_qlearning.h"

// ======== UTILS ========

static double rand_uniform(double min, double max) {
    return min + (max - min) * ((double) rand() / (double) RAND_MAX);
}

static double clamp(double val, double min, double max) {
    return val < min ? min : (val > max ? max : val);
}

// ======== QTable ========

static void qtable_init(QTable *q, int x, int y, int a) {
    q->x_size = x;
    q->y_size = y;
    q->a_size = a;
    q->data = (double*) calloc(x * y * a, sizeof(double));
}

static void qtable_free(QTable *q) {
    free(q->data);
    q->data = NULL;
}

static double* qtable_get(QTable *q, int i, int j, int a) {
    return &q->data[(i * q->y_size * q->a_size) + (j * q->a_size) + a];
}

static double qtable_max(QTable *q, int i, int j) {
    double maxv = -1e9;
    for (int a = 0; a < q->a_size; a++) {
        double v = *qtable_get(q, i, j, a);
        if (v > maxv) maxv = v;
    }
    return maxv;
}

// ======== RobotQLearningV2 ========

RobotQLearningV2 *robot_create(int caso) {
    RobotQLearningV2 *r = (RobotQLearningV2*) calloc(1, sizeof(RobotQLearningV2));

    r->caso = caso;
    r->d = 0.1;
    r->vmax = 0.5;
    r->rotmax = M_PI / 4.0;
    r->h = 0.05;

    r->alfa = 0.1;
    r->gama = 0.9;
    r->epsilon_inicial = 1.0;
    r->epsilon_min = 0.05;
    r->taxa_decaimento = 0.995;
    r->epsilon = r->epsilon_inicial;

    r->num_episodios = 500;
    r->passos_por_episodio = 200;
    r->dist_tolerancia = 0.1;

    // Discretização
    r->centro_x = 0;
    r->centro_y = 0;
    r->num_estados_x = 21;
    r->num_estados_y = 21;

    int nx = r->num_estados_x;
    int ny = r->num_estados_y;

    r->faixas_x = (double*) malloc(nx * sizeof(double));
    r->faixas_y = (double*) malloc(ny * sizeof(double));

    for (int i = 0; i < nx; i++) {
        r->faixas_x[i] = -r->d * (nx/2) + i * r->d;
        r->faixas_y[i] = -r->d * (ny/2) + i * r->d;
    }

    // Ações
    r->num_acoes_v = 3;
    r->num_acoes_rot = 3;
    r->num_acoes = r->num_acoes_v * r->num_acoes_rot;

    r->v_valores = (double*) malloc(r->num_acoes_v * sizeof(double));
    r->rot_valores = (double*) malloc(r->num_acoes_rot * sizeof(double));

    for (int i = 0; i < r->num_acoes_v; i++)
        r->v_valores[i] = -r->vmax + i * (2 * r->vmax / (r->num_acoes_v - 1));

    for (int j = 0; j < r->num_acoes_rot; j++)
        r->rot_valores[j] = -r->rotmax + j * (2 * r->rotmax / (r->num_acoes_rot - 1));

    robot_init_qtable(r);
    return r;
}

void robot_free(RobotQLearningV2 *r) {
    qtable_free(&r->Q_table);
    free(r->faixas_x);
    free(r->faixas_y);
    free(r->v_valores);
    free(r->rot_valores);
    free(r);
}

void robot_init_qtable(RobotQLearningV2 *r) {
    qtable_init(&r->Q_table, r->num_estados_x, r->num_estados_y, r->num_acoes);
}

// ====== CORE ======

void robot_calc_estados_discretos(RobotQLearningV2 *r, double erro_bruto[3],
                                  int estado_discreto[2], double erros_norm[2], double *dist_alvo) {
    erros_norm[0] = erro_bruto[0];
    erros_norm[1] = erro_bruto[1];
    *dist_alvo = sqrt(erros_norm[0]*erros_norm[0] + erros_norm[1]*erros_norm[1]);

    double dx = erro_bruto[0];
    double dy = erro_bruto[1];
    double idx = (dx + r->d * (r->num_estados_x/2)) / r->d;
    double idy = (dy + r->d * (r->num_estados_y/2)) / r->d;

    estado_discreto[0] = (int) floor(idx);
    estado_discreto[1] = (int) floor(idy);

    estado_discreto[0] = (int) clamp(estado_discreto[0], 0, r->num_estados_x-1);
    estado_discreto[1] = (int) clamp(estado_discreto[1], 0, r->num_estados_y-1);
}

int robot_escolher_acao(RobotQLearningV2 *r, int estado_discreto[2]) {
    if (rand_uniform(0, 1) < r->epsilon)
        return rand() % r->num_acoes;

    double maxv = -1e9;
    int best = 0;
    for (int a = 0; a < r->num_acoes; a++) {
        double v = *qtable_get(&r->Q_table, estado_discreto[0], estado_discreto[1], a);
        if (v > maxv) {
            maxv = v;
            best = a;
        }
    }
    return best;
}

void robot_mapear_acao_para_velocidades(RobotQLearningV2 *r, int acao,
                                        double erros_norm[2], double *v, double *rot) {
    int i_v = acao / r->num_acoes_rot;
    int i_rot = acao % r->num_acoes_rot;
    *v = r->v_valores[i_v];
    *rot = r->rot_valores[i_rot];
}

void robot_calcular_erro_inicial(RobotQLearningV2 *r, double destino[2], double pose[3], double Er[3]) {
    Er[0] = destino[0] - pose[0];
    Er[1] = destino[1] - pose[1];
    Er[2] = atan2(Er[1], Er[0]) - pose[2];
}

void robot_simular_acao_caso1(RobotQLearningV2 *r, double Er[3], double U[2], double Er_novo[3]) {
    double v = U[0];
    double w = U[1];
    double x = Er[0];
    double y = Er[1];
    double t = Er[2];

    double x_n = x - r->h * (v * cos(t) - w * y);
    double y_n = y - r->h * (v * sin(t) + w * x);
    double t_n = t - r->h * w;

    Er_novo[0] = x_n;
    Er_novo[1] = y_n;
    Er_novo[2] = t_n;
}

void robot_simular_acao_caso2(RobotQLearningV2 *r, double pose[3], double destino[2],
                              double U[2], double Er_novo[3], double pose_novo[3]) {
    double v = U[0];
    double w = U[1];
    double x = pose[0];
    double y = pose[1];
    double t = pose[2];

    double x_n = x + r->h * v * cos(t);
    double y_n = y + r->h * v * sin(t);
    double t_n = t + r->h * w;

    pose_novo[0] = x_n;
    pose_novo[1] = y_n;
    pose_novo[2] = t_n;

    Er_novo[0] = destino[0] - x_n;
    Er_novo[1] = destino[1] - y_n;
    Er_novo[2] = atan2(Er_novo[1], Er_novo[0]) - t_n;
}

double robot_calcular_recompensa(RobotQLearningV2 *r, int estado_discreto[2], int estado_proximo_discreto[2]) {
    double dx = fabs(estado_proximo_discreto[0] - r->centro_x);
    double dy = fabs(estado_proximo_discreto[1] - r->centro_y);
    double dist = sqrt(dx*dx + dy*dy);
    return -dist;
}

void robot_atualizar_tabela_q(RobotQLearningV2 *r, int estado_discreto[2], int acao,
                              double recompensa, int estado_proximo_discreto[2]) {
    double *q_antigo = qtable_get(&r->Q_table, estado_discreto[0], estado_discreto[1], acao);
    double max_futuro = qtable_max(&r->Q_table, estado_proximo_discreto[0], estado_proximo_discreto[1]);

    *q_antigo += r->alfa * (recompensa + r->gama * max_futuro - *q_antigo);
}

// ====== TRAIN ======

void robot_train(RobotQLearningV2 *r) {
    double destino[2] = {0, 0};

    for (int ep = 0; ep < r->num_episodios; ep++) {
        double pose[3] = {rand_uniform(-1,1), rand_uniform(-1,1), rand_uniform(-M_PI, M_PI)};
        double Er[3];
        robot_calcular_erro_inicial(r, destino, pose, Er);

        for (int passo = 0; passo < r->passos_por_episodio; passo++) {
            double erros_norm[2], dist;
            int estado_discreto[2];
            robot_calc_estados_discretos(r, Er, estado_discreto, erros_norm, &dist);

            int acao = robot_escolher_acao(r, estado_discreto);

            double v, rot;
            robot_mapear_acao_para_velocidades(r, acao, erros_norm, &v, &rot);

            double U[2] = {v, rot};
            double Er_novo[3];
            double pose_novo[3];

            if (r->caso == 1)
                robot_simular_acao_caso1(r, Er, U, Er_novo);
            else
                robot_simular_acao_caso2(r, pose, destino, U, Er_novo, pose_novo);

            double erros_norm_novo[2], dist_novo;
            int estado_proximo_discreto[2];
            robot_calc_estados_discretos(r, Er_novo, estado_proximo_discreto, erros_norm_novo, &dist_novo);

            double recompensa = robot_calcular_recompensa(r, estado_discreto, estado_proximo_discreto);
            robot_atualizar_tabela_q(r, estado_discreto, acao, recompensa, estado_proximo_discreto);

            memcpy(Er, Er_novo, 3*sizeof(double));
            memcpy(pose, pose_novo, 3*sizeof(double));

            if (dist_novo < r->dist_tolerancia) break;
        }

        r->epsilon = fmax(r->epsilon_min, r->epsilon * r->taxa_decaimento);
    }
}

// ====== TEST ======

void robot_test_policy(RobotQLearningV2 *r) {
    double destino[2] = {0, 0};
    double pose[3] = {1, 1, M_PI/4};
    double Er[3];
    robot_calcular_erro_inicial(r, destino, pose, Er);

    for (int passo = 0; passo < 200; passo++) {
        double erros_norm[2], dist;
        int estado_discreto[2];
        robot_calc_estados_discretos(r, Er, estado_discreto, erros_norm, &dist);

        int acao = robot_escolher_acao(r, estado_discreto);
        double v, rot;
        robot_mapear_acao_para_velocidades(r, acao, erros_norm, &v, &rot);

        double U[2] = {v, rot};
        double Er_novo[3], pose_novo[3];

        if (r->caso == 1)
            robot_simular_acao_caso1(r, Er, U, Er_novo);
        else
            robot_simular_acao_caso2(r, pose, destino, U, Er_novo, pose_novo);

        memcpy(Er, Er_novo, 3*sizeof(double));
        memcpy(pose, pose_novo, 3*sizeof(double));

        if (dist < r->dist_tolerancia) break;
    }
}

// ====== MANAGER ======

TabelaQManager *manager_create() {
    TabelaQManager *m = (TabelaQManager*) calloc(1, sizeof(TabelaQManager));
    m->num_formatos = 2;
    m->formatos_suportados = (char**) malloc(2 * sizeof(char*));
    m->formatos_suportados[0] = strdup("dat");
    m->formatos_suportados[1] = strdup("bin");
    return m;
}

void manager_free(TabelaQManager *m) {
    for (int i = 0; i < m->num_formatos; i++)
        free(m->formatos_suportados[i]);
    free(m->formatos_suportados);
    free(m);
}

void manager_salvar_binario(TabelaQManager *m, RobotQLearningV2 *r, const char *arquivo) {
    FILE *f = fopen(arquivo, "wb");
    if (!f) return;
    fwrite(&r->Q_table.x_size, sizeof(int), 1, f);
    fwrite(&r->Q_table.y_size, sizeof(int), 1, f);
    fwrite(&r->Q_table.a_size, sizeof(int), 1, f);
    fwrite(r->Q_table.data, sizeof(double), r->Q_table.x_size * r->Q_table.y_size * r->Q_table.a_size, f);
    fclose(f);
}

void manager_salvar_texto_scilab(TabelaQManager *m, RobotQLearningV2 *r, const char *arquivo) {
    FILE *f = fopen(arquivo, "w");
    if (!f) return;
    fprintf(f, "Q = zeros(%d,%d,%d);\n", r->Q_table.x_size, r->Q_table.y_size, r->Q_table.a_size);
    for (int i = 0; i < r->Q_table.x_size; i++)
        for (int j = 0; j < r->Q_table.y_size; j++)
            for (int a = 0; a < r->Q_table.a_size; a++)
                fprintf(f, "Q(%d,%d,%d) = %.6f;\n", i+1, j+1, a+1, *qtable_get(&r->Q_table, i,j,a));
    fclose(f);
}

void manager_salvar_tabela_completa(TabelaQManager *m, RobotQLearningV2 *r, const char *nome_base) {
    char txt_path[256];
    char bin_path[256];
    snprintf(txt_path, sizeof(txt_path), "%s.dat", nome_base);
    snprintf(bin_path, sizeof(bin_path), "%s.bin", nome_base);

    manager_salvar_texto_scilab(m, r, txt_path);
    manager_salvar_binario(m, r, bin_path);
}
