# Robot Q-Learning - Versão C

Este projeto implementa um algoritmo de Q-Learning para controle de robô em C, convertido da versão Python original.

## Estrutura dos Arquivos

- `robot.h` - Definições de estruturas e declarações de funções
- `robot.c` - Implementação das funções do robô e do Q-Learning
- `main.c` - Programa principal que executa o treinamento
- `Makefile` - Script de compilação

## Características Principais

### Estruturas de Dados

- **RobotQLearning**: Estrutura principal contendo todos os parâmetros do robô e a tabela Q
- **Vector2D/Vector3D**: Vetores para representar posições e poses
- **EstadoDiscreto**: Estados discretizados para a tabela Q
- **HistoricoItem**: Registro do histórico de treinamento

### Funcionalidades

1. **Treinamento com Q-Learning**

   - 5000 episódios por padrão
   - Estratégia epsilon-greedy
   - Decaimento de epsilon ao longo do tempo
   - Dois casos de simulação (atualização direta de erro ou via pose)

2. **Discretização de Estados**

   - Espaço de estados discretizado em 11x11 células
   - Normalização de erros de posição
   - 66 ações possíveis (11 velocidades × 6 velocidades de rotação)

3. **Salvamento de Resultados**

   - Exportação da tabela Q para formato texto compatível com Scilab
   - Apenas valores não-zero são salvos para eficiência

4. **Teste de Política**
   - Teste da política aprendida sem exploração
   - Trajetória completa registrada

## Como Compilar e Executar

### No Windows (PowerShell)

```powershell
# Compilar
make

# Compilar e executar
make run

# Limpar arquivos gerados
make clean
```

### No Linux/Unix

```bash
# Compilar
make

# Compilar e executar
make run

# Limpar arquivos gerados
make clean
```

## Saída do Programa

O programa exibe:

1. **Informações de Inicialização**

   - Parâmetros do robô
   - Configuração da tabela Q

2. **Progresso do Treinamento**

   - Atualização a cada 500 episódios
   - Número de passos e valor de epsilon

3. **Estatísticas Finais**

   - Média, mínimo e máximo de passos
   - Últimos 10 episódios
   - Análise da tabela Q

4. **Teste da Política**

   - Sucesso/falha
   - Trajetória percorrida

5. **Arquivo Gerado**
   - `meu_robo_treinado.dat` - Tabela Q em formato Scilab

## Parâmetros Configuráveis

No código `robot.c`, função `criar_robot()`:

- `d = 0.5` - Distância à frente do robô (metros)
- `vmax = 0.5` - Velocidade máxima de translação (m/s)
- `rotmax = π/8` - Velocidade máxima de rotação (rad/s)
- `alfa = 0.1` - Taxa de aprendizado
- `gama = 0.9` - Fator de desconto
- `epsilon_inicial = 1.0` - Exploração inicial
- `epsilon_min = 0.05` - Exploração mínima
- `num_episodios = 5000` - Número de episódios de treinamento
- `passos_por_episodio = 200` - Máximo de passos por episódio

## Diferenças em Relação à Versão Python

1. **Gestão de Memória**: Alocação e liberação manual de memória para a tabela Q
2. **Arrays**: Uso de arrays de tamanho fixo e alocação dinâmica
3. **Sem Matplotlib**: A visualização gráfica foi removida (apenas console)
4. **Salvamento**: Apenas formato texto .dat (sem .npz)
5. **Performance**: Versão C pode ser significativamente mais rápida

## Requisitos

- GCC (ou outro compilador C compatível)
- Make
- Biblioteca matemática padrão (libm)

## Notas

- O gerador de números aleatórios é inicializado com o tempo atual
- A tabela Q é uma matriz 3D alocada dinamicamente
- Os índices são 1-based (como no Python original) mas convertidos para 0-based internamente
- Todos os recursos são liberados adequadamente no final da execução

## Exemplo de Uso

```c
// Criar robô (caso 1)
RobotQLearning* robot = criar_robot(1);

// Treinar
int tamanho_historico;
HistoricoItem* historico = treinar(robot, &tamanho_historico);

// Salvar tabela Q
salvar_tabela_q_texto(robot, "meu_robo_treinado.dat");

// Testar política
ResultadoTeste resultado = testar_politica(robot, 100);

// Limpar memória
liberar_resultado_teste(&resultado);
free(historico);
destruir_robot(robot);
```

## Licença

Este projeto é baseado no código Python original e segue a mesma licença.
