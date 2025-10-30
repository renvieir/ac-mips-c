# ==============================================================================
# Makefile para Projeto C (Q-Learning Robot)
# Foco: Estrutura Modular, Boas Práticas e Segurança em C.
# ==============================================================================

# Nome do Executável
#TARGET = robot_qlearning
TARGET = robot

# Fontes e Objetos
SRCS = main.c robot.c
OBJS = $(SRCS:.c=.o)

# Compilador
CC = gcc

# Flags de Compilação
# -Wall -Wextra -Werror: Segurança e qualidade
# -std=c99: compatível com o código
# -O3: otimização agressiva
# -march=native: otimiza para a CPU local
# -ffast-math: otimizações matemáticas agressivas
# -funroll-loops: desenrola loops pequenos
# -flto: Link-Time Optimization
CFLAGS = -Wall -Wextra -Werror -std=c99 -O3 -march=native -ffast-math -funroll-loops -flto

# Flags de Compilação para Debug (use 'make DEBUG=1' para ativar)
ifdef DEBUG
CFLAGS = -Wall -Wextra -Werror -std=c99 -O0 -g
endif

# Flags de Linkagem
LDFLAGS = -lm

# ------------------------------------------------------------------------------
# Regra Padrão
# ------------------------------------------------------------------------------
all: $(TARGET)

# Linkagem Final
$(TARGET): $(OBJS)
	@echo "Linking $(TARGET)"
	$(CC) $(CFLAGS) $(OBJS) -o $(TARGET) $(LDFLAGS)

# ------------------------------------------------------------------------------
# Compilação Genérica (.c → .o)
# ------------------------------------------------------------------------------
%.o: %.c
	@echo "Compiling $<"
	$(CC) -c $< -o $@ $(CFLAGS)

# ------------------------------------------------------------------------------
# Utilitários
# ------------------------------------------------------------------------------
clean:
	@echo "Cleaning build artifacts..."
	rm -f $(OBJS) $(TARGET) *.dat *.bin

run: $(TARGET)
	@echo "Running $(TARGET)..."
	./$(TARGET)

help:
	@echo "Targets disponíveis:"
	@echo "  make          - Compila com otimizações máximas (-O3, -march=native, -flto)"
	@echo "  make DEBUG=1  - Compila em modo debug (-O0, -g)"
	@echo "  make run      - Compila e executa"
	@echo "  make clean    - Remove arquivos gerados"

# ------------------------------------------------------------------------------
# Dependências Explícitas
# ------------------------------------------------------------------------------
robot.o: robot.h
main.o: robot.h

# ------------------------------------------------------------------------------
# Pseudo-targets
# ------------------------------------------------------------------------------
.PHONY: all clean run help
