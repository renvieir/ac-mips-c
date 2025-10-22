# ==============================================================================
# Makefile para Projeto C (Q-Learning Robot)
# Foco: Estrutura Modular, Boas Práticas e Segurança em C.
# ==============================================================================

# Nome do Executável
TARGET = robot_qlearning

# Fontes e Objetos
SRCS = main.c robot_qlearning.c
OBJS = $(SRCS:.c=.o)

# Compilador
CC = gcc

# Flags de Compilação
# -Wall -Wextra -Werror: Segurança e qualidade
# -std=c99: compatível com o código
# -O2: otimização moderada
# -g: informações de debug
CFLAGS = -Wall -Wextra -Werror -std=c99 -O2 -g

# Flags de Linkagem
LDFLAGS = -lm

# ------------------------------------------------------------------------------
# Regra Padrão
# ------------------------------------------------------------------------------
all: $(TARGET)

# Linkagem Final
$(TARGET): $(OBJS)
	@echo "Linking $(TARGET)"
	$(CC) $(OBJS) -o $(TARGET) $(LDFLAGS)

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

# ------------------------------------------------------------------------------
# Dependências Explícitas
# ------------------------------------------------------------------------------
robot_qlearning.o: robot_qlearning.h
main.o: robot_qlearning.h

# ------------------------------------------------------------------------------
# Pseudo-targets
# ------------------------------------------------------------------------------
.PHONY: all clean run
