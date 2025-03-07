from collections import deque

class Buffer:
    def __init__(self, tamanho_maximo):
        """
        Inicializa o FrameBuffer com um tamanho máximo especificado.
        - `tamanho_maximo`: Limite máximo de itens que podem ser armazenados na fila.
        - `fila`: Um deque que armazena os itens no buffer.
        """
        self.tamanho_maximo = tamanho_maximo
        self.fila = deque()

    def adicionar(self, item):
        """
        Adiciona um item ao final do buffer.
        - Se o buffer já estiver cheio (atingir o tamanho máximo), remove o item mais antigo (no início da fila).
        - `item`: O elemento a ser adicionado ao buffer.
        """
        if len(self.fila) >= self.tamanho_maximo:
            self.fila.popleft()  # Remove o item mais antigo para manter o tamanho máximo
        self.fila.append(item)

    def remover(self):
        """
        Remove e retorna o item mais antigo (no início da fila).
        - Levanta um IndexError se o buffer estiver vazio.
        - Retorna: O item removido.
        """
        if not self.fila:
            raise IndexError("Remoção de elemento de uma fila vazia.")
        return self.fila.popleft()

    def tamanho(self):
        """
        Retorna o número atual de itens no buffer.
        - Retorna: Um inteiro representando o tamanho do buffer.
        """
        return len(self.fila)
    
    def __getitem__(self, index):
        """
        Permite acesso direto a itens do buffer por índice.
        - Suporta índices negativos (ex.: -1 para o último item).
        - Verifica se o índice está dentro do intervalo válido, levantando IndexError caso contrário.
        - `index`: O índice do item a ser acessado.
        - Retorna: O item no índice especificado.
        """
        if index < -len(self.fila):
            index = 0
        if index >= len(self.fila):
            raise IndexError("Índice fora do intervalo.")
        elif index < 0:
            index += len(self.fila)  # Ajusta o índice negativo para acessar os itens de forma decrescente
        return self.fila[index]

    def __repr__(self):
        """
        Retorna uma representação string do buffer.
        - Mostra os itens contidos no deque no formato de lista.
        - Retorna: String representando o buffer.
        """
        return repr(self.fila)
