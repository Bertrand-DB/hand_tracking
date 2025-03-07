import math
import time
from Buffer import Buffer

F_BASE = 0
F_ULTIMO = -1
F_PENULTIMO = -2
X = 0
Y = 1
Z = 2


class Gestures:
    def __init__(self, right_hand, cam_resolution, cam_fov_diagonal, palm_cross_size, buffer_size=5):
        """
        Inicializa a classe `Gestures` que gerencia o rastreamento de gestos e movimentos 
        da mão, fornecendo informações necessárias para o cálculo de angulações e distâncias.

        Este construtor configura os parâmetros iniciais relacionados à câmera, mão rastreada, 
        e tamanho da palma, além de inicializar buffers para armazenar dados de frames e tempos.

        Parâmetros:
            right_hand (bool): Define qual mão será rastreada (True para mão direita, False para mão esquerda).
            cam_resolution (tuple): Resolução da câmera em pixels como (largura, altura).
            cam_fov_diagonal (float): Campo de visão diagonal da câmera em graus.
            palm_cross_size (tuple): Dimensões reais da palma da mão em centímetros, no formato 
                                     (largura entre os pontos 5-17, altura entre os pontos 0-13).
            buffer_size (int, opcional): Tamanho do buffer para armazenar dados de frames. 
                                         O padrão é 5.
        """

        # Dimensões da câmera em pixels
        self.cam_width, self.cam_height = cam_resolution  # Ex.: (1920, 1080)
        
        # Campo de visão diagonal da câmera, usado para cálculos de profundidade
        self.cam_fov_diagonal = cam_fov_diagonal  # Ex.: 60.0 graus
        
        # Dimensões reais da palma da mão em centímetros
        # Usadas como referência para estimar distâncias e escalas
        self.palm_width, self.palm_heigth = palm_cross_size  # Ex.: (8.0 cm, 12.0 cm)

        # Tamanho do buffer para armazenar dados de frames e tempos (número de quadros recentes a manter)
        self.BUFFER_SIZE = buffer_size
        
        # Buffers circulares para armazenar dados de quadros (frames) e tempos
        # `frame_buffer` armazena coordenadas dos marcos da mão
        # `time_buffer` armazena os tempos correspondentes a cada quadro
        self.frame_buffer = Buffer(buffer_size)
        self.time_buffer = Buffer(buffer_size)
        
        # Indica se a mão rastreada é a direita (True) ou a esquerda (False)
        self.right_hand = right_hand  # Ex.: True para mão direita

    def frame_update(self, hand_points_coord):
        self.frame_buffer.adicionar(hand_points_coord)
        self.time_buffer.adicionar(time.time())

    def estimate_depth(self, frame=F_ULTIMO):
        """
        Calcula a distância do plano do objeto à câmera com base no FOV diagonal, 
        resolução da câmera, distância entre dois pontos na imagem e a distância real entre esses pontos.
            
        Retorno:
            float: Distância do plano do objeto à câmera (mesma unidade de `distancia_real`).
        """
        v_5_17 = self.p_to_vec(self.frame_buffer[frame][5], self.frame_buffer[frame][17])
        dist_horizontal = self.magnitude_vetorial(v_5_17)    # entre os pontos 5 e 17

        v_13_0 = self.p_to_vec(self.frame_buffer[frame][13], self.frame_buffer[frame][0])
        dist_vertical = self.magnitude_vetorial(v_13_0)      # entre os pontos 13 e 0
        
        # Caso a mão esteja inclinada de alguma forma, uma das distancias em pixeis diminuirá mais que a outra
        # dependendo da inclinação aplicada. Logo, o eixo menos afetado (maior distancia) será usado.
        # Por padrão, sem rotação, a distancia horizontal é cerca de 74.88% da vertical. Logo, *1.2511 para compensar
        if dist_horizontal*1.2511 > dist_vertical:
            distancia_pixels = dist_horizontal
            distancia_real = self.palm_width
        else:
            distancia_pixels = dist_vertical
            distancia_real = self.palm_heigth

        # Conversão do FOV diagonal para radianos
        fov_diagonal_rad = math.radians(self.cam_fov_diagonal)
        
        # Cálculo da diagonal da resolução em pixels
        diagonal_pixels = math.sqrt(self.cam_width**2 + self.cam_height**2)
        
        # Cálculo da tangente do ângulo FOV diagonal
        tan_fov_diagonal = math.tan(fov_diagonal_rad / 2)
        
        # Fórmula para calcular a distância do objeto
        distancia_objeto = (distancia_real * diagonal_pixels) / (2 * distancia_pixels * tan_fov_diagonal)

        # Na calibragem, foi observado uma margem de erro de 22.97% para mais
        distancia_objeto *= 0.770265674 # Compensa erro médio
        
        return distancia_objeto

    def estimate_angle_z(self, frame=F_ULTIMO):
        """
        Calcula a angulação da mão em torno do eixo Z (profundidade).
        
        Retorno:
            float - Ângulo em graus do dedo em relação ao eixo horizontal (eixo X positivo).
        """
        # Coordenadas dos pontos
        v_0_9 = self.p_to_vec(self.frame_buffer[frame][0], self.frame_buffer[frame][9])
        
        p_aux = self.frame_buffer[frame][0]
        p_aux = (p_aux[X]+3, p_aux[Y], p_aux[Z])    # Ponto auxiliar no eixo horizontal
        
        v_0_aux = self.p_to_vec(self.frame_buffer[frame][0], p_aux)
        
        return self.angle_vectors(v_0_9, v_0_aux)

    def estimate_angle_y(self, frame=F_ULTIMO):
        """
        Estima o ângulo de rotação da mão no eixo Y com base na análise da palma da mão.
        A função calcula o ângulo relativo da palma em relação à câmera, considerando os vetores
        formados pelos pontos-chave da base da palma (pulso, base do dedo indicador e base do dedo mínimo).

        Parâmetros:
            frame (int): Índice do quadro atual no buffer de quadros contendo os pontos da mão.
                        O valor padrão é F_ULTIMO, referindo-se ao quadro mais recente.

        Retorna:
            float: Ângulo estimado no eixo Y, mapeado para o intervalo [0, 180] graus.

        Observações:
            - O cálculo é limitado para rotações com inclinação significativa no eixo X.
        """
         # Ajuste baseado nas proporções entre largura e altura da palma  
        v_0_5 = self.p_to_vec(self.frame_buffer[frame][0], self.frame_buffer[frame][5])
        v_0_17 = self.p_to_vec(self.frame_buffer[frame][0], self.frame_buffer[frame][17])

        # Ângulo da perspectiva 2D dos vetores da base da palma da mão
        angle = self.angle_vectors(v_0_5, v_0_17)
            
        # Calcula a influência da rotacão no eixo x no ângulo perspectivo da palma medido
        # O excedente ao intervalo [41,-41] é devido à inclinação no eixo x.
        # Assim, com uma proporção entre o intervalo esperado e a alteração, calcula um novo intervalo para
        # rotação do eixo y.
        alteracao = 0
        if angle > 41 or angle < -41:
            alteracao = abs(angle) - 41
        
        proporcao = 1 + alteracao/41
        
        #! Limitado a pouca inclinação no eixo x, caso contrário, sera calculado 0 ou 180

        return self.mapear_intervalo(angle, 41*proporcao, -41*proporcao, 0, 180)

    def estimate_angle_x(self, frame=F_ULTIMO):
        """
        Estima o ângulo de rotação da mão no eixo X com base na análise da palma da mão.

        A função calcula o ângulo relativo da palma em relação à câmera, utilizando vetores
        formados pelos pontos-chave da base da palma (base do dedo indicador, base do dedo mínimo e pulso).

        Parâmetros:
            frame (int): Índice do quadro atual no buffer de quadros contendo os pontos da mão.
                        O valor padrão é F_ULTIMO, referindo-se ao quadro mais recente.

        Retorna:
            float: Ângulo estimado no eixo X, mapeado para o intervalo [0, 180] graus.

        Observações:
            - Há queda de precisão no intervalo [70, 120] graus devido a limitações na detecção de pontos pela projeção 2D.
            - Para inclinações significativas no eixo Y, o resultado pode ser limitado a 0 ou 180 graus,
            comprometendo a precisão da estimativa.
        """
        v_17_5 = self.p_to_vec(self.frame_buffer[frame][17], self.frame_buffer[frame][5])
        v_17_0 = self.p_to_vec(self.frame_buffer[frame][17], self.frame_buffer[frame][0])
        
        # Ângulo da perspectiva 2D dos vetores da base da palma da mão
        angle = self.angle_vectors(v_17_5, v_17_0)
        
        # Calcula a influência da rotacão no eixo x no ângulo perspectivo da palma medido
        # O excedente ao intervalo [-80,80] é devido à inclinação no eixo y.
        # Assim, com uma proporção entre o intervalo esperado e a alteração, calcula um novo intervalo para
        # rotação do eixo y.
        alteracao = 0
        if angle > 80 or angle < -80:
            alteracao = abs(angle) - 80
        
        proporcao = 1 + alteracao/80
        
        #! Queda na precisão no intervalo [70, 120] graus devido a limitações do OpenCV
        #! Limitado a pouca inclinação no eixo y, caso contrário, sera calculado 0 ou 180

        return self.mapear_intervalo(angle, -80*proporcao, 80*proporcao, 0, 180)

    def estimate_position(self, frame=F_ULTIMO, normalize_z=False):
        """
        Estima a posição central da mão no espaço tridimensional com base nos pontos-chave da palma
        (a base do dedo indicador, base do dedo mínimo e pulso).

        Parâmetros:
            frame (int): Índice do quadro utilizado para estimar a posição. 
                        O valor padrão é F_ULTIMO (último quadro no buffer).
            normalize_z (bool): Se `True`, o valor do eixo Z será normalizado para [-1,1].
                                Caso contrário, a profundidade real é estimada pelo método `estimate_depth`.

        Retorna:
            tuple: Coordenadas da posição da mão no espaço tridimensional (x, y, z), onde:
                - x e y representam a posição no plano da imagem, em pixels.
                - z representa a profundidade, em centímetros, se `normalize_z` for `False`. 
                Caso contrário, z será a coordenada normalizada fornecida pelo OpenCV.
        """
        b_indicador = self.frame_buffer[frame][5]
        pulso = self.frame_buffer[frame][0]
        b_minimo = self.frame_buffer[frame][17]

        x_central = (b_indicador[X] + pulso[X] + b_minimo[X]) / 3
        y_central = (b_indicador[Y] + pulso[Y] + b_minimo[Y]) / 3
        z_central = (b_indicador[Z] + pulso[Z] + b_minimo[Z]) / 3
        posicao = (x_central, y_central, z_central)

        if not normalize_z:
            posicao = (posicao[X], posicao[Y], self.estimate_depth(frame))
            
        return posicao
  
    def is_finger_flexed(self, dedo_id, frame=F_ULTIMO):
        """
        Verifica se o dedo está flexionado analisando a posição da ponta do dedo em relação 
        ao vetor de referência formado pelos pontos da mão detectados. A detecção é feita 
        com base no produto escalar entre dois vetores.

        Parâmetros:
            dedo_id (int): Índice da ponta do dedo a ser verificado.
            frame (int, opcional): Identificador do frame a ser analisado no buffer. 
                                Por padrão, utiliza F_ULTIMO, ou seja, o último frame processado.

        Retorno:
            bool: 
                - `True` se o dedo está flexionado.
                - `False` caso contrário.
        """

        # Vetores de comparação:
        # Para o polegar (dedo_id == 4), usa uma lógica especial devido à sua dinâmica diferente
        if dedo_id == 4:
            v_ref_dedo = self.p_to_vec(self.frame_buffer[frame][3], self.frame_buffer[frame][4])  # Vetor do polegar
            v_ref_0 = self.p_to_vec(self.frame_buffer[frame][2], self.frame_buffer[frame][0])    # Vetor de referência
        else:
            # Para os demais dedos, cria vetores da base do dedo até a ponta, e da base até o ponto de origem da mão.
            v_ref_dedo = self.p_to_vec(self.frame_buffer[frame][dedo_id-3], self.frame_buffer[frame][dedo_id])
            v_ref_0 = self.p_to_vec(self.frame_buffer[frame][dedo_id-3], self.frame_buffer[frame][0])

        # Depuração: Imprime o valor do produto escalar calculado entre os dois vetores.
        #print(f"id {dedo_id} -> {self.produto_escalar(v_ref_dedo, v_ref_0):.2f}")

        # Flexão do dedo é determinada pelo sinal do produto escalar:
        # - Se o produto escalar > 0: Vetores estão no mesmo sentido -> Dedo está flexionado.
        # - Caso contrário: Vetores estão em sentidos opostos -> Dedo está esticado.

        return self.produto_escalar(v_ref_dedo, v_ref_0) > 0

    def fingers_flexed(self):
        """
        Identifica quais dedos estão flexionados com base na posição e ângulos das articulações.
        
        Descrição:
            - O polegar é avaliado pelo ângulo entre suas articulações.
            - Os outros dedos utilizam a função `is_finger_flexed`, que verifica a posição da ponta do dedo
            em relação a uma linha perpendicular na articulação intermediária.

        Retorno:
            list - Índices dos dedos flexionados no frame atual.
        """
        dedos = [4,8,12,16,20]
        dedos_flex = []
        # lógica de flexão dos demais dedos baseado em retas perpendiculares e determinantes
        for dedo in dedos:
            if self.is_finger_flexed(dedo): dedos_flex.append(dedo)
            
        return dedos_flex

    def deslocation(self, frame0=F_PENULTIMO, frame1=F_ULTIMO):
        """
        Calcula o vetor de deslocamento no espaço 3D do ponto central da mão
        entre dois quadros.

        Parâmetros:
            frame0 (int): Índice do quadro de referência inicial. 
                        O valor padrão é F_PENULTIMO (penúltimo quadro no buffer).
            frame1 (int): Índice do quadro de referência final.
                        O valor padrão é F_ULTIMO (último quadro no buffer).

        Retorna:
            tuple: Vetor de deslocamento no espaço 3D (dx, dy, dz), onde:
            - dx e dy representam o deslocamento em pixels no plano 2D (imagem).
            - dz representa o deslocamento em profundidade, medido em centímetros (cm).

        Observações:
            - O deslocamento é calculado considerando a profundidade (eixo Z), estimada pelo método `estimate_depth`.
        """
        posicao_ref = self.estimate_position(frame0)
        posicao_atual = self.estimate_position(frame1)

        deslocamento = self.p_to_vec(posicao_ref, posicao_atual)
        
        return deslocamento
        

############################## FUNÇÕES AUXILIARES ##############################


    def mapear_intervalo(self, valor, antigo_min, antigo_max, novo_min, novo_max):
        return ((valor - antigo_min) * (novo_max - novo_min)) / (antigo_max - antigo_min) + novo_min
    
    def p_to_vec(self, p1, p2):
        vetor = (
            p2[X] - p1[X],
            p2[Y] - p1[Y],
            p2[Z] - p1[Z]
        )
        return vetor

    def produto_vetorial(self, v1, v2):
        # Produto vetorial 3D
        cross_product = (
            v1[1] * v2[2] - v1[2] * v2[1],  # Componente X
            v1[2] * v2[0] - v1[0] * v2[2],  # Componente Y
            v1[0] * v2[1] - v1[1] * v2[0],  # Componente Z
        )
        return cross_product

    def produto_escalar(self, v1, v2):
        return sum(a * b for a, b in zip(v1, v2))

    def magnitude_vetorial(self, v):
        return math.sqrt(sum(vi**2 for vi in v))

    def angle_vectors(self, v1, v2, min_angle=True):
        # Produto escalar entre os vetores para calcular o ângulo entre eles
        prod_escalar = self.produto_escalar(v1, v2)

        # Magnitudes dos vetores (usadas para normalizar o cosseno)
        mag_v1 = self.magnitude_vetorial(v1)
        mag_v2 = self.magnitude_vetorial(v2)

        # Cálculo do cosseno do ângulo
        mag_produto = mag_v1 * mag_v2
        if not mag_produto: mag_produto = 0.00000001 #evita divisao por zero
        cos_angle = prod_escalar / mag_produto

        # Garantir que o cosseno esteja no intervalo [-1, 1] para evitar erros numéricos
        cos_angle = max(-1, min(1, cos_angle))

        # Ângulo em graus calculado com base no cosseno
        angle = math.degrees(math.acos(cos_angle))

        # Produto vetorial para determinar o sinal da rotação no eixo Z
        if not min_angle:
            prod_vetorial = self.produto_vetorial(v1, v2)
            if prod_vetorial < 0:
                angle = -angle
            angle %= 2*math.pi

        # Devido a orientação das mãos serem diferentes, inverte o sinal
        if not self.right_hand: angle = -angle

        # Retorna o ângulo entre os vetores em perspectiva
        return angle