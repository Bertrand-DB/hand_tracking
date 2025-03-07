import cv2
from mediapipe.python import solution_base
from mediapipe.python.solutions import hands
from mediapipe.python.solutions import drawing_utils


class HandTrack:
    """
    Classe para rastrear mãos usando Mediapipe e OpenCV.

    Atributos:
        cam (cv2.VideoCapture): Objeto que gerencia a captura de vídeo da câmera.
        detect_hands (module): Módulo do Mediapipe responsável pela detecção de mãos.
        map_hands (Hands): Objeto do Mediapipe configurado para processar landmarks das mãos.
        image (numpy.ndarray): Última imagem capturada pela câmera.
        results (Hands.process): Resultados do processamento da imagem com landmarks detectados.
        hands (list): Lista que armazenará os landmarks detectados em cada quadro.
    """
    def __init__(self, cam_index):
        """
        Inicializa o objeto HandTrack com a câmera e as configurações do Mediapipe.

        Parâmetros:
            cam_index (int): Índice da câmera a ser utilizada. 
                             (0 para a câmera padrão, 1 para uma câmera externa, etc.)
        """
        # Inicializa a captura de vídeo a partir do índice fornecido
        self.__cam_init(cam_index)
        
        # Carrega o módulo de detecção de mãos do Mediapipe
        self.detect_hands = hands
        
        # Configura o processador de landmarks das mãos
        self.map_hands = self.detect_hands.Hands()
        
        # Inicializa os atributos para armazenar a imagem, os resultados e os landmarks
        self.image = None    # Armazena o último quadro capturado
        self.results = None  # Armazena os resultados do processamento Mediapipe
        self.hands = None    # Armazena os landmarks detectados


    def __del__(self):
        """
        Método destrutor da classe HandTrack.

        Libera os recursos alocados durante a execução:
            - Fecha a captura de vídeo para evitar vazamento de memória ou travamento do dispositivo.
            - Fecha todas as janelas do OpenCV abertas durante o processamento.

        Este método é automaticamente chamado pelo Python quando o objeto é destruído.
        """
        # Libera a câmera
        self.cam.release()
        
        # Fecha todas as janelas abertas pelo OpenCV
        cv2.destroyAllWindows()

    def __cam_init(self, cam_index):
        """
        Inicializa a câmera com o índice fornecido. Caso ocorra algum erro durante a inicialização, 
        o erro é capturado e tratado, e a câmera é configurada como None.

        Parâmetros:
            cam_index (int): Índice da câmera a ser inicializada. Normalmente, 0 para a câmera padrão.
        
        Retorno:
            Nenhum. Caso haja falha na inicialização, a câmera é configurada como None.
        """
        try:
            self.cam = cv2.VideoCapture(cam_index)

            if not self.cam.isOpened():
                raise ValueError("Não foi possível acessar a câmera.")
        
        # Trata exceções específicas de erro de valor
        except ValueError as val_err:
            print(f"Erro ao inicializar a câmera: {val_err}")
            self.cam = None
        
        # Trata exceções genéricas
        except Exception as err:
            print(f"Erro ao inicializar a câmera: {err}")
            self.cam = None

    def process_image(self):
        """
        Processa um quadro capturado pela câmera, converte-o para o formato necessário, 
        e detecta landmarks de mãos na imagem.

        Retorno:
            bool: Retorna `True` se a imagem foi processada com sucesso e landmarks foram detectados. 
                Retorna `False` caso não seja possível capturar um quadro da câmera.
        """
        # Captura um quadro da câmera
        success, self.image = self.cam.read()
        
        # Verifica se a captura foi bem-sucedida
        if not success: 
            print("Erro: A câmera selecionada não retornou imagens")
            return False

        # Espelha horizontalmente a imagem para criar uma visão de "espelho"
        self.image = cv2.flip(self.image, 1)

        # Converte a imagem do formato BGR (OpenCV) para RGB (Mediapipe)
        imageRGB = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)

        # Processa a imagem para detectar landmarks das mãos
        self.results = self.map_hands.process(imageRGB)

        # Armazena os landmarks detectados
        self.hands = self.results.multi_hand_landmarks

        # Retorna True para indicar que o processamento foi bem-sucedido
        return True

    def draw_hand(self, show_landmark_id=False):
        """
        Desenha os landmarks das mãos detectadas na imagem, com a opção de exibir os IDs dos landmarks.

        Parâmetros:
            show_landmark_id (bool): 
                - Se `False` (padrão), desenha apenas os landmarks e suas conexões na imagem.
                - Se `True`, desenha também os números identificadores (IDs) dos landmarks próximos aos pontos.
        """
        # Se nenhuma mão foi detectada, encerra o método
        if not self.hands: 
            return

        # Caso a exibição dos IDs dos landmarks não seja necessária
        if not show_landmark_id:
            for hand in self.hands:
                # Desenha os landmarks e suas conexões na imagem
                drawing_utils.draw_landmarks(self.image, hand, self.detect_hands.HAND_CONNECTIONS)
        else:
            # Caso a exibição dos IDs dos landmarks seja necessária
            for hand in self.hands:
                drawing_utils.draw_landmarks(self.image, hand, self.detect_hands.HAND_CONNECTIONS)
                
                # Percorre cada landmark da mão detectada
                for id, coord in enumerate(hand.landmark):
                    h, w, _ = self.image.shape
                    # Converte as coordenadas normalizadas (0-1) para coordenadas de pixels
                    cx, cy = int(coord.x * w), int(coord.y * h) 
                    # Desenha o ID do landmark próximo ao ponto correspondente
                    cv2.putText(self.image, str(id), (cx+5, cy+15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    def show_image(self):
        """
        Exibe a imagem da câmera em uma janela utilizando o OpenCV.

        A função utiliza o OpenCV para mostrar a imagem atual armazenada na variável `self.image`
        em uma janela chamada "Hand Tracking". Essa janela será atualizada toda vez que a imagem 
        for alterada ou capturada.
        """
        cv2.imshow("Hand Tracking", self.image)

    def get_hands_coordinates(self):
        """
        Obtém as coordenadas 3D dos marcos (landmarks) de cada mão detectada,
        separando-as entre mão esquerda e direita.

        A função verifica se há mãos detectadas e armazena as coordenadas das mãos esquerda
        e direita em listas separadas, incluindo as coordenadas de profundidade (z) normalizadas.

        Retorno:
            tuple: (left_hand_coord, right_hand_coord) - Tupla contendo duas listas de coordenadas (x, y, z):
                - left_hand_coord: Coordenadas 3D dos marcos da mão esquerda (ou False se não detectada).
                - right_hand_coord: Coordenadas 3D dos marcos da mão direita (ou False se não detectada).
        
        Observação:
            - Coordenadas x e y retornadas em pixeis, já a coordenada z, nomalizada.
        """
        left_hand_coord = []
        right_hand_coord = []

        if self.hands:  # Verifica se há mãos detectadas
            for idx, hand in enumerate(self.hands):
                # Obter a dominância da mão (esquerda ou direita)
                handedness = self.results.multi_handedness[idx].classification[0].label  # "Left" ou "Right"

                # Adicionar as coordenadas para as listas apropriadas, dependendo da mão (esquerda ou direita)
                if handedness == "Left":
                    for coord in hand.landmark:
                        # Converter as coordenadas normalizadas dos landmarks para pixels
                        h, w, _ = self.image.shape
                        x, y = int(coord.x * w), int(coord.y * h)
                        z = coord.z  # Profundidade relativa normalizada
                        left_hand_coord.append((x, y, z))
                else:
                    for coord in hand.landmark:
                        # Converter as coordenadas normalizadas dos landmarks para pixels
                        h, w, _ = self.image.shape
                        x, y = int(coord.x * w), int(coord.y * h)
                        z = coord.z  # Profundidade relativa normalizada
                        right_hand_coord.append((x, y, z))

        # Se nenhuma coordenada foi detectada para uma mão, define como False
        if len(left_hand_coord) == 0: 
            left_hand_coord = False
        if len(right_hand_coord) == 0: 
            right_hand_coord = False

        # Retorna as coordenadas 3D das mãos esquerda e direita
        return (left_hand_coord, right_hand_coord)


