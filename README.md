# Detector - Jogo da Velha Robótico com Visão Computacional (Fanuc)

Este repositório contém scripts em Python para permitir que um robô Fanuc jogue Jogo da Velha contra um humano, utilizando visão computacional (OpenCV) para detectar o tabuleiro, as peças e calcular as jogadas, além de realizar a limpeza do tabuleiro após o jogo. A comunicação com o robô é feita através do protocolo CIP (Common Industrial Protocol) usando a biblioteca `pycomm3`.

## Funcionalidades Principais

* **Calibração de Homografia:** Mapeia os pixels da câmera para as coordenadas reais (milímetros) do robô.
* **Calibração da Grade:** Define a área de jogo e as posições centrais das 9 células do tabuleiro.
* **Gameplay Interativo:** O usuário joga clicando na célula desejada na janela de visualização.
* **Inteligência Artificial (Minimax):** O robô calcula a melhor jogada (ataque ou defesa) usando o algoritmo Minimax.
* **Comunicação Robô (CIP):** Envia as coordenadas da jogada para registradores específicos do robô Fanuc e utiliza flags para sincronização.
* **Limpeza Automática:** Após o fim do jogo, o robô automaticamente remove as peças do tabuleiro.

## Estrutura do Projeto

* **`calibracao_automatica_V3_salva_arquivo.py`:** Script para realizar a calibração da **homografia**. O robô se move para pontos pré-definidos, o script detecta um marcador nesses pontos (via câmera) e lê as coordenadas reais do robô via CIP. Salva os pares de pontos (pixel da câmera vs. mm do robô) no arquivo `pontos_calibracao.txt`.
* **`Calibrador_Grade_v5_salva_pixels.py`:** Script interativo (com sliders) para calibrar a **detecção da grade** e a **estimativa dos centros**. O usuário ajusta os parâmetros de threshold adaptativo e área para isolar o quadrado central do tabuleiro. O script então estima a posição dos 9 centros (em pixels) e salva essas coordenadas no arquivo `grid_calibracao.txt` ao pressionar ESC.
* **`gameplaysupremo.py`:** Script principal que executa o **Jogo da Velha** e a **limpeza automática**. Ele lê os arquivos de calibração (`pontos_calibracao.txt` e `grid_calibracao.txt`), gerencia os turnos, calcula a jogada do robô, envia as coordenadas para o robô via CIP e coordena a limpeza no final.

## Pré-requisitos

* Python 3.x
* Bibliotecas Python:
    * `opencv-python`
    * `numpy`
    * `pycomm3`
* Robô Fanuc configurado para comunicação CIP via Ethernet.
* Uma câmera conectada ao computador onde o script Python será executado.

## Instalação

1.  Clone o repositório:
    ```bash
    git clone [https://github.com/lucaspz100/Detector.git](https://github.com/lucaspz100/Detector.git)
    cd Detector
    ```
2.  Instale as bibliotecas necessárias:
    ```bash
    pip install opencv-python numpy pycomm3
    ```

## Configuração

Antes de executar os scripts, verifique e ajuste:

* **IP do Robô:** Edite a variável `ip_robot` (ou `IP_DO_ROBO`) nos scripts `.py` para corresponder ao endereço IP do seu robô Fanuc.
* **Índice da Câmera:** Edite a variável `camera_index` (ou `CAMERA_INDEX`) nos scripts `.py` para o índice correto da sua câmera (geralmente 0 ou 1).

## Uso

### 1. Calibração da Homografia

* **Objetivo:** Criar o arquivo `pontos_calibracao.txt` que relaciona pixels da câmera com coordenadas do robô.
* **Como usar:**
    1.  Execute o script `calibracao_automatica_V3_salva_arquivo.py`.
    2.  Execute o programa correspondente no Teach Pendant do robô Fanuc. Este programa deve mover o robô sequencialmente para os 9 pontos de calibração.
    3.  Em cada ponto, o programa do robô deve:
        * Definir R\[5] = 1 (sinalizar para o Python).
        * Esperar um curto período (ex: 1 segundo).
        * (Opcional, mas recomendado) Definir R\[5] = 0.
    4.  O script Python detectará o pulso em R\[5], lerá as coordenadas X e Y atuais do robô (de R\[6] e R\[7]), detectará a posição do marcador na imagem da câmera e salvará o par de pontos.
    5.  Após coletar os 9 pontos, o script salvará os dados em `pontos_calibracao.txt` e calculará/salvará a matriz de homografia (opcionalmente).

### 2. Calibração da Grade (Visual)

* **Objetivo:** Criar o arquivo `grid_calibracao.txt` com as coordenadas em **pixels** dos 9 centros estimados do tabuleiro.
* **Como usar:**
    1.  Execute o script `Calibrador_Grade_v5_salva_pixels.py`.
    2.  Posicione a câmera sobre o tabuleiro vazio.
    3.  Ajuste os sliders (`BlockSize`, `C`, `MIN_AREA`, `MAX_AREA`, `TapeThickness_px`) até que o script detecte corretamente o **quadrado central** (contorno verde) e **estime** corretamente a posição dos 9 centros (círculos azuis). A mensagem "Grade Estimada (Centro OK)" deve aparecer em verde.
    4.  Pressione a tecla `ESC`. Os 9 centros estimados (em coordenadas de pixel) serão salvos no arquivo `grid_calibracao.txt`.

### 3. Jogo da Velha e Limpeza

* **Objetivo:** Jogar Jogo da Velha contra o robô e, ao final, fazer o robô limpar o tabuleiro.
* **Como usar:**
    1.  Certifique-se que os arquivos `pontos_calibracao.txt` e `grid_calibracao.txt` existem no mesmo diretório.
    2.  Execute o script `gameplaysupremo.py`.
    3.  **Pressione 'g':** O script carregará os centros da grade do arquivo `grid_calibracao.txt`, calculará as coordenadas do robô correspondentes e os limites da área de jogo. Os centros serão exibidos na tela.
    4.  **Clique:** Clique na célula onde você (jogador 'X') deseja jogar.
        * O script envia as coordenadas (X, Y) do centro da célula clicada para R\[1] e R\[2].
        * O script **liga** R\[5] (seta para 1).
        * *O programa do robô deve detectar R\[5]=1, pegar uma peça 'X', ir para R\[1], R\[2], colocar a peça, e **desligar** R\[5] (setar para 0).*
    5.  **Jogada do Robô:** Quando o script Python detectar que R\[5] voltou a 0 (após a jogada do usuário), ele:
        * Calcula a melhor jogada ('O') usando Minimax.
        * Envia as coordenadas (X, Y) da jogada do robô para R\[1] e R\[2].
        * **Liga** R\[5] (seta para 1).
        * *O programa do robô deve detectar R\[5]=1, pegar uma peça 'O', ir para R\[1], R\[2], colocar a peça, e **desligar** R\[5] (setar para 0).*
    6.  O ciclo se repete até o jogo terminar (vitória, derrota ou empate).
    7.  **Limpeza Automática:** Assim que o jogo termina (`game_over == True`) e o robô termina sua última ação (`R[5] == 0`), a sequência de limpeza começa:
        * O script **liga** R\[9] (seta para 1).
        * Ele detecta todas as peças (Azul=1, Vermelho=2) *dentro* da área da grade.
        * Para cada peça encontrada:
            * Envia as coordenadas (X, Y) da peça para R\[1], R\[2].
            * Envia o ID da cor (1 ou 2) para R\[8].
            * **Liga** R\[5] (seta para 1).
            * *O programa do robô deve detectar R\[9]=1 e R\[5]=1, ir para R\[1], R\[2], pegar a peça de cor R\[8], guardá-la, e **desligar** R\[5] (setar para 0).*
            * O script Python detecta R\[5]=0 e repete o processo para a próxima peça detectada.
        * Quando não há mais peças detectadas na grade, o script **desliga** R\[9] (seta para 0).
    8.  **Pressione 'r':** Reseta o jogo a qualquer momento (se o robô não estiver ocupado ou limpando). Tenta forçar R\[5] e R\[9] para 0.
    9.  **Pressione 'ESC':** Encerra o programa.

## Observações

* O programa do robô Fanuc (TP) precisa ser desenvolvido separadamente para interpretar os registradores (R\[1], R\[2], R\[5], R\[8], R\[9]) e realizar os movimentos correspondentes (pegar peça X/O, ir para coordenada, colocar peça, pegar peça para limpeza, guardar peça).
* A precisão da homografia e da detecção da grade é crucial para o bom funcionamento.
* A iluminação do ambiente pode afetar a detecção das peças e da grade. A recalibração da grade (`Calibrador_Grade_v5_salva_pixels.py`) pode ser necessária se a iluminação mudar significativamente.
