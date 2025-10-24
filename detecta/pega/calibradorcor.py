import cv2
import numpy as np

# --- PASSO DE CONFIGURAÇÃO ---
# Você PRECISA ajustar estes valores para a cor do seu bloco e sua iluminação.
# Exemplo para um bloco AZUL VIVO. Use um "HSV Color Picker" online para achar os valores da sua cor.
limite_inferior_cor = np.array([90, 80, 50])    # HSV mínimo para azul
limite_superior_cor = np.array([130, 255, 255]) # HSV máximo para azul
# -----------------------------

# Inicia a captura de vídeo
cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("Erro: Não foi possível abrir a câmera.")
    exit()

while True:
    # 1. Captura do frame
    ret, frame = cap.read()
    if not ret:
        break

    # 2. Conversão de BGR para HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 3. Criação da máscara para a cor específica
    mascara = cv2.inRange(hsv, limite_inferior_cor, limite_superior_cor)
    
    # Opcional: Aplicar operações morfológicas para remover ruídos da máscara
    mascara = cv2.erode(mascara, None, iterations=2)
    mascara = cv2.dilate(mascara, None, iterations=2)

    # 4. Encontrar contornos na máscara
    # cv2.RETR_EXTERNAL é ótimo para pegar apenas o contorno externo do objeto
    # cv2.CHAIN_APPROX_SIMPLE remove pontos redundantes e economiza memória
    contornos, _ = cv2.findContours(mascara.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Verifica se algum contorno foi encontrado
    if len(contornos) > 0:
        # Pega o maior contorno (assumindo que seja o nosso bloco)
        c = max(contornos, key=cv2.contourArea)
        
        # Filtra contornos muito pequenos para evitar ruído
        if cv2.contourArea(c) > 500: # O valor 500 é um exemplo, ajuste se necessário
            # --- CÁLCULO DA POSIÇÃO (CENTRO) ---
            M = cv2.moments(c)
            if M["m00"] != 0:
                # Calcula o centroide (cx, cy) do bloco
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                posicao_texto = f"Pos: ({cx}, {cy})"
                cv2.circle(frame, (cx, cy), 7, (255, 255, 255), -1) # Desenha o centro
                cv2.putText(frame, posicao_texto, (cx - 50, cy - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # --- CÁLCULO DA ROTAÇÃO ---
            # Encontra o retângulo de área mínima que envolve o contorno
            rect = cv2.minAreaRect(c)
            # rect é uma tupla: ((centro_x, centro_y), (largura, altura), angulo_de_rotacao)
            box = cv2.boxPoints(rect)
            box = np.intp(box) # Converte para inteiros
            
            angulo = round(rect[2], 2)
            rotacao_texto = f"Rot: {angulo} graus"
            
            # Desenha o retângulo rotacionado no frame original
            cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)
            cv2.putText(frame, rotacao_texto, (cx - 50, cy + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # Mostra os frames
    cv2.imshow('Frame Original', frame)
    cv2.imshow('Mascara de Cor', mascara)

    # Condição de saída (tecla ESC)
    if cv2.waitKey(1) & 0xFF == 27:
        break

# Libera os recursos
cap.release()
cv2.destroyAllWindows()