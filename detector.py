import struct
import cv2
import numpy as np
from pycomm3 import CIPDriver, Services
import sys
import time
import threading

# --- FAIXA DE COR AZUL ---
limite_inferior_azul = np.array([80, 120, 70])
limite_superior_azul = np.array([150, 255, 255])

# --- FAIXA DE COR VERMELHA ---
limite_inferior_vermelho1 = np.array([0, 100, 100])
limite_superior_vermelho1 = np.array([10, 255, 255])
limite_inferior_vermelho2 = np.array([170, 100, 100])
limite_superior_vermelho2 = np.array([180, 255, 255])

# --- PONTOS DE CALIBRAÇÃO DE HOMOGRAFIA ---
# Os pontos que a CÂMERA viu (em pixels brutos)
p_camera = np.array([
    [632, 147],
    [645, 520],
    [652, 863],
    [1016, 860],
    [1012, 525],
    [1010, 107],
    [1409, 107],
    [1409, 441],
    [1409, 768]
], dtype=np.float32)

# Os pontos REAIS do ROBÔ (em milímetros)
p_robot = np.array([
    [365, 225],  # Ponto 1
    [367, 0],  # Ponto 2
    [367, -214],  # Ponto 3
    [594, -214],  # Ponto 4
    [594, -7],  # Ponto 5
    [594, 248],  # Ponto 6
    [840, 248],  # Ponto 7
    [840, 43],  # Ponto 8
    [840, -161]  # Ponto 9
], dtype=np.float32)

# --- CALCULA A HOMOGRAFIA ---
H, _ = cv2.findHomography(p_camera, p_robot, cv2.RANSAC)
if H is None:
    print("ERRO: Não foi possível calcular a homografia. Verifique os pontos.")
    sys.exit()

# --- CLASSE DE COMUNICAÇÃO CIP ---
class FanucCIP:
    def __init__(self, ip_robot, cam_index=0):
        self.ip = ip_robot
        self.cam_index = cam_index
        self.connected = False
        self.plc = None
        self.last_X = 0.0
        self.last_Y = 0.0
        self.last_Angle = 0.0
        self.last_Color_ID = 0 # 1=Azul, 2=Vermelho

    def connect(self):
        try:
            self.plc = CIPDriver(self.ip)
            self.plc.open()
            self.connected = True
            print(f"Conectado ao robô Fanuc em {self.ip}")
            return True
        except Exception as e:
            print(f"Erro na conexão CIP: {e}")
            self.connected = False
            return False

    def disconnect(self):
        if self.connected:
            try:
                self.plc.close()
                print("Desconectado do robô CIP.")
            except Exception as e:
                print(f"Erro ao desconectar: {e}")
            finally:
                self.connected = False

    def write_cip_explicit_register(self, register_index, value):
        if not self.connected:
            return False
        try:
            int_value = int(round(float(value)))
            data_payload = int_value.to_bytes(4, 'little', signed=True)
            response = self.plc.generic_message(
                service=Services.set_attribute_single,
                class_code=0x6B,
                instance=0x01,
                attribute=register_index,
                request_data=data_payload,
                connected=True
            )
            if response.error:
                print(f"ERRO CIP ESCRITA R[{register_index}]: {response.error}")
                return False
            return True
        except Exception as e:
            print(f"ERRO EXCEÇÃO ESCRITA CIP R[{register_index}]: {e}")
            return False

    # --- FUNÇÕES DE PULSO ---
    def _pulse_flag_threaded(self, register_index, duration_sec):
        try:
            print(f"THREAD: Ligando R[{register_index}]...")
            success_on = self.write_cip_explicit_register(register_index, 1)
            time.sleep(duration_sec)
            print(f"THREAD: Desligando R[{register_index}]...")
            success_off = self.write_cip_explicit_register(register_index, 0)
            if not (success_on and success_off):
                print(f"THREAD: Erro ao pulsar R[{register_index}].")
        except Exception as e:
            print(f"THREAD: Exceção na thread do pulso: {e}")

    def pulse_flag(self, register_index, duration_sec=1.0):
        if not self.connected:
            print("ERRO (Pulso): Robô desconectado.")
            return
        pulse_thread = threading.Thread(
            target=self._pulse_flag_threaded,
            args=(register_index, duration_sec)
        )
        pulse_thread.daemon = True
        pulse_thread.start()
    # --- FIM DAS FUNÇÕES DE PULSO ---

    def aplicar_homografia(self, x_pixel, y_pixel):
        global H
        ponto_pixel = np.array([[[x_pixel, y_pixel]]], dtype=np.float32)
        ponto_robo_transformado = cv2.perspectiveTransform(ponto_pixel, H)
        X_robo = ponto_robo_transformado[0][0][0]
        Y_robo = ponto_robo_transformado[0][0][1]
        return X_robo, Y_robo

    def _processar_contornos(self, contornos, cor_id, lista_blocos, frame_para_desenho):
        """
        Função auxiliar para encontrar blocos, calcular dados e desenhar.
        cor_id: 1 para Azul, 2 para Vermelho
        """
        cor_desenho = (0, 255, 0) if cor_id == 1 else (0, 0, 255)

        for c in contornos:
            if cv2.contourArea(c) > 100:
                rect = cv2.minAreaRect(c)
                (x_pixel, y_pixel), (width, height), angle = rect

                # --- LÓGICA DE ÂNGULO REINTRODUZIDA ---
                # Etapa 1: Converte o ângulo do OpenCV para o padrão matemático
                
                angulo_real = angle
                
                if angulo_real >45:
                    angulo_real = angulo_real-45
                else:
                    angulo_real=-angulo_real

                
                # Aplica a homografia
                X_robot, Y_robot = self.aplicar_homografia(x_pixel, y_pixel)

                # Desenha o contorno
                box = cv2.boxPoints(rect)
                box = np.intp(box)
                cv2.drawContours(frame_para_desenho, [box], 0, cor_desenho, 2)

                # Adiciona o bloco válido à lista
                lista_blocos.append({
                    'x_pixel': x_pixel,
                    'x_robo': X_robot,
                    'y_robo': Y_robot,
                    'angulo': angulo_real,
                    'cor_id': cor_id,
                })

    def run_vision_and_send(self):
        cap = cv2.VideoCapture(self.cam_index)
        if not cap.isOpened():
            print(f"Erro: não foi possível abrir a câmera ({self.cam_index}).")
            return

        # --- Configurando a Resolução ---
        desired_width = 1920
        desired_height = 1080
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)
        width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print(f"Resolução da câmera definida para: {width}x{height}")
        # ------------------------------------

        print("\n--- VISÃO 2D (MULTI-COR) e ENVIO CIP (COM ÂNGULO) ---")
        print("Pressione 'v' para enviar (X, Y, Ângulo, Cor) do bloco MAIS À DIREITA.")
        print("Pressione 'ESC' para sair.")
        print("------------------------------------------------------\n")

        DETECTION_SUCCESS = False

        while True:
            ret, frame = cap.read()
            if not ret:
                break

            DETECTION_SUCCESS = False
            blocos_detectados = []

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # --- Processar Cor Azul ---
            mascara_azul = cv2.inRange(hsv, limite_inferior_azul, limite_superior_azul)
            mascara_azul = cv2.erode(mascara_azul, None, iterations=2)
            mascara_azul = cv2.dilate(mascara_azul, None, iterations=2)
            contornos_azul, _ = cv2.findContours(mascara_azul, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            self._processar_contornos(contornos_azul, 1, blocos_detectados, frame)

            # --- Processar Cor Vermelha ---
            mascara_v1 = cv2.inRange(hsv, limite_inferior_vermelho1, limite_superior_vermelho1)
            mascara_v2 = cv2.inRange(hsv, limite_inferior_vermelho2, limite_superior_vermelho2)
            mascara_vermelho = cv2.bitwise_or(mascara_v1, mascara_v2)
            mascara_vermelho = cv2.erode(mascara_vermelho, None, iterations=2)
            mascara_vermelho = cv2.dilate(mascara_vermelho, None, iterations=2)
            contornos_vermelho, _ = cv2.findContours(mascara_vermelho, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            self._processar_contornos(contornos_vermelho, 2, blocos_detectados, frame)

            # --- Máscara combinada (para exibição) ---
            mascara_total = cv2.bitwise_or(mascara_azul, mascara_vermelho)

            # --- Seleção de Alvo (O MAIS À DIREITA) ---
            if len(blocos_detectados) > 0:
                # Encontra o bloco com o MAIOR 'x_pixel'
                bloco_alvo = max(blocos_detectados, key=lambda b: b['x_pixel'])

                # Armazena os dados do alvo para envio
                self.last_X = bloco_alvo['x_robo']
                self.last_Y = bloco_alvo['y_robo']
                self.last_Angle = bloco_alvo['angulo']
                self.last_Color_ID = bloco_alvo['cor_id']
                DETECTION_SUCCESS = True

                # Atualiza o HUD
                cor_nome = "Azul" if self.last_Color_ID == 1 else "Vermelho"
                robo_texto = f"ALVO (Direita): X={self.last_X:.1f} Y={self.last_Y:.1f} A={self.last_Angle:.1f} Cor={cor_nome}({self.last_Color_ID})"
                cv2.putText(frame, robo_texto, (5, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # --- Status de Envio na Tela ---
            if DETECTION_SUCCESS:
                status_conn = "PRONTO para 'v'" if self.connected else "DESCONECTADO"
                texto_status = f"Status: {status_conn}"
                cor_texto = (0, 255, 255) # Amarelo
                cv2.putText(frame, texto_status, (5, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.7, cor_texto, 2)

            elif not DETECTION_SUCCESS:
                status_msg = "BUSCANDO OBJETOS..." if self.connected else "DESCONECTADO! BUSCANDO OBJETOS..."
                cv2.putText(frame, status_msg, (5, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            # Redimensiona a janela de exibição para caber na tela
            frame_display = cv2.resize(frame, (1280, 720), interpolation=cv2.INTER_AREA)
            mascara_display = cv2.resize(mascara_total, (1280, 720), interpolation=cv2.INTER_NEAREST)

            cv2.imshow('Frame Original (1080p)', frame_display)
            cv2.imshow('Mascara (Azul e Vermelho)', mascara_display)

            # --- MONITORAMENTO DE TECLAS ---
            key = cv2.waitKey(1) & 0xFF
            if key == 27: # ESC para Sair
                break

            if key == ord('v'): # 'v' para enviar TUDO
                if self.connected and DETECTION_SUCCESS:
                    print("\nTecla 'v' pressionada. Enviando dados do alvo (mais à direita)...")

                    # 1. Envia X, Y, Ângulo e COR
                    success_X = self.write_cip_explicit_register(1, self.last_X)
                    success_Y = self.write_cip_explicit_register(2, self.last_Y)
                    success_A = self.write_cip_explicit_register(3, self.last_Angle)
                    success_C = self.write_cip_explicit_register(4, self.last_Color_ID)

                    if success_X and success_Y and success_A and success_C:
                        print(f"ENVIO COORDENADAS OK: X={self.last_X:.1f}, Y={self.last_Y:.1f}, A={self.last_Angle:.1f}, COR={self.last_Color_ID}")

                        # 2. Pulsa R[5]
                        self.pulse_flag(5, 1.0)

                    else:
                        print("Falha ao enviar coordenadas (X, Y, A ou C) CIP.")
                        if not self.connected:
                            print("Tentando reconectar...")
                            self.connect()

                elif not self.connected:
                    print("ERRO: Robô desconectado.")
                else:
                    print("ERRO: Nenhum objeto detectado.")

        # Libera a câmera e fecha as janelas
        cap.release()
        cv2.destroyAllWindows()

# --- PONTO DE ENTRADA DO SCRIPT ---
if __name__ == "__main__":
    # **ALTERE O IP E O ÍNDICE DA CÂMERA AQUI**
    ip_robot = "192.168.1.100"
    camera_index = 1
    # *****************************************

    fanuc = FanucCIP(ip_robot, camera_index)

    # Tenta conectar e rodar
    if fanuc.connect():
        fanuc.run_vision_and_send()
        fanuc.disconnect()
    else:
        # Se não conectar, roda mesmo assim (apenas para debug da visão)
        print("Rodando apenas visão (sem conexão CIP).")
        fanuc.run_vision_and_send()