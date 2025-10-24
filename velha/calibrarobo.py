import struct
import cv2
import numpy as np
from pycomm3 import CIPDriver, Services
import sys
import time

# --- CONFIGURAÇÕES DE VISÃO ---
limite_inferior_cor = np.array([80, 120, 70])
limite_superior_cor = np.array([150, 255, 255])

# --- CONFIGURAÇÕES DE CALIBRAÇÃO ---
IP_DO_ROBO = "192.168.1.100" 
CAMERA_INDEX = 1
# ALTERADO PARA 9 PONTOS
NUM_PONTOS_PARA_CALIBRAR = 9

# Registradores do Robô (APENAS LEITURA)
REG_FLAG = 5  # R[5] - Flag de Pulso (Robô define como 1)
REG_X = 6     # R[6] - Posição X real do robô
REG_Y = 7     # R[7] - Posição Y real do robô

# --- CLASSE DE COMUNICAÇÃO CIP (SOMENTE LEITURA) ---
class FanucCIPCalibrator:
    def __init__(self, ip_robot):
        self.ip = ip_robot
        self.connected = False
        self.plc = None

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
            self.plc.close()
            print("Desconectado do robô CIP.")
            self.connected = False

    def read_register(self, register_index):
        """Lê um valor (INT32) do registrador R[]."""
        if not self.connected: return None, False
        try:
            response = self.plc.generic_message(
                service=Services.get_attribute_single,
                class_code=0x6B,
                instance=0x01,
                attribute=register_index,
                connected=True
            )
            if response.error: 
                return None, False
            value = int.from_bytes(response.value, 'little', signed=True)
            return value, True
        except Exception as e:
            return None, False

def detectar_bloco(frame):
    """Detecta o bloco na imagem e retorna seu centro (x_pixel, y_pixel)."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mascara = cv2.inRange(hsv, limite_inferior_cor, limite_superior_cor)
    mascara = cv2.erode(mascara, None, iterations=2)
    mascara = cv2.dilate(mascara, None, iterations=2)
    contornos, _ = cv2.findContours(mascara.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contornos) > 0:
        c = max(contornos, key=cv2.contourArea)
        if cv2.contourArea(c) > 100: 
            (x_pixel, y_pixel), raio = cv2.minEnclosingCircle(c)
            cv2.circle(frame, (int(x_pixel), int(y_pixel)), int(raio), (0, 255, 0), 2)
            cv2.circle(frame, (int(x_pixel), int(y_pixel)), 5, (0, 0, 255), -1)
            return (x_pixel, y_pixel), frame, mascara
    return None, frame, mascara

# --- NOVA FUNÇÃO PARA FORMATAR A SAÍDA ---
def formatar_e_imprimir_pontos(lista_camera, lista_robo):
    """
    Pega as listas de pontos coletados e imprime no formato
    de array numpy para copiar e colar.
    """
    print("\n" + "="*50)
    print("--- CÓDIGO DE CALIBRAÇÃO PARA COPIAR E COLAR ---")
    print("="*50 + "\n")

    # --- Formatar p_camera ---
    print("# --- PONTOS DE CALIBRAÇÃO DE HOMOGRAFIA ---")
    print("# Os pontos que a CÂMERA viu (em pixels brutos)")
    print("p_camera = np.array([")
    
    num_pontos_cam = len(lista_camera)
    for i, ponto in enumerate(lista_camera):
        # Formata como [XXX, YYY] (arredondado para inteiro)
        linha = f"    [{int(round(ponto[0]))}, {int(round(ponto[1]))}]"
        if i < num_pontos_cam - 1:
            linha += ","
        print(linha)
        
    print("], dtype=np.float32)\n")

    # --- Formatar p_robot ---
    print("# Os pontos REAIS do ROBÔ (em milímetros)")
    print("p_robot = np.array([")
    
    num_pontos_robo = len(lista_robo)
    for i, ponto in enumerate(lista_robo):
        # Formata como [XXX, YYY],  # Ponto N
        # Os pontos do robô (R[6], R[7]) já são inteiros
        linha = f"    [{ponto[0]}, {ponto[1]}]"
        if i < num_pontos_robo - 1:
            linha += ","
        linha += f"  # Ponto {i+1}"
        print(linha)
        
    print("], dtype=np.float32)\n")
    print("="*50)
    print("--- FIM DO CÓDIGO ---")
    print("="*50 + "\n")


# --- SCRIPT PRINCIPAL DE CALIBRAÇÃO ---
if __name__ == "__main__":
    
    print("--- INICIANDO SEQUÊNCIA DE CALIBRAÇÃO (Detecção de Pulso) ---")
    
    fanuc = FanucCIPCalibrator(IP_DO_ROBO)
    cap = cv2.VideoCapture(CAMERA_INDEX)
    
    if not cap.isOpened():
        print(f"Erro: Não foi possível abrir a câmera {CAMERA_INDEX}")
        sys.exit()

    # --- MUDANÇA AQUI: DEFININDO A RESOLUÇÃO ---
    print(f"Definindo resolução para 1920x1080...")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    # Verifica se a resolução foi aceita
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print(f"Resolução da câmera confirmada: {width}x{height}")
    
    if width == 0 or height == 0:
        print("ERRO: A câmera não suporta ou não aceitou a resolução 1920x1080.")
        print("Verifique o índice da câmera ou tente uma resolução diferente.")
        cap.release()
        sys.exit()
    # --- FIM DA MUDANÇA ---
    
    if not fanuc.connect():
        print("Não foi possível conectar ao robô. Encerrando.")
        cap.release()
        sys.exit()

    p_camera_list = []
    p_robot_list = []

    last_flag_state = 0 

    print(f"\n[AVISO] Inicie o programa de calibração no robô.")
    print(f"Aguardando {NUM_PONTOS_PARA_CALIBRAR} pulsos em R[{REG_FLAG}]...")

    try:
        while len(p_camera_list) < NUM_PONTOS_PARA_CALIBRAR:
            
            ret, frame = cap.read()
            if not ret: break

            pixel_pos, frame_vis, mascara_vis = detectar_bloco(frame)
            current_flag, flag_ok = fanuc.read_register(REG_FLAG)
            
            if not flag_ok:
                current_flag = 0 

            # Lógica de "Gatilho" (Detecção de Borda de Subida)
            if current_flag == 1 and last_flag_state == 0:
                
                print(f"\n[GATILHO] Pulso detectado (R[{REG_FLAG}] mudou de 0 -> 1)!")
                
                if pixel_pos is None:
                    print("ERRO: Pulso detectado, mas a câmera não encontrou o bloco!")
                    print("Ponto ignorado. Aguardando próximo pulso.")
                
                else:
                    robot_x, x_ok = fanuc.read_register(REG_X)
                    robot_y, y_ok = fanuc.read_register(REG_Y)

                    if not (x_ok and y_ok):
                        print("ERRO: Pulso detectado, mas não foi possível ler R[6] ou R[7].")
                        print("Ponto ignorado. Aguardando próximo pulso.")
                    
                    else:
                        p_camera_list.append(list(pixel_pos))
                        p_robot_list.append([robot_x, robot_y])
                        
                        ponto_num = len(p_camera_list)
                        print(f"--> PONTO {ponto_num} CAPTURADO:")
                        print(f"    Pixel Câmera: ({pixel_pos[0]:.1f}, {pixel_pos[1]:.1f})")
                        print(f"    Posição Robô: ({robot_x}, {robot_y})")

            # Atualiza o estado da flag para a próxima iteração
            last_flag_state = current_flag

            # Exibir UI
            ponto_num_str = len(p_camera_list) + 1
            status_text = f"Aguardando pulso (Ponto {ponto_num_str} de {NUM_PONTOS_PARA_CALIBRAR}) em R[{REG_FLAG}]..."
            cv2.putText(frame_vis, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # Redimensiona a janela de exibição para caber na tela, se necessário
            # (A captura é 1920x1080, mas a exibição pode ser menor)
            frame_display = cv2.resize(frame_vis, (1280, 720), interpolation=cv2.INTER_AREA)
            mascara_display = cv2.resize(mascara_vis, (1280, 720), interpolation=cv2.INTER_NEAREST)

            cv2.imshow("Calibração Automática (Detecção de Pulso)", frame_display)
            cv2.imshow("Máscara", mascara_display)
            
            if cv2.waitKey(1) & 0xFF == 27:
                print("Calibração cancelada pelo usuário.")
                break

    finally:
        # Calcular, Salvar e IMPRIMIR a Homografia
        
        # Altera a condição para ser exata
        if len(p_camera_list) == NUM_PONTOS_PARA_CALIBRAR:
            print(f"\n--- COLETA CONCLUÍDA ({len(p_camera_list)} pontos) ---")
            
            p_camera_np = np.array(p_camera_list, dtype=np.float32)
            p_robot_np = np.array(p_robot_list, dtype=np.float32)
            
            H, _ = cv2.findHomography(p_camera_np, p_robot_np, cv2.RANSAC)
            
            print("Matriz de Homografia (H) calculada:")
            print(H)
            
            np.save("homografia_salva.npy", H)
            print("\nMatriz salva com sucesso em 'homografia_salva.npy'")
            
            # --- IMPRIME OS PONTOS NO FORMATO SOLICITADO ---
            formatar_e_imprimir_pontos(p_camera_list, p_robot_list)
            # --------------------------------------------------

        else:
            print(f"Calibração não concluída. Coletados {len(p_camera_list)} de {NUM_PONTOS_PARA_CALIBRAR} pontos.")
            print("Nenhum arquivo salvo ou formato impresso.")

        # Limpeza
        cap.release()
        cv2.destroyAllWindows()
        fanuc.disconnect()