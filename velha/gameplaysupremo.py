import struct
import cv2
import numpy as np
from pycomm3 import CIPDriver, Services
import sys
import time
import threading
import math
import json

# --- NOMES DOS ARQUIVOS DE CONFIGURAÇÃO ---
NOME_ARQUIVO_PONTOS = "pontos_calibracao.txt"
NOME_ARQUIVO_GRID = "grid_calibracao.txt" # Contém coordenadas em pixels dos 9 centros
# ------------------------------------

# --- Dimensões ---
ORIGINAL_WIDTH = 1920
ORIGINAL_HEIGHT = 1080
DISPLAY_WIDTH = 1280
DISPLAY_HEIGHT = 720
# ------------------------------

# --- Faixas de Cor ---
limite_inferior_azul = np.array([80, 120, 70])
limite_superior_azul = np.array([150, 255, 255])
limite_inferior_vermelho1 = np.array([0, 100, 100])
limite_superior_vermelho1 = np.array([10, 255, 255])
limite_inferior_vermelho2 = np.array([170, 100, 100])
limite_superior_vermelho2 = np.array([180, 255, 255])
# -----------------------------------------------------------

# Tempo de espera antes da limpeza (segundos)
CLEANUP_DELAY_SECONDS = 5.0

# =========================================================
# --- CARREGAR PONTOS DE HOMOGRAFIA DO ARQUIVO ---
# =========================================================
def carregar_pontos_do_arquivo(filename):
    p_camera_list = []
    p_robot_list = []
    try:
        with open(filename, 'r') as f: content = f.read()
    except FileNotFoundError: print(f"ERRO: Arquivo '{filename}' não encontrado."); return None, None
    except Exception as e: print(f"ERRO ao ler '{filename}': {e}"); return None, None
    lendo_camera = False; lendo_robot = False
    for line in content.splitlines():
        line = line.strip()
        if "p_camera = np.array([" in line: lendo_camera = True; lendo_robot = False; continue
        elif "p_robot = np.array([" in line: lendo_camera = False; lendo_robot = True; continue
        elif "], dtype=np.float32)" in line: lendo_camera = False; lendo_robot = False; continue
        if lendo_camera or lendo_robot:
            if line.startswith('[') and ']' in line:
                try:
                    data_str = line.split(']')[0].strip('[')
                    x, y = map(float, data_str.split(','))
                    if lendo_camera: p_camera_list.append([int(round(x)), int(round(y))])
                    elif lendo_robot: p_robot_list.append([int(round(x)), int(round(y))])
                except ValueError as e: print(f"Aviso: Linha inválida: '{line}'. Erro: {e}")
    if not p_camera_list or not p_robot_list or len(p_camera_list) != len(p_robot_list):
        print(f"ERRO: Dados incompletos. Câmera: {len(p_camera_list)}, Robô: {len(p_robot_list)}."); return None, None
    print(f"\n[SUCESSO] Carregados {len(p_camera_list)} pontos de calibração de '{filename}'.")
    return np.array(p_camera_list, dtype=np.float32), np.array(p_robot_list, dtype=np.float32)

# --- Executa o carregamento da Homografia ---
p_camera, p_robot = carregar_pontos_do_arquivo(NOME_ARQUIVO_PONTOS)
if p_camera is None or p_robot is None: sys.exit("ERRO FATAL: Falha ao carregar pontos de calibração.")
H, _ = cv2.findHomography(p_camera, p_robot, cv2.RANSAC)
if H is None: sys.exit("ERRO: Não foi possível calcular a homografia.")
# =========================================================


# =========================================================
# --- CARREGAR CENTROS DA GRADE (PIXELS) DO ARQUIVO ---
# =========================================================
def carregar_centros_grid_pixels(filename):
    try:
        with open(filename, 'r') as f:
            centros_pixels = json.load(f)
        if isinstance(centros_pixels, list) and len(centros_pixels) == 9:
            centros_pixels_tuples = [(int(p[0]), int(p[1])) for p in centros_pixels]
            print(f"[SUCESSO] Carregados {len(centros_pixels_tuples)} centros da grade (pixels) de '{filename}'.")
            return centros_pixels_tuples
        else:
            print(f"[ERRO] Arquivo '{filename}' não contém lista válida de 9 pontos."); return None
    except FileNotFoundError: print(f"[ERRO] Arquivo '{filename}' não encontrado."); return None
    except json.JSONDecodeError: print(f"[ERRO] Arquivo '{filename}' não é JSON válido."); return None
    except Exception as e: print(f"[ERRO] Falha inesperada ao ler '{filename}': {e}."); return None
# =========================================================


# --- CLASSE DE COMUNICAÇÃO CIP ---
class FanucTicTacToeAndClean:
    def __init__(self, ip_robot, cam_index=0):
        self.ip = ip_robot; self.cam_index = cam_index
        self.connected = False; self.plc = None
        self.grid_centers_robo = [] # Coordenadas do Robô
        self.grid_centers_pixel = None # Coordenadas em Pixel
        self.grid_min_x = None; self.grid_max_x = None; self.grid_min_y = None; self.grid_max_y = None
        self.USER_PLAYER_CHAR = 'X'; self.ROBOT_PLAYER_CHAR = 'O'
        self.game_board = [' '] * 9; self.game_over = False; self.winner = None
        self.robot_is_busy = False # Flag baseada em R[5]
        self.cleanup_mode = False; self.last_sent_coords = {}
        self.waiting_for_cleanup_start = False; self.game_end_time = None

    def connect(self):
        try:
            self.plc = CIPDriver(self.ip); self.plc.open(); self.connected = True
            print(f"Conectado a {self.ip}"); return True
        except Exception as e: print(f"Erro conexão: {e}"); self.connected = False; return False

    def disconnect(self):
        if self.connected:
            print("Garantindo R[5]=0 e R[9]=0...");
            self.write_cip_explicit_register(5, 0); self.write_cip_explicit_register(9, 0); time.sleep(0.1)
            try: self.plc.close(); print("Desconectado.")
            except Exception as e: print(f"Erro desconectar: {e}")
            finally: self.connected = False

    def write_cip_explicit_register(self, register_index, value):
        if not self.connected: return False
        try:
            int_value = int(round(float(value)))
            data_payload = int_value.to_bytes(4, 'little', signed=True)
            response = self.plc.generic_message(
                service=Services.set_attribute_single, class_code=0x6B, instance=0x01,
                attribute=register_index, request_data=data_payload, connected=True)
            if response.error: print(f"ERRO ESCRITA R[{register_index}]: {response.error}"); return False
            return True
        except Exception as e: print(f"ERRO EXCEÇÃO ESCRITA R[{register_index}]: {e}"); return False

    def read_register(self, register_index):
        if not self.connected: return None, False
        try:
            response = self.plc.generic_message(
                service=Services.get_attribute_single, class_code=0x6B, instance=0x01,
                attribute=register_index, connected=True)
            if response.error: return None, False
            value = int.from_bytes(response.value, 'little', signed=True)
            return value, True
        except Exception as e: return None, False # Não printa exceção aqui

    def aplicar_homografia(self, x_pixel, y_pixel):
        global H; pt = np.array([[[x_pixel, y_pixel]]], dtype=np.float32); trans = cv2.perspectiveTransform(pt, H); return trans[0][0][0], trans[0][0][1]

    def load_grid_and_boundaries(self):
        global NOME_ARQUIVO_GRID
        print("\nCarregando centros da grade (pixels)..."); centros_pixels = carregar_centros_grid_pixels(NOME_ARQUIVO_GRID)
        if centros_pixels:
            self.grid_centers_pixel = centros_pixels; self.grid_centers_robo = []
            try:
                for (cx, cy) in centros_pixels: self.grid_centers_robo.append(self.aplicar_homografia(cx, cy))
                if len(self.grid_centers_robo) == 9:
                    sx=abs(self.grid_centers_robo[8][0]-self.grid_centers_robo[0][0])/2.0; sy=abs(self.grid_centers_robo[8][1]-self.grid_centers_robo[0][1])/2.0
                    mx=sx/1.5; my=sy/1.5; cx1=self.grid_centers_robo[0][0]; cy1=self.grid_centers_robo[0][1]; cx9=self.grid_centers_robo[8][0]; cy9=self.grid_centers_robo[8][1]
                    self.grid_min_x=min(cx1,cx9)-mx; self.grid_max_x=max(cx1,cx9)+mx; self.grid_min_y=min(cy1,cy9)-my; self.grid_max_y=max(cy1,cy9)+my
                    print("SUCESSO: Grade carregada."); return True
                else: print("ERRO: 9 pontos não carregados."); self._reset_state(); return False
            except Exception as e: print(f"ERRO homografia: {e}"); self._reset_state(); return False
        else: print("FALHA ao carregar centros."); self._reset_state(); return False

    def _reset_state(self):
        self.grid_centers_robo = []; self.grid_centers_pixel = None; self.grid_min_x=None; self.grid_max_x=None; self.grid_min_y=None; self.grid_max_y=None
        self.game_board = [' ']*9; self.game_over=False; self.winner=None; self.robot_is_busy=False
        self.cleanup_mode=False; self.last_sent_coords={}; self.waiting_for_cleanup_start = False; self.game_end_time = None
        if self.connected: self.write_cip_explicit_register(9, 0); self.write_cip_explicit_register(5, 0)

    # --- (JOGO DA VELHA - Lógica) ---
    def print_board(self, board): print(" ESTADO ATUAL:"); print(f" |{board[0]}|{board[1]}|{board[2]}|");print(" |-|-|-|");print(f" |{board[3]}|{board[4]}|{board[5]}|");print(" |-|-|-|");print(f" |{board[6]}|{board[7]}|{board[8]}|")
    def is_winner(self, board, player): win=[[0,1,2],[3,4,5],[6,7,8],[0,3,6],[1,4,7],[2,5,8],[0,4,8],[2,4,6]]; return any(all(board[i]==player for i in c) for c in win)
    def is_board_full(self, board): return ' ' not in board
    def _can_win(self, board, player): opp=self.ROBOT_PLAYER_CHAR if player==self.USER_PLAYER_CHAR else self.USER_PLAYER_CHAR; win=[[0,1,2],[3,4,5],[6,7,8],[0,3,6],[1,4,7],[2,5,8],[0,4,8],[2,4,6]]; return any(opp not in [board[i] for i in c] for c in win)
    def check_game_over(self, board):
        if self.is_winner(board, self.USER_PLAYER_CHAR): return self.USER_PLAYER_CHAR
        if self.is_winner(board, self.ROBOT_PLAYER_CHAR): return self.ROBOT_PLAYER_CHAR
        uc=self._can_win(board,self.USER_PLAYER_CHAR); rc=self._can_win(board,self.ROBOT_PLAYER_CHAR)
        if not uc and not rc: return 'Draw' # Empate por bloqueio
        if self.is_board_full(board): return 'Draw' # Empate por preenchimento
        return None
    def minimax(self, board, depth, is_maximizing):
        winner = self.check_game_over(board);
        if winner == self.ROBOT_PLAYER_CHAR: return 10 - depth
        if winner == self.USER_PLAYER_CHAR: return depth - 10
        if winner == 'Draw': return 0
        score_func = max if is_maximizing else min; best_score = -math.inf if is_maximizing else math.inf
        player = self.ROBOT_PLAYER_CHAR if is_maximizing else self.USER_PLAYER_CHAR
        for i in range(9):
            if board[i] == ' ': board[i] = player; score = self.minimax(board, depth + 1, not is_maximizing); board[i] = ' '; best_score = score_func(score, best_score)
        return best_score

    # --- (CORRIGIDO) find_best_move ---
    def find_best_move(self, board):
        best_score = -math.inf; best_move = -1
        for i in range(9):
            if board[i] == ' ':
                board[i] = self.ROBOT_PLAYER_CHAR
                score = self.minimax(board, 0, False)
                board[i] = ' '
                # Linha corrigida (indentação)
                if score > best_score: best_score = score; best_move = i
        return best_move
    # --- Fim da Correção ---

    # --- Função para detectar blocos (Usada na Limpeza e visualização) ---
    def _detect_all_blocks(self, frame, draw_contours=True):
        blocos = []; hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        ids_cores = {1: (limite_inferior_azul, limite_superior_azul), 2: (limite_inferior_vermelho1, limite_superior_vermelho1, limite_inferior_vermelho2, limite_superior_vermelho2)}
        cores_desenho = {1: (255,0,0), 2: (0,0,255)}
        for cid, lims in ids_cores.items():
            if cid==1: mask=cv2.inRange(hsv, lims[0], lims[1])
            else: m1=cv2.inRange(hsv, lims[0], lims[1]); m2=cv2.inRange(hsv, lims[2], lims[3]); mask = cv2.bitwise_or(m1, m2)
            mask = cv2.dilate(cv2.erode(mask, None, iterations=2), None, iterations=2)
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                if cv2.contourArea(c) > 100:
                    rect=cv2.minAreaRect(c);
                    if draw_contours: box=np.intp(cv2.boxPoints(rect)); cv2.drawContours(frame, [box], 0, cores_desenho[cid], 2)
                    (xp, yp), _, _ = rect; xr, yr = self.aplicar_homografia(xp, yp)
                    blocos.append({'x_robo': xr, 'y_robo': yr, 'cor_id': cid})
        return blocos

    # --- Função para iniciar a limpeza ---
    def _start_cleanup_sequence(self):
        print("\n--- INICIANDO LIMPEZA ---"); self.cleanup_mode = True; self.last_sent_coords = {}
        if self.connected: print("Ligando R[9]=1"); self.write_cip_explicit_register(9, 1)

    # --- Callback do Mouse ---
    def handle_click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            sx=ORIGINAL_WIDTH/DISPLAY_WIDTH; sy=ORIGINAL_HEIGHT/DISPLAY_HEIGHT; ox=int(x*sx); oy=int(y*sy); print(f"\nClique ({x},{y})->({ox},{oy})")
            if self.cleanup_mode: print("Limpeza."); return;
            if self.game_over: print("Jogo acabou."); return
            if not self.grid_centers_pixel: print("Grade não calibrada."); return
            if self.robot_is_busy: print("Aguarde robô."); return
            if not self.connected: print("Robô desconectado."); return
            if self.game_board.count('X') > self.game_board.count('O'): print("Não é sua vez."); return

            click_pos = np.array([ox, oy]); dists = [np.linalg.norm(click_pos-np.array(c)) for c in self.grid_centers_pixel]; idx = np.argmin(dists)
            if dists[idx] > 150: print(f"Clique longe (Dist: {dists[idx]:.1f})"); return

            print(f"Célula: {idx + 1}")
            if self.game_board[idx] == ' ':
                print("Enviando jogada USR..."); (ux, uy) = self.grid_centers_robo[idx]
                sX=self.write_cip_explicit_register(1, ux); sY=self.write_cip_explicit_register(2, uy)
                if not (sX and sY): print("Falha envio coords (USR)."); return
                print(f"ENVIO USR OK: X={ux:.1f}, Y={uy:.1f}"); sF = self.write_cip_explicit_register(5, 1);
                if not sF: print("Falha LIGAR R[5] (USR)!"); return
                print("R[5] LIGADO (USR)..."); self.robot_is_busy = True
                self.game_board[idx] = self.USER_PLAYER_CHAR; self.print_board(self.game_board)
                self.winner = self.check_game_over(self.game_board)
                if self.winner: self.game_over = True; print(f"--- FIM DE JOGO! Result: {self.winner} ---")
            else: print("Célula ocupada.")


    # --- LOOP PRINCIPAL ---
    def run_vision_and_send(self):
        global ORIGINAL_WIDTH, ORIGINAL_HEIGHT
        if not self.load_grid_and_boundaries(): print("AVISO: Falha ao carregar grade. 'g'.")

        cap = cv2.VideoCapture(self.cam_index)
        if not cap.isOpened(): print(f"Erro câmera {self.cam_index}."); return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, ORIGINAL_WIDTH); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, ORIGINAL_HEIGHT)
        aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)); ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"Resolução: {aw}x{ah}")
        if aw!=ORIGINAL_WIDTH or ah!=ORIGINAL_HEIGHT: print("AVISO: Resolução diferente!"); ORIGINAL_WIDTH=aw; ORIGINAL_HEIGHT=ah

        window_name = 'Jogo da Velha & Limpeza Automática'; cv2.namedWindow(window_name)
        cv2.setMouseCallback(window_name, self.handle_click)
        print("\n--- JOGO DA VELHA & LIMPEZA ---"); print("'g': Grade | 'r': Reset | 'ESC': Sair | CLIQUE: Jogar")
        print("Limpeza automática no FIM DE JOGO."); print("-----------------------------")

        centros_grid_pixel = self.grid_centers_pixel; frame = None

        while True:
            ret, current_frame_read = cap.read();
            if ret: frame = current_frame_read
            elif frame is None: print("Erro frame."); break
            frame_display = frame.copy()

            robot_finished_now = False
            # --- Leitura periódica de R[5] ---
            if self.connected and self.robot_is_busy:
                flag_value, read_ok = self.read_register(5)
                if read_ok and flag_value == 0:
                    print("(Loop) R[5] = 0. Robô liberado.")
                    self.robot_is_busy = False; robot_finished_now = True; self.last_sent_coords = {}
                # elif not read_ok: print("(Loop) Aviso: Falha leitura R[5]...") # Opcional

            # --- Lógica Principal ---
            if not self.robot_is_busy:

                # --- Inicia Limpeza se Jogo Acabou E Robô Terminou ---
                if self.game_over and robot_finished_now and not self.cleanup_mode:
                     print(f"Jogo terminou ({self.winner}). Iniciando limpeza...")
                     self._start_cleanup_sequence()

                # --- Processa Limpeza ---
                elif self.cleanup_mode:
                     current_pieces = []
                     if self.grid_min_x is not None:
                         all_detected = self._detect_all_blocks(frame.copy(), draw_contours=False)
                         for piece in all_detected:
                            px, py = piece['x_robo'], piece['y_robo']
                            if (self.grid_min_x <= px <= self.grid_max_x and self.grid_min_y <= py <= self.grid_max_y): current_pieces.append(piece)
                     print(f"Limpando... Peças restantes: {len(current_pieces)}")

                     if current_pieces:
                         next_piece = current_pieces[0]; px=next_piece['x_robo']; py=next_piece['y_robo']; pcid=next_piece['cor_id']; coord_key = f"{px:.0f}_{py:.0f}"
                         if coord_key == self.last_sent_coords.get("key"): print("Coords iguais..."); time.sleep(0.2)
                         else:
                              print(f"Enviando peça {('Azul' if pcid==1 else 'Vermelha')} p/ limpar...");
                              sX=self.write_cip_explicit_register(1,px); sY=self.write_cip_explicit_register(2,py); sC=self.write_cip_explicit_register(8,pcid); sM=self.write_cip_explicit_register(9,1)
                              if sX and sY and sC and sM:
                                   sF=self.write_cip_explicit_register(5,1);
                                   if sF: self.robot_is_busy=True; self.last_sent_coords = {"key": coord_key}
                                   else: print("Falha R[5]!")
                              else: print("Falha R[1/2/8/9].")
                     else: # Fim limpeza
                         print("Limpeza concluída."); self.cleanup_mode = False; self.last_sent_coords = {}
                         if self.connected: print("Desligando R[9]..."); self.write_cip_explicit_register(9, 0)

                # --- Processa Jogada do Robô ---
                elif robot_finished_now and not self.game_over:
                    if self.game_board.count('X') > self.game_board.count('O'):
                        print("Calculando/enviando jogada ROBÔ..."); idx = self.find_best_move(self.game_board)
                        if idx != -1:
                            (px_r, py_r) = self.grid_centers_robo[idx]; sX_r=self.write_cip_explicit_register(1,px_r); sY_r=self.write_cip_explicit_register(2,py_r)
                            if (sX_r and sY_r):
                                print(f"ENVIO ROBÔ OK: Célula {idx+1}"); sF_r = self.write_cip_explicit_register(5, 1);
                                if sF_r:
                                    print("R[5] LIGADO (Robô)..."); self.robot_is_busy = True; self.game_board[idx] = self.ROBOT_PLAYER_CHAR; self.print_board(self.game_board)
                                    self.winner = self.check_game_over(self.game_board)
                                    if self.winner: self.game_over = True; print(f"--- FIM DE JOGO! Result: {self.winner} ---")
                                else: print("Falha R[5] (Robô)!")
                            else: print("Falha envio coords (Robô).")
                        else: # Empate ou erro
                             self.winner = self.check_game_over(self.game_board)
                             if self.winner == 'Draw': self.game_over = True; print("--- JOGO EMPATADO ---")
                             else: print("ERRO: Minimax não achou jogada.")
            # --- Fim Lógica ---

            # --- Desenhos ---
            if self.grid_centers_pixel:
                for i, (cx, cy) in enumerate(self.grid_centers_pixel):
                    cv2.putText(frame_display, str(i+1), (cx-10, cy+10), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255,255,255), 3)
                    p_char = self.game_board[i];
                    if p_char != ' ': color = (255,100,100) if p_char=='X' else (100,100,255); (tw,th),_ = cv2.getTextSize(p_char, cv2.FONT_HERSHEY_SIMPLEX, 2.5, 5); tx=cx-tw//2; ty=cy+th//2; cv2.putText(frame_display, p_char, (tx,ty), cv2.FONT_HERSHEY_SIMPLEX, 2.5, color, 5)
            # Desenha blocos detectados
            self._detect_all_blocks(frame_display, draw_contours=True)

            # --- HUD ---
            current_pieces_on_board_count = "?"
            if self.grid_min_x is not None:
                all_detected_hud = self._detect_all_blocks(frame.copy(), draw_contours=False); on_board_hud = [p for p in all_detected_hud if (self.grid_min_x <= p['x_robo'] <= self.grid_max_x and self.grid_min_y <= p['y_robo'] <= self.grid_max_y)]; current_pieces_on_board_count = len(on_board_hud)

            if self.cleanup_mode: status_msg = f"LIMPANDO... [{current_pieces_on_board_count} detec.]"; color = (255,165,0)
            elif self.game_over: status_msg = f"FIM: {self.winner}. Aguardando R[5]=0 p/ limpar..."; color = (0, 200, 200)
            elif not self.grid_centers_robo: status_msg = "GRADE NAO CALIBRADA. 'g'."; color = (0,0,255)
            elif self.robot_is_busy: status_msg = "AGUARDANDO ROBO..."; color = (0,165,255)
            else: status_msg = "Sua vez. CLIQUE."; color = (0,255,0)
            cv2.putText(frame_display, status_msg, (15, 75), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 3)
            status_conn = "CONECTADO" if self.connected else "DESCONECTADO"; color_conn = (0,255,255) if self.connected else (0,0,255); cv2.putText(frame_display, f"Status: {status_conn}", (15, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color_conn, 2)

            # --- Exibe ---
            display_frame_resized = cv2.resize(frame_display, (DISPLAY_WIDTH, DISPLAY_HEIGHT)); cv2.imshow(window_name, display_frame_resized)

            # --- Teclas ---
            key = cv2.waitKey(50) & 0xFF
            if key == 27: break # ESC
            if key == ord('g'):
                if self.robot_is_busy or self.cleanup_mode: print("Aguarde..."); continue
                print("\n--- CARREGANDO GRADE ---"); centros_grid_pixel = self.load_grid_and_boundaries();
                if centros_grid_pixel: self.print_board(self.game_board)
            if key == ord('r'):
                if self.robot_is_busy or self.cleanup_mode: print("Aguarde..."); continue
                print("\n--- JOGO RESETADO ---"); self._reset_state(); self.print_board(self.game_board); centros_grid_pixel = None

        cap.release(); cv2.destroyAllWindows()

# --- PONTO DE ENTRADA ---
if __name__ == "__main__":
    ip_robot = "192.168.1.100"; camera_index = 1
    game = FanucTicTacToeAndClean(ip_robot, camera_index)
    if game.connect(): game.run_vision_and_send(); game.disconnect()
    else: print("Rodando só visão."); game.run_vision_and_send()