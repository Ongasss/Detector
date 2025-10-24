import cv2
import numpy as np
import json
import sys
import math

# --- CONFIGURAÇÕES ---
camera_index = 1
desired_width = 1920
desired_height = 1080
NOME_ARQUIVO_GRID = "grid_calibracao.txt"
# ---------------------

def nothing(x):
    pass

# Abrir a câmera
cap = cv2.VideoCapture(camera_index)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)
actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
image_center_x = actual_width // 2
image_center_y = actual_height // 2
print(f"Centro da imagem (pixels): ({image_center_x}, {image_center_y})")


# Criar a janela de trackbars
cv2.namedWindow("Trackbars")

# --- VALORES INICIAIS ---
cv2.createTrackbar("BlockSize (x2)+3", "Trackbars", 39, 100, nothing)
cv2.createTrackbar("C (val-25)", "Trackbars", 50, 50, nothing)
cv2.createTrackbar("MIN_AREA_x100", "Trackbars", 30, 1000, nothing) # Começa em 3000
cv2.createTrackbar("MAX_AREA_x100", "Trackbars", 3000, 4000, nothing) # Começa em 300000
cv2.createTrackbar("TapeThickness_px", "Trackbars", 20, 100, nothing)

# Kernel
kernel = np.ones((7, 7), np.uint8)

print("OBJETIVO: Ajuste os sliders ate a tela 'Resultado' mostrar 'Grade Estimada (Centro OK)' (em VERDE).")
print("DICA: Foque em isolar APENAS o quadrado do meio na janela 'Mascara'.")
print("DICA: Ajuste 'TapeThickness_px' para a espessura da sua fita branca.")
print("Pressione ESC para sair e SALVAR os CENTROS ESTIMADOS (em pixels).") # <--- Mensagem alterada

# Inicializa as variáveis
block_size, c, min_area, max_area, tape_thickness = 81, 25, 3000, 300000, 20
grid_estimated_on_exit = False
final_estimated_centers_pixel = [] # <--- NOVO: Para guardar os centros a serem salvos

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Erro de camera")
            break

        frame_desenho = frame.copy()

        # --- 1. PRE-PROCESSAMENTO ---
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # --- 2. PEGAR VALORES DOS TRACKBARS ---
        block_size_val = cv2.getTrackbarPos("BlockSize (x2)+3", "Trackbars")
        block_size = (block_size_val * 2) + 3
        c_val = cv2.getTrackbarPos("C (val-25)", "Trackbars")
        c = c_val - 25
        min_area = cv2.getTrackbarPos("MIN_AREA_x100", "Trackbars") * 100
        max_area = cv2.getTrackbarPos("MAX_AREA_x100", "Trackbars") * 100
        tape_thickness = cv2.getTrackbarPos("TapeThickness_px", "Trackbars")

        # --- 3. APLICAR THRESHOLD ADAPTATIVO (LÓGICA INVERTIDA) ---
        mascara_com_fita = cv2.adaptiveThreshold(
            gray_blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY, block_size, c
        )
        mascara = cv2.bitwise_not(mascara_com_fita)

        # --- 4. LIMPAR MASCARA ---
        mascara_limpa = cv2.morphologyEx(mascara, cv2.MORPH_CLOSE, kernel, iterations=3)
        mascara_limpa = cv2.morphologyEx(mascara_limpa, cv2.MORPH_OPEN, kernel, iterations=1)

        # --- 5. ENCONTRAR E FILTRAR CONTORNOS ---
        contornos, _ = cv2.findContours(mascara_limpa, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        celulas_filtradas = []
        for cont in contornos:
            area = cv2.contourArea(cont)
            if min_area < area < max_area:
                celulas_filtradas.append(cont)

        # --- 6. ENCONTRAR O QUADRADO MAIS CENTRAL ---
        central_square_contour = None
        min_dist_to_center = float('inf')
        estimated_centers_pixel = [] # Zera a cada loop
        grid_estimated_on_exit = False # Reseta a cada loop

        if len(celulas_filtradas) > 0:
            for contorno_celula in celulas_filtradas:
                M = cv2.moments(contorno_celula)
                if M["m00"] > 1e-5:
                    cx_pixel = int(M["m10"] / M["m00"])
                    cy_pixel = int(M["m01"] / M["m00"])
                    dist = math.sqrt((cx_pixel - image_center_x)**2 + (cy_pixel - image_center_y)**2)
                    if dist < min_dist_to_center:
                        min_dist_to_center = dist
                        central_square_contour = contorno_celula

        # --- 7. ESTIMAR GRADE E DESENHAR RESULTADO ---
        if central_square_contour is not None:
            grid_estimated_on_exit = True
            cor_texto = (0, 255, 0) # Verde
            status_text = "Grade Estimada (Centro OK)"

            cv2.drawContours(frame_desenho, [central_square_contour], -1, (0, 255, 0), 3)

            M = cv2.moments(central_square_contour)
            center5_x_pix = int(M["m10"] / (M["m00"] + 1e-5))
            center5_y_pix = int(M["m01"] / (M["m00"] + 1e-5))
            rect = cv2.minAreaRect(central_square_contour)
            (w, h) = rect[1]
            square_size = (w + h) / 2.0
            step = square_size + tape_thickness

            print(f" -> Centro (Px): ({center5_x_pix}, {center5_y_pix}), Tam. Médio: {square_size:.1f}, Fita: {tape_thickness}, Passo: {step:.1f}")

            estimated_centers_pixel = [ # Recalcula os centros estimados
                (int(round(center5_x_pix - step)), int(round(center5_y_pix - step))),
                (int(round(center5_x_pix)),      int(round(center5_y_pix - step))),
                (int(round(center5_x_pix + step)), int(round(center5_y_pix - step))),
                (int(round(center5_x_pix - step)), int(round(center5_y_pix))),
                (center5_x_pix, center5_y_pix),
                (int(round(center5_x_pix + step)), int(round(center5_y_pix))),
                (int(round(center5_x_pix - step)), int(round(center5_y_pix + step))),
                (int(round(center5_x_pix)),      int(round(center5_y_pix + step))),
                (int(round(center5_x_pix + step)), int(round(center5_y_pix + step)))
            ]
            # --- FIM DA ESTIMAÇÃO ---

            # Guarda os centros estimados para salvar depois
            final_estimated_centers_pixel = estimated_centers_pixel

            # Desenha os 9 centros estimados
            for i, (cx, cy) in enumerate(estimated_centers_pixel):
                 cv2.circle(frame_desenho, (cx, cy), 15, (255, 0, 0), 3) # Círculo Azul
                 cv2.putText(frame_desenho, str(i+1), (cx-10, cy+10), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)

        else:
            cor_texto = (0, 0, 255) # Vermelho
            status_text = "Procurando Quadrado Central..."
            final_estimated_centers_pixel = [] # Limpa se não encontrou
            if celulas_filtradas:
                cv2.drawContours(frame_desenho, celulas_filtradas, -1, (0, 0, 255), 3)

        # Escreve o status e os valores na tela
        cv2.putText(frame_desenho, status_text, (50, 70), cv2.FONT_HERSHEY_SIMPLEX, 1.5, cor_texto, 3)
        cv2.putText(frame_desenho, f"Min Area: {min_area}", (50, 130), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(frame_desenho, f"Max Area: {max_area}", (50, 170), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(frame_desenho, f"BlockSize: {block_size}", (50, 210), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(frame_desenho, f"C: {c}", (50, 250), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(frame_desenho, f"Tape Thick (px): {tape_thickness}", (50, 290), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # --- 8. EXIBIÇÃO ---
        frame_display = cv2.resize(frame_desenho, (1280, 720))
        mascara_display = cv2.resize(mascara_limpa, (1280, 720))

        cv2.imshow("Calibrador - Estima Grade e Salva Centros", frame_display) # Nome da janela alterado
        cv2.imshow("Mascara (Celulas)", mascara_display)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC
            print("\nESC pressionado, salvando valores...")
            break

finally:
    # --- (MODIFICADO) 9. SALVAR CENTROS ESTIMADOS (em pixels) ---
    print(f"Salvando centros estimados da grade em '{NOME_ARQUIVO_GRID}'...")

    if grid_estimated_on_exit and final_estimated_centers_pixel:
        # Converte tuplas (x, y) para listas [x, y] para o JSON
        centers_to_save = [[int(x), int(y)] for x, y in final_estimated_centers_pixel]

        try:
            with open(NOME_ARQUIVO_GRID, 'w') as f:
                json.dump(centers_to_save, f, indent=4) # Salva a lista de listas
            print("[SUCESSO] Centros (pixels) da grade salvos.")
        except Exception as e:
            print(f"[ERRO] Falha ao salvar '{NOME_ARQUIVO_GRID}': {e}")
    else:
         print("[AVISO] A grade não estava sendo estimada no momento de sair. Nada foi salvo.")

    cap.release()
    cv2.destroyAllWindows()
    print("Calibração encerrada.")