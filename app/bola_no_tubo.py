#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Bola no Tubo - PC GUI (Python + Tkinter + Matplotlib + PySerial)

Funcionalidades:
- Selecionar e conectar à porta serial (COM/tty) criada pelo HC-05 (115200-8N1).
- Receber pacotes do microcontrolador conforme o protocolo (big-endian):
  TX micro->PC (15 bytes por quadro, a cada ~100 ms):
    A: modo (1 byte)
    B: setpoint altura (2 bytes, mm)
    C: altura medida (2 bytes, mm)
    D: tempo de voo médio (2 bytes, contagem timer)
    E: temperatura (2 bytes, décimos de °C)
    F: setpoint posição válvula (2 bytes, passos)
    G: posição atual da válvula (2 bytes, passos)
    H: ciclo útil da ventoinha (2 bytes, 0..1023)
- Exibir valores numéricos: altura, temperatura, setpoint altura, duty ventoinha, posição da válvula.
- Exibir 3 gráficos empilhados (mesmo eixo X/tempo): setpoint de altura, ciclo útil da ventoinha, posição da válvula.
- Permitir enviar comandos PC->micro (7 bytes, big-endian):
    A: modo (1 byte) [0=Manual, 1=Ventoinha, 2=Válvula, 3=Reset]
    B: setpoint altura (2 bytes, mm)
    C: setpoint posição válvula (2 bytes, passos)
    D: setpoint ciclo útil ventoinha (2 bytes, 0..1023)
- Botão de RESET envia modo=3 e zera demais campos.

Requisitos:
  pip install pyserial matplotlib

"""
import sys
import threading
import time
import struct
from collections import deque
from typing import Optional

import serial
import serial.tools.list_ports

import tkinter as tk
from tkinter import ttk, messagebox

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# -------------------------------
# Parâmetros do protocolo/serial
# -------------------------------
BAUDRATE = 115200
BYTES_PER_FRAME_RX = 15  # micro -> PC
BYTES_PER_FRAME_TX = 7   # PC -> micro
READ_TIMEOUT = 0.05       # s, leitura não bloqueante
PLOT_WINDOW_SECONDS = 60  # janela de plot (últimos N segundos)
HEIGHT_MAX_MM = 500
# Conversões p/ % (ajuste conforme o Roteiro)
MAX_DUTY_RAW = 1023   # duty máximo do micro (ex.: 10 bits)
MAX_VALVE_STEPS = 420 # passos máximos da válvula
FRAME_GAP_S = 0.040    # 40 ms para delimitar quadro RX por timeout

# Campos do quadro RX (micro->pc), todos big-endian
# A: modo (1B), B..H: unsigned short (2B)
# Índices de bytes (para referência, mas usaremos struct.unpack)
# [0]=A, [1:3]=B, [3:5]=C, [5:7]=D, [7:9]=E, [9:11]=F, [11:13]=G, [13:15]=H

# -------------------------------
# Utilidades
# -------------------------------
def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    return [p.device for p in ports]

def be_u16(b):
    """Decode 2 bytes big-endian -> unsigned short"""
    return struct.unpack('>H', b)[0]

def clamp(val, lo, hi):
    return max(lo, min(hi, val))

# -------------------------------
# Classe principal da GUI
# -------------------------------
class BolaNoTuboApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Bola no Tubo")
        self.geometry("1100x800")

        # Estado serial
        self.ser: Optional[serial.Serial] = None
        self.reader_thread = None
        self.reader_alive = False
        self.buffer = bytearray()

        # Dados para exibição
        self.mode_var = tk.IntVar(value=0)
        self.sp_height_var = tk.IntVar(value=0)      # mm
        self.sp_valve_var = tk.IntVar(value=0)       # passos
        self.sp_duty_var = tk.IntVar(value=0)        # 0..1023

        self.rx_mode = tk.IntVar(value=0)
        self.rx_height = tk.IntVar(value=0)          # mm
        self.rx_height_sp = tk.IntVar(value=0)       # mm (B)
        self.rx_tof = tk.IntVar(value=0)             # contagem
        self.rx_temp_x10 = tk.IntVar(value=0)        # décimos de °C
        self.rx_valve_sp = tk.IntVar(value=0)        # passos
        self.rx_valve_pos = tk.IntVar(value=0)       # passos
        self.rx_duty = tk.IntVar(value=0)            # 0..1023

        # Histórico para gráficos (deques com (timestamp, value))
        self.t_hist = deque()
        self.sp_height_hist = deque()
        self.meas_height_hist = deque()
        self.duty_pct_hist = deque()
        self.valve_pct_hist = deque()

        self._build_ui()
        self._schedule_plot_update()

    # -------------------------------
    # Construção da UI
    # -------------------------------
    def _build_ui(self):
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)

        top = ttk.Frame(self)
        top.grid(row=0, column=0, sticky="ew", padx=10, pady=6)
        top.columnconfigure(5, weight=1)

        # Portas seriais
        ttk.Label(top, text="Porta:").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(top, values=list_serial_ports(), width=20, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=4)
        ttk.Button(top, text="Atualizar", command=self._refresh_ports).grid(row=0, column=2, padx=4)
        self.connect_btn = ttk.Button(top, text="Conectar", command=self._toggle_connection)
        self.connect_btn.grid(row=0, column=3, padx=4)

        # Modo + comandos
        ttk.Label(top, text="Modo:").grid(row=0, column=4, sticky="e", padx=(20,4))
        self.mode_combo = ttk.Combobox(top, state="readonly", width=16,
                                       values=[
                                           "0 - Manual",
                                           "1 - Ventoinha (PI/PID)",
                                           "2 - Válvula (PI/PID)",
                                           "3 - Reset",
                                       ])
        self.mode_combo.current(0)
        self.mode_combo.grid(row=0, column=5, sticky="w")
        ttk.Button(top, text="Enviar Comando", command=self._send_current_command).grid(row=0, column=6, padx=6)
        ttk.Button(top, text="RESET", command=self._send_reset).grid(row=0, column=7, padx=6)

        # Linha de entradas (setpoints)
        sp = ttk.LabelFrame(self, text="Setpoints / Comandos")
        sp.grid(row=1, column=0, sticky="ew", padx=10, pady=6)
        for i in range(8):
            sp.columnconfigure(i, weight=1)

        ttk.Label(sp, text="SP Altura (mm):").grid(row=0, column=0, sticky="e")
        self.sp_height_entry = ttk.Spinbox(sp, from_=0, to=500, increment=1, textvariable=self.sp_height_var, width=8)
        self.sp_height_entry.grid(row=0, column=1, sticky="w", padx=4)

        ttk.Label(sp, text="SP Válvula (%):").grid(row=0, column=2, sticky="e")
        self.sp_valve_entry = ttk.Spinbox(sp, from_=0, to=100, increment=1, textvariable=self.sp_valve_var, width=8)
        self.sp_valve_entry.grid(row=0, column=3, sticky="w", padx=4)

        ttk.Label(sp, text="SP Duty Ventoinha (%):").grid(row=0, column=4, sticky="e")
        self.sp_duty_entry = ttk.Spinbox(sp, from_=0, to=100, increment=1, textvariable=self.sp_duty_var, width=8)
        self.sp_duty_entry.grid(row=0, column=5, sticky="w", padx=4)

        ttk.Button(sp, text="Enviar", command=self._send_current_command).grid(row=0, column=6, padx=8)

        # Linha de valores recebidos
        rx = ttk.LabelFrame(self, text="Recebidos do Micro")
        rx.grid(row=2, column=0, sticky="ew", padx=10, pady=6)
        for i in range(10):
            rx.columnconfigure(i, weight=1)

        ttk.Label(rx, text="Modo RX:").grid(row=0, column=0, sticky="e")
        ttk.Label(rx, textvariable=self.rx_mode).grid(row=0, column=1, sticky="w")

        ttk.Label(rx, text="SP Altura RX (mm):").grid(row=0, column=2, sticky="e")
        ttk.Label(rx, textvariable=self.rx_height_sp).grid(row=0, column=3, sticky="w")

        ttk.Label(rx, text="Altura (mm):").grid(row=0, column=4, sticky="e")
        ttk.Label(rx, textvariable=self.rx_height).grid(row=0, column=5, sticky="w")

        ttk.Label(rx, text="ToF médio:").grid(row=0, column=6, sticky="e")
        ttk.Label(rx, textvariable=self.rx_tof).grid(row=0, column=7, sticky="w")

        ttk.Label(rx, text="Temperatura (°C):").grid(row=0, column=8, sticky="e")
        self.rx_temp_degC = tk.StringVar(value="0.0")
        ttk.Label(rx, textvariable=self.rx_temp_degC).grid(row=0, column=9, sticky="w")

        ttk.Label(rx, text="SP Válvula (passos):").grid(row=1, column=0, sticky="e")
        ttk.Label(rx, textvariable=self.rx_valve_sp).grid(row=1, column=1, sticky="w")

        ttk.Label(rx, text="Posição Válvula (passos):").grid(row=1, column=2, sticky="e")
        ttk.Label(rx, textvariable=self.rx_valve_pos).grid(row=1, column=3, sticky="w")

        ttk.Label(rx, text="Duty Vent. (0..1023):").grid(row=1, column=4, sticky="e")
        ttk.Label(rx, textvariable=self.rx_duty).grid(row=1, column=5, sticky="w")

        # Gráficos
        plots = ttk.LabelFrame(self, text="Gráficos")
        plots.grid(row=3, column=0, sticky="nsew", padx=10, pady=6)
        plots.rowconfigure(1, weight=1)
        plots.columnconfigure(0, weight=1)

        # Controle da janela de tempo
        ctrl = ttk.Frame(plots)
        ctrl.grid(row=0, column=0, sticky="ew", pady=(4,4))
        ttk.Label(ctrl, text="Janela (s):").grid(row=0, column=0, sticky="w")
        self.window_seconds_var = tk.IntVar(value=PLOT_WINDOW_SECONDS)
        self.window_seconds = PLOT_WINDOW_SECONDS
        wnd = ttk.Spinbox(ctrl, from_=5, to=600, increment=5, textvariable=self.window_seconds_var, width=6, command=self._apply_window_seconds)
        wnd.grid(row=0, column=1, padx=6)
        ttk.Button(ctrl, text="Aplicar", command=self._apply_window_seconds).grid(row=0, column=2)

        self.fig = Figure(figsize=(10, 6), dpi=100)
        self.ax1 = self.fig.add_subplot(211)
        self.ax2 = self.fig.add_subplot(212, sharex=self.ax1)

        self.ax1.set_ylabel("Altura (mm)")
        self.ax2.set_ylabel("Duty / Válvula (%)")
        self.ax2.set_xlabel("Tempo (s)")

        self.line1_sp, = self.ax1.plot([], [], lw=1, label='SP Altura')
        self.line1_h,  = self.ax1.plot([], [], lw=1, label='Altura medida')
        self.ax1.legend(loc='upper right', fontsize=8)

        self.line2_duty,  = self.ax2.plot([], [], lw=1, label='Duty (%)')
        self.line2_valve, = self.ax2.plot([], [], lw=1, label='Válvula (%)')
        self.ax2.legend(loc='upper right', fontsize=8)

        self.canvas = FigureCanvasTkAgg(self.fig, master=plots)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=1, column=0, sticky="nsew")

        # Rodapé
        status = ttk.Frame(self)
        status.grid(row=4, column=0, sticky="ew", padx=10, pady=(0,6))
        self.status_label = ttk.Label(status, text="Desconectado.")
        self.status_label.grid(row=0, column=0, sticky="w")

    # -------------------------------
    # Conexão serial
    # -------------------------------
    def _refresh_ports(self):
        self.port_combo['values'] = list_serial_ports()
        if not self.port_combo.get() and self.port_combo['values']:
            self.port_combo.current(0)

    def _toggle_connection(self):
        if self.ser and self.ser.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_combo.get()
        if not port:
            messagebox.showwarning("Porta", "Selecione uma porta serial.")
            return
        try:
            self.ser = serial.Serial(port=port, baudrate=BAUDRATE, timeout=READ_TIMEOUT)
            self.reader_alive = True
            self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self.reader_thread.start()
            self.connect_btn.config(text="Desconectar")
            self.status_label.config(text=f"Conectado em {port} @ {BAUDRATE} bps")
        except Exception as e:
            messagebox.showerror("Erro de conexão", str(e))
            self.ser = None
            self.connect_btn.config(text="Conectar")
            self.status_label.config(text="Desconectado.")

    def _disconnect(self):
        self.reader_alive = False
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.connect_btn.config(text="Conectar")
        self.status_label.config(text="Desconectado.")

    # -------------------------------
    # Leitura de dados
    # -------------------------------
    def _reader_loop(self):
        self.buffer = bytearray()
        last_byte_t = None
        while self.reader_alive and self.ser and self.ser.is_open:
            try:
                chunk = self.ser.read(256)  # leitura não bloqueante (timeout curto)
                now = time.monotonic()
                if chunk:
                    for b in chunk:
                        t = time.monotonic()
                        if last_byte_t is not None and (t - last_byte_t) > FRAME_GAP_S:
                            # gap detectado: finalizar frame atual
                            if len(self.buffer) == BYTES_PER_FRAME_RX:
                                self._handle_frame(bytes(self.buffer))
                            # descartar buffers de tamanho diferente
                            self.buffer.clear()
                        self.buffer.append(b if isinstance(b, int) else ord(b))
                        last_byte_t = t
                else:
                    # Sem bytes novos: verificar se há frame pendente por gap
                    if last_byte_t is not None and (now - last_byte_t) > FRAME_GAP_S and self.buffer:
                        if len(self.buffer) == BYTES_PER_FRAME_RX:
                            self._handle_frame(bytes(self.buffer))
                        self.buffer.clear()
                        last_byte_t = None
                    time.sleep(0.005)
            except Exception as e:
                # Erro na leitura: desconectar
                self.after(0, lambda: self.status_label.config(text=f"Erro de leitura: {e}"))
                self.after(0, self._disconnect)
                break

    def _handle_frame(self, frame: bytes):
        if len(frame) != BYTES_PER_FRAME_RX:
            return
        try:
            # > = big-endian, B H H H H H H H
            A, B, C, D, E, F, G, H = struct.unpack('>B H H H H H H H', frame)
        except struct.error:
            return

        # Atualizar variáveis RX
        self.rx_mode.set(A)
        self.rx_height_sp.set(B)
        self.rx_height.set(C)
        self.rx_tof.set(D)
        self.rx_temp_x10.set(E)
        self.rx_valve_sp.set(F)
        self.rx_valve_pos.set(G)
        self.rx_duty.set(H)
        self.rx_temp_degC.set(f"{E/10.0:.1f}")

        # Atualizar históricos para plot
        now = time.time()
        self.t_hist.append(now)
        self.sp_height_hist.append(B)
        self.meas_height_hist.append(C)
        duty_pct = (H / MAX_DUTY_RAW) * 100.0 if MAX_DUTY_RAW else 0.0
        valve_pct = (G / MAX_VALVE_STEPS) * 100.0 if MAX_VALVE_STEPS else 0.0
        self.duty_pct_hist.append(duty_pct)
        self.valve_pct_hist.append(valve_pct)

        # Remover dados fora da janela
        tmin = now - self.window_seconds
        while self.t_hist and self.t_hist[0] < tmin:
            self.t_hist.popleft()
            self.sp_height_hist.popleft()
            self.meas_height_hist.popleft()
            self.duty_pct_hist.popleft()
            self.valve_pct_hist.popleft()

    # -------------------------------
    # Envio de comandos
    # -------------------------------
    def _send_current_command(self):
        # Modo da combobox
        mode_idx = self.mode_combo.current()
        if mode_idx < 0:
            mode_idx = 0
        sp_h = clamp(self.sp_height_var.get(), 0, 500)  # mm
        # Valores configuráveis em %
        sp_v_pct = clamp(self.sp_valve_var.get(), 0, 100)     # %
        sp_d_pct = clamp(self.sp_duty_var.get(), 0, 100)      # %
        # Conversão para unidades cruas do micro
        sp_v = int(round(sp_v_pct * MAX_VALVE_STEPS / 100.0))
        sp_d = int(round(sp_d_pct * MAX_DUTY_RAW / 100.0))
        payload = struct.pack('>B H H H', mode_idx, sp_h, sp_v, sp_d)
        self._send(payload)

    def _send_reset(self):
        payload = struct.pack('>B H H H', 3, 0, 0, 0)
        self._send(payload)

    def _send(self, payload: bytes):
        if not self.ser or not self.ser.is_open:
            messagebox.showwarning("Serial", "Não conectado.")
            return
        if len(payload) != BYTES_PER_FRAME_TX:
            messagebox.showerror("Protocolo", "Tamanho de quadro inválido para TX.")
            return
        try:
            self.ser.write(payload)
            self.status_label.config(text=f"Enviado: {payload.hex(' ').upper()}")
        except Exception as e:
            messagebox.showerror("Erro TX", str(e))

    
    def _apply_window_seconds(self):
        try:
            val = int(self.window_seconds_var.get())
            if val < 5:
                val = 5
            if val > 600:
                val = 600
            self.window_seconds = val
        except Exception:
            pass
# -------------------------------
    # Atualização periódica dos gráficos
    # -------------------------------
    def _schedule_plot_update(self):
        self._update_plots()
        # atualizar a cada 200 ms
        self.after(200, self._schedule_plot_update)

    def _update_plots(self):
        if not self.t_hist:
            return
        # eixo X relativo (segundos desde o primeiro ponto dentro da janela)
        t0 = self.t_hist[0]
        x = [t - t0 for t in self.t_hist]

        self.line1_sp.set_data(x, list(self.sp_height_hist))
        self.line1_h.set_data(x, list(self.meas_height_hist))
        self.line2_duty.set_data(x, list(self.duty_pct_hist))
        self.line2_valve.set_data(x, list(self.valve_pct_hist))

        # Fixar limites razoáveis
        self.ax1.set_ylim(0, HEIGHT_MAX_MM)
        self.ax2.set_ylim(0, 100)

        # Manter X iniciando em 0 até a janela atual
        if x:
            xmax = max(x)
            self.ax1.set_xlim(0, max(5, xmax))
        self.canvas.draw_idle()

    # -------------------------------
    # Fechamento
    # -------------------------------
    def on_close(self):
        try:
            self._disconnect()
        finally:
            self.destroy()

# -------------------------------
# main
# -------------------------------
def main():
    app = BolaNoTuboApp()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()

if __name__ == "__main__":
    main()
