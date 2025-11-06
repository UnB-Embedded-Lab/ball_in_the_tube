# 🧠 Bola no Tubo – Aplicativo

Este aplicativo em Python permite **monitorar e controlar** o experimento de **Bola no Tubo**, enviando comandos de controle e configurando parâmetros de PID ao microcontrolador via **Bluetooth (HC-05)** ou **porta serial USB**.  

Ele foi desenvolvido em **Tkinter + Matplotlib + PySerial**, com suporte completo a exibição de dados em tempo real.

---

## 🧩 Funcionalidades principais

✅ Comunicação serial com o microcontrolador por Bluetooth (HC-05).  
✅ Exibição em tempo real de:
- **Altura medida da bola (mm)** e **setpoint de altura**.  
- **Duty-cycle da ventoinha (%)** e **posição da válvula (%)** no mesmo gráfico.  
- **Temperatura (°C)** e demais variáveis recebidas.

✅ Envio de comandos:
- Mudar o **modo de operação** (manual, ventoinha, válvula, reset).  
- Alterar **setpoints de altura**, **duty (%)**, e **posição da válvula (%)**. 

✅ Configuração gráfica:
- Janela de tempo ajustável (5 – 600 s).  
- Escalas automáticas e atualização contínua.  

✅ Ícone institucional da **UnB** integrado na janela e na barra de tarefas (Windows).

---

## 🔧 Requisitos de software

Instale as dependências com:

```bash
pip install -r requirements.txt
```

Dependências:
- `pyserial`
- `matplotlib`
- `tkinter` (vem com Python em Windows)

Recomenda-se Python ≥ 3.8.

---

## 📡 Pareamento do módulo Bluetooth HC-05

Antes de abrir o app:
1. Ligue o módulo **HC-05** conectado ao microcontrolador.  
2. No **Windows**, abra **Configurações → Bluetooth e dispositivos → Adicionar dispositivo**.  
3. Escolha **Bluetooth clássico**, selecione **HC-05**, e insira o PIN padrão `1234` ou `0000`.  
4. Após o pareamento, o Windows criará uma **porta COM** (ex: `COM7` ou `COM9`).  
5. Abra o aplicativo e selecione essa porta no menu suspenso **“Porta:”**.  
6. Clique em **Conectar** — a mensagem de status mostrará `Conectado em COMx @ 115200 bps`.

> 💡 Dica: Se a conexão falhar, verifique se a porta não está em uso por outro software (como o Serial Monitor do Arduino IDE).

---

## 🖥️ Uso do aplicativo

### 1️⃣ Conectar e visualizar
- Escolha a porta serial (`COMx` ou `/dev/ttyUSBx`) e clique **Conectar**.  
- O app começará a ler quadros de dados enviados pelo microcontrolador a cada 40 ms.

### 2️⃣ Monitorar
Os valores recebidos são mostrados na seção **“Recebidos do Micro”** e nos gráficos:
- **Gráfico 1:** Setpoint e altura medida (mm).  
- **Gráfico 2:** Duty (%) e posição da válvula (%).

### 3️⃣ Enviar comandos
- Escolha o **modo de operação** (Manual, Ventoinha, Válvula, Reset).  
- Ajuste os valores desejados:
  - **Altura (mm)**  
  - **Válvula (%)**  
  - **Duty (%)**
- Clique em **Enviar**.


---

## 🧾 Protocolo de comunicação

### Recepção (micro → PC)
Quadro de **15 bytes**, big-endian:
| Byte(s) | Descrição | Tipo |
|----------|------------|------|
| 1 | modo atual | uint8 |
| 2–3 | SP de altura | uint16 |
| 4–5 | altura medida | uint16 |
| 6–7 | ToF médio | uint16 |
| 8–9 | temperatura ×10 | uint16 |
| 10–11 | SP da válvula | uint16 |
| 12–13 | posição da válvula | uint16 |
| 14–15 | duty (raw) | uint16 |

### Envio (PC → micro)
- Modo 0/1/2/3: `>B H H H` (7 bytes)

---

## 🪟 Ícone e integração visual

O aplicativo usa o ícone oficial da **UnB** (`unb.ico`) tanto:
- Na **janela principal**,  
- Quanto na **barra de tarefas do Windows**, via `AppUserModelID`.

No caso de empacotamento com PyInstaller:
```bash
pyinstaller -w -F bola_no_tubo.py --icon unb.ico
```

---

## 📎 Ajustes e parâmetros
No topo do arquivo `bola_no_tubo.py`, é possível alterar:
```python
HEIGHT_MAX_MM = 500
MAX_DUTY_RAW = 1023
MAX_VALVE_STEPS = 420
```
Esses valores definem os limites máximos usados para escalas e conversões.

---

## 💬 Suporte e sugestões
Caso os gráficos não exibam dados, verifique:
- O microcontrolador realmente envia quadros de 15 bytes.  
- A taxa de transmissão está em **115200 bps**.  
- Nenhum outro programa usa a porta serial.

## 🧰 Licença e autoria

Projeto desenvolvido na **Universidade de Brasília (UnB)**  
Laboratório de Sistemas Embarcados — Prof. Dr. Guillermo Álvarez Bestard

Uso acadêmico e pesquisa. Cite a fonte em publicações relacionadas.
