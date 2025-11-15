# ESP32-CAM (AI Thinker) - Guia de Configuração

## 📋 Requisitos

### Hardware
- ESP32-CAM (AI Thinker)
- Cabo USB para programação (FTDI ou similar)
- Fonte de alimentação adequada (5V, mínimo 500mA)

### Software
- Arduino IDE (versão 1.8.x ou superior)
- Biblioteca ESP32 do Arduino
- **WiFiManager by tzapu** (instalar via Library Manager)

## 🔧 Instalação

### 1. Instalar bibliotecas necessárias

#### WiFiManager
1. No Arduino IDE, vá em **Sketch → Incluir Biblioteca → Gerenciar Bibliotecas**
2. Procure por "WiFiManager"
3. Instale "WiFiManager by tzapu"

### 2. Instalar suporte ESP32 no Arduino IDE

1. Abra o Arduino IDE
2. Vá em **Arquivo → Preferências**
3. No campo "URLs Adicionais para Gerenciadores de Placas", adicione:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Vá em **Ferramentas → Placa → Gerenciador de Placas**
5. Procure por "ESP32" e instale "esp32 by Espressif Systems"

### 3. Configurar a Placa

1. Vá em **Ferramentas → Placa → ESP32 Arduino**
2. Selecione **"AI Thinker ESP32-CAM"**
3. Configure as seguintes opções:
   - **Upload Speed**: 115200
   - **CPU Frequency**: 240MHz
   - **Flash Frequency**: 80MHz
   - **Flash Mode**: QIO
   - **Flash Size**: 4MB (32Mb)
   - **Partition Scheme**: Default 4MB with spiffs
   - **Core Debug Level**: Nenhum
   - **PSRAM**: Enabled (se disponível)

### 4. Upload do Código

1. Conecte o ESP32-CAM ao computador via cabo USB
2. **IMPORTANTE**: Para fazer upload, você precisa:
   - Pressionar e segurar o botão RESET do ESP32-CAM
   - Pressionar e segurar o botão BOOT/IO0
   - Soltar o botão RESET (mantendo BOOT pressionado)
   - Soltar o botão BOOT
   - Fazer o upload no Arduino IDE
   - Após o upload, pressionar RESET novamente

3. Selecione a porta COM correta em **Ferramentas → Porta**
4. Clique em **Upload** (seta para a direita)

## 📡 Uso

### 1. Configurar WiFi (Primeira Vez)

**IMPORTANTE**: O código usa WiFiManager, então você NÃO precisa editar o código para configurar o WiFi!

Na primeira execução ou se não conseguir conectar a uma rede salva:

1. O ESP32-CAM criará um Access Point chamado **"ESP32-CAM-Config"**
2. Conecte-se a essa rede WiFi (senha: **config123**)
3. Abra um navegador e acesse: `http://192.168.4.1`
4. Você verá um portal de configuração WiFi
5. Selecione sua rede WiFi e digite a senha
6. Clique em "Salvar"
7. O ESP32 se conectará automaticamente e reiniciará

### 2. Verificar Conexão

Após o upload, abra o Monitor Serial (Ctrl+Shift+M) com velocidade 115200 baud.

**Se já tiver WiFi configurado:**
```
🔄 Iniciando WiFiManager...
✅ WiFi conectado!
📡 IP Address: 192.168.1.100
📡 RSSI: -45 dBm
🚀 Servidor HTTP iniciado!
```

**Se for a primeira vez (sem WiFi configurado):**
```
🔄 Iniciando WiFiManager...
*WM: Configuring access point...
*WM: AP IP: 192.168.4.1
```
Neste caso, conecte-se ao Access Point "ESP32-CAM-Config" e configure o WiFi.

### 3. Testar no Navegador

Abra seu navegador e acesse:
- `http://[IP_DO_ESP32]/` - Página inicial com informações e links
- `http://[IP_DO_ESP32]/capture` - Capturar uma imagem
- `http://[IP_DO_ESP32]/stream` - Stream de vídeo
- `http://[IP_DO_ESP32]/status` - Status do sistema (JSON)
- `http://[IP_DO_ESP32]/resetwifi` - Resetar configuração WiFi

### 4. Reconfigurar WiFi

Se precisar mudar a rede WiFi:

**Opção 1 - Via Web:**
1. Acesse `http://[IP_DO_ESP32]/resetwifi` no navegador
2. O ESP32 reiniciará e abrirá o portal de configuração
3. Conecte-se ao Access Point "ESP32-CAM-Config" e configure novamente

**Opção 2 - Via Código:**
1. No código, descomente a linha: `wm.resetSettings();`
2. Faça upload do código
3. O ESP32 abrirá o portal de configuração na próxima inicialização

### 5. Configurar no Código Python

No arquivo `raspberrypi.py`, atualize o IP do ESP32-CAM:

```python
ESP_CAM_IP = "192.168.1.100"  # Use o IP mostrado no Monitor Serial
```

## 🔌 Pinout ESP32-CAM (AI Thinker)

```
GPIO 0  - XCLK (Clock da câmera)
GPIO 1  - U0TXD (Serial TX)
GPIO 3  - U0RXD (Serial RX)
GPIO 2  - LED Flash
GPIO 4  - LED Status
GPIO 5  - Y2 (Dados câmera)
GPIO 12 - MTDI
GPIO 13 - MTCK
GPIO 14 - MTMS
GPIO 15 - MTDO
GPIO 16 - XCLK
GPIO 17 - PCLK
GPIO 18 - Y3 (Dados câmera)
GPIO 19 - Y4 (Dados câmera)
GPIO 21 - Y5 (Dados câmera)
GPIO 22 - PCLK
GPIO 23 - HREF
GPIO 25 - VSYNC
GPIO 26 - SIOD (I2C Data)
GPIO 27 - SIOC (I2C Clock)
GPIO 32 - PWDN
GPIO 34 - Y8 (Dados câmera)
GPIO 35 - Y9 (Dados câmera)
GPIO 36 - Y6 (Dados câmera)
GPIO 39 - Y7 (Dados câmera)
```

## ⚠️ Problemas Comuns

### 1. Erro ao fazer upload
- Certifique-se de pressionar os botões RESET e BOOT na sequência correta
- Tente reduzir a velocidade de upload para 921600 ou 115200
- Verifique se o cabo USB está funcionando corretamente

### 2. Câmera não inicializa
- Verifique se a câmera está conectada corretamente
- Alguns modelos podem precisar de ajustes nos pinos
- Tente reduzir a resolução no código

### 3. WiFi não conecta
- Se for a primeira vez, conecte-se ao Access Point "ESP32-CAM-Config" e configure via portal web
- Certifique-se de que o WiFi está em 2.4GHz (ESP32 não suporta 5GHz)
- Verifique a força do sinal WiFi
- Use a rota `/resetwifi` para reconfigurar o WiFi
- Se necessário, descomente `wm.resetSettings();` no código e faça upload novamente

### 4. Imagens muito grandes/lentas
- Reduza a resolução no código (FRAMESIZE_SVGA ou menor)
- Ajuste a qualidade JPEG (aumente o número para menor qualidade)
- Verifique se o ESP32 tem PSRAM habilitado

## 🔧 Ajustes de Performance

### Para melhor qualidade (se tiver PSRAM):
```cpp
config.frame_size = FRAMESIZE_UXGA;  // 1600x1200
config.jpeg_quality = 10;            // Alta qualidade
```

### Para melhor performance (sem PSRAM):
```cpp
config.frame_size = FRAMESIZE_VGA;    // 640x480
config.jpeg_quality = 12;             // Qualidade média
```

### Para máxima velocidade:
```cpp
config.frame_size = FRAMESIZE_QVGA;   // 320x240
config.jpeg_quality = 15;             // Qualidade menor
```

## 📚 Recursos Adicionais

- [Documentação ESP32-CAM](https://github.com/espressif/arduino-esp32)
- [Exemplos ESP32-CAM](https://github.com/espressif/arduino-esp32/tree/master/libraries/ESP32/examples/Camera)
- [Fórum ESP32](https://www.esp32.com/)

