# Testes-Robo-VSSS ⚽🤖

## Descrição

Projeto para testes e configuração do robô de competição **Very Small Size Soccer (VSSS)**, integrando **WeMos D1 R2 WiFi ESP8266** para processamento de dados e comunicação, **L298P Motor Shield** para controle dos motores e utilizando o **ROS (Robot Operating System)** para comunicação com a câmera.

📌 **Acesse nosso ****[Trello](https://trello.com/invite/b/QqVcYJjL/ATTI950a8fb48b9b4b5b42c92ab7aaf6e06dF6B7B3DC/categoria-very-small-sizer-soccer-ifce-sobral)**** para informações detalhadas e organizadas sobre recursos, instalação das dependências do ROS, fotos dos testes, fotos dos robôs e do campo.**

## 📌 Características Principais

- 🔹 **WeMos D1 R2 WiFi ESP8266** para comunicação Wi-Fi do robô.
- 🔹 **L298P Motor Shield** para controle dos motores.
- 🔹 **ROS** para integração com a câmera.
- 🔹 **Visão computacional** com OpenCV para detecção de cores, posicionamento e orientação.
- 🔹 **Controle PID** para ajuste dinâmico das velocidades angular e linear.
- 🔹 **Comunicação Wi-Fi** entre robôs e computador.

## 📂 Estrutura do Projeto

```
Testes-Robo-VSSS/
├── algoritmos dos robôs/      # Algoritmos principais dos robôs
│   ├── algoritmo do jogador.py # Algoritmo do robô jogador
│   ├── algoritmo do goleiro.py # Algoritmo do robô goleiro
├── calibração/                # Scripts para calibração de cores via HSV
│   ├── calibration.py         # Calibração de cores do robô e da bola
├── Outros testes/             # Testes adicionais e controle manual
├── README.md
```

## 🛠 Como Funciona?

O controle principal do robô é feito por um nó no computador, que faz um **subscribe** das imagens da câmera e um **publish** das velocidades para o **WeMos D1 R2 WiFi ESP8266**. A ESP recebe esses comandos e envia as velocidades para a ponte (**L298P Motor Shield**) para controlar os motores do robô.

1. **Calibração de cores** 🎨

   - Execute o pacote `usb_cam` e o script `calibration.py`.
   - Ajuste os valores HSV usando sliders para definir as cores do robô e da bola.
   - Anote os valores HSV (lower e upper) das cores e atualize nos arquivos `algoritmo do jogador.py` ou `algoritmo do goleiro.py`.

2. **Execução dos Algoritmos** 🎯

   - Para rodar o robô jogador ou goleiro, execute:
     - `roscore`
     - Pacote `usb_cam` para a câmera
     - `serial_node` para conectar a ESP ao computador
     - O arquivo `algoritmo do jogador.py` **ou** `algoritmo do goleiro.py` (os dois não funcionam juntos simultaneamente)

## 🎥 Vídeos de Testes

Confira os testes do robô em funcionamento:


🔗 Playlist completa dos testes: [YouTube](https://www.youtube.com/watch?v=eGvhpNceoEk\&list=PLrcudhIfihuCvisP4GRgBESSTaiVgz14I\&index=1)

## 🚀 Como Executar o Projeto

1. **Clone o repositório**

   ```bash
   git clone https://github.com/seu-usuario/Testes-Robo-VSSS.git
   cd Testes-Robo-VSSS
   ```

2. **Configure o ambiente**

   - Instale ROS, OpenCV e dependências.
   - Verifique compatibilidade com WeMos D1 R2 WiFi ESP8266 e L298P.

3. **Calibre as cores**

   ```bash
   roscore
   rosrun usb_cam usb_cam_node
   python calibration.py
   ```

4. **Execute os algoritmos dos robôs**

   ```bash
   roscore
   rosrun usb_cam usb_cam_node
   rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0
   python "algoritmos dos robôs/algoritmo do jogador.py" # ou
   python "algoritmos dos robôs/algoritmo do goleiro.py"
   ```

## 🏆 Contribuições

Sinta-se à vontade para abrir **Issues** e enviar **Pull Requests** para melhorias!

## 📜 Licença

Este projeto está sob a licença **MIT**. Leia o arquivo [LICENSE](LICENSE) para mais detalhes.

---

🚀 **Desenvolvido com paixão pela robótica!** 🤖⚽

