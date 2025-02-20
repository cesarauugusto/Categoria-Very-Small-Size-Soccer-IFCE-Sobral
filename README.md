# Testes-Robo-VSSS ⚽🤖

## Descrição

Projeto para testes e configuração do robô de competição **Very Small Size Soccer (VSSS)**, integrando **ESPDUINO** para processamento de dados, **L298** para controle dos motores e utilizando o **ROS (Robot Operating System)** para comunicação com a câmera.

## 📌 Características Principais

- 🔹 **ESPDUINO** para processamento e comunicação do robô.
- 🔹 **L298** para controle dos motores.
- 🔹 **ROS** para integração com a câmera.
- 🔹 **Visão computacional** para detecção de cores e posicionamento.
- 🔹 **Controle dinâmico** de velocidade angular e linear.

## 📂 Estrutura do Projeto

```
Testes-Robo-VSSS/
├── calibration/               # Scripts para calibração de cores via HSV
│   ├── color_calibration.py   # Calibração de cores do robô e da bola
├── src/
│   ├── color_detection_node.py # Arquivo principal para detecção de cores
│   ├── velocity_control.py     # Cálculo de velocidades e conversão para PWM
├── README.md
```

## 🛠 Como Funciona?

1. **Calibração de cores** 🎨

   - O script `color_calibration.py` é utilizado para ajustar os valores de HSV das cores do robô e da bola.
   - Os valores calibrados são inseridos no arquivo `color_detection_node.py`.

2. **Detecção de cores** 🎯

   - O arquivo `color_detection_node.py` é o principal do projeto.
   - Ele identifica as cores do robô e da bola usando visão computacional.
   - A partir das cores detectadas, calcula as velocidades linear e angular do robô.

3. **Controle do Robô** ⚙️

   - Após calcular as velocidades, os dados são transformados em PWM.
   - O controle final é enviado aos motores para guiar o robô até a bola de forma eficiente e precisa.

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

   - Instale as dependências necessárias para o ROS e OpenCV.
   - Verifique a compatibilidade com o ESPDUINO e o driver do motor L298.

3. **Calibre as cores**

   ```bash
   python calibration/color_calibration.py
   ```

4. **Execute a detecção de cores**

   ```bash
   python src/color_detection_node.py
   ```

## 🏆 Contribuições

Sinta-se à vontade para abrir **Issues** e enviar **Pull Requests** para melhorias!

## 📜 Licença

Este projeto está sob a licença **MIT**. Leia o arquivo [LICENSE](LICENSE) para mais detalhes.

---

🚀 **Desenvolvido com paixão pela robótica!** 🤖⚽

