# 🤖⚽ VSSS Open Platform

<p align="center">
  <img src="./public/images/ieee.png" alt="Logo IEEE" width="90"/>
  &nbsp;&nbsp;&nbsp;&nbsp;
  <img src="./public/images/logo-ifce.png" alt="Logo IFCE" width="150"/>
</p>

<h2 align="center">
  Plataforma Open Source para Reprodução de um Sistema de Competição Very Small Size Soccer
</h2>

<p align="center">
  <strong>Robótica móvel • Visão computacional • ROS • OpenCV • Sistemas embarcados • Futebol de robôs</strong>
</p>

---

## 🌐 Acesse a plataforma online

<p align="center">
  <a href="https://cesarauugusto.github.io/Categoria-Very-Small-Size-Soccer-IFCE-Sobral/">
    <strong>👉 Clique aqui para visitar o site do projeto</strong>
  </a>
</p>

<p align="center">
  No site está a documentação completa do projeto, com explicações sobre o campo, estrutura do robô, ambiente ROS, visão computacional, funcionamento do sistema, códigos, imagens, referências e materiais de apoio.
</p>

---

## 👨‍💻 Informações do projeto

| Função                    | Nome                                                                    |
| ------------------------- | ----------------------------------------------------------------------- |
| **Autor / Desenvolvedor** | **César Augusto**                                                       |
| **Orientador**            | **Prof. Aldinei**                                                       |
| **Coorientador**          | **Prof. Tabosa**                                                        |
| **Instituição**           | **Instituto Federal de Educação, Ciência e Tecnologia do Ceará — IFCE** |
| **Categoria**             | **IEEE Very Small Size Soccer — VSSS**                                  |
| **Tipo de projeto**       | **Pesquisa, desenvolvimento e documentação open source**                |

---

## 🏫 Instituição

<p align="center">
  <img src="./public/images/logo-ifce.png" alt="Logo IFCE" width="160"/>
</p>

<p align="center">
  <strong>Instituto Federal de Educação, Ciência e Tecnologia do Ceará — IFCE</strong>
</p>

---

## 🛠️ Tecnologias principais

<p align="center">
  <img src="./public/images/ubuntu.png" alt="Logo Ubuntu" width="110"/>
  &nbsp;&nbsp;&nbsp;&nbsp;
  <img src="./public/images/ros.png" alt="Logo ROS" width="150"/>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Status-Em%20desenvolvimento-198754?style=for-the-badge"/>
  <img src="https://img.shields.io/badge/Ubuntu-20.04-E95420?style=for-the-badge"/>
  <img src="https://img.shields.io/badge/ROS-Noetic-22314a?style=for-the-badge"/>
  <img src="https://img.shields.io/badge/OpenCV-Visão%20Computacional-5C3EE8?style=for-the-badge"/>
  <img src="https://img.shields.io/badge/Python-3.x-3776AB?style=for-the-badge"/>
</p>

---

## 📌 Sobre o projeto

O **VSSS Open Platform** é uma plataforma aberta criada para apresentar, organizar e documentar um sistema de futebol de robôs da categoria **IEEE Very Small Size Soccer — VSSS**.

O projeto reúne informações sobre a construção do campo, a estrutura física do robô, o ambiente ROS, a câmera USB, a visão computacional, a comunicação com o microcontrolador e as estratégias de movimentação usadas no sistema.

A proposta é disponibilizar um material público, visual e didático para estudantes, pesquisadores e equipes interessadas em robótica móvel, sistemas embarcados, controle robótico e futebol de robôs.

---

## 🎯 Objetivo

O objetivo do projeto é desenvolver e documentar uma plataforma **open source** para auxiliar no estudo e na reprodução de um sistema de competição **Very Small Size Soccer**.

A plataforma busca reunir em um só lugar:

* **Explicação do projeto**
* **Estrutura física do campo**
* **Modelagem e organização do robô**
* **Ambiente ROS**
* **Processamento de imagem**
* **Identificação de cor e posição**
* **Controle do robô**
* **Estratégias de atacante e goleiro**
* **Códigos de teste**
* **Programas finais**
* **Referências e recursos de apoio**

---

## 🧠 Áreas envolvidas

| Área                    | Aplicação no projeto                             |
| ----------------------- | ------------------------------------------------ |
| **Robótica móvel**      | Controle e movimentação dos robôs no campo       |
| **Visão computacional** | Detecção da bola, robôs e marcadores             |
| **ROS**                 | Organização da comunicação entre os processos    |
| **OpenCV**              | Processamento das imagens capturadas pela câmera |
| **Sistemas embarcados** | Controle dos motores por microcontrolador        |
| **Controle robótico**   | Orientação, deslocamento e tomada de decisão     |
| **Modelagem física**    | Campo, estrutura do robô e arquivos de apoio     |
| **Interface web**       | Apresentação pública e organizada do projeto     |

---

## ⚙️ Plataformas e ferramentas utilizadas

| Categoria                  | Tecnologia       |
| -------------------------- | ---------------- |
| **Sistema operacional**    | Ubuntu 20.04 LTS |
| **Middleware robótico**    | ROS Noetic       |
| **Linguagem principal**    | Python           |
| **Visão computacional**    | OpenCV           |
| **Visualização ROS**       | RViz             |
| **Visualização de imagem** | rqt_image_view   |
| **Grafo de comunicação**   | rqt_graph        |
| **Câmera**                 | Câmera USB       |
| **Pacote de câmera**       | usb_cam          |
| **Interface web**          | React + Vite     |
| **Versionamento**          | Git e GitHub     |
| **Hospedagem**             | GitHub Pages     |

---

## 🔌 Microcontrolador e eletrônica

O sistema utiliza microcontrolador para receber os comandos enviados pelo computador e acionar os motores do robô.

### Microcontrolador utilizado

* **ESP8266 / WeMos ESP8266**

### Componentes relacionados

* **ESP8266**
* **Motores DC**
* **Driver ou shield de motores**
* **Bateria**
* **Estrutura impressa em 3D**
* **Câmera USB**
* **Computador com Ubuntu e ROS**

O ESP8266 atua como ponte entre o sistema de controle executado no computador e os motores responsáveis pela movimentação do robô.

---

## 🧱 Campo e estrutura física

A estrutura física do projeto foi desenvolvida com base nas características da categoria **IEEE Very Small Size Soccer**.

### Campo

O campo foi planejado considerando:

* Superfície preta;
* Material não reflexivo;
* Estrutura em madeira;
* Paredes laterais;
* Área do goleiro;
* Gols;
* Marcações internas;
* Cantos adaptados para evitar que a bola fique presa.

### Robô

O protótipo do robô foi modelado para comportar os principais componentes eletrônicos e mecânicos do sistema.

A estrutura considera:

* Espaço para motores;
* Espaço para bateria;
* Posicionamento do microcontrolador;
* Organização interna dos componentes;
* Facilidade de montagem;
* Facilidade de manutenção;
* Limites dimensionais da categoria.

---

## 📷 Visão computacional

A visão computacional é uma das partes centrais do projeto.

A câmera é posicionada acima do campo, capturando uma visão superior do ambiente. A partir dessa imagem, o sistema identifica os principais elementos necessários para o funcionamento do robô.

O sistema busca reconhecer:

* **Bola**
* **Robô**
* **Marcadores coloridos**
* **Posição dos objetos**
* **Orientação do robô**
* **Regiões importantes do campo**

O processamento é feito com **OpenCV**, utilizando segmentação por cor, máscaras HSV, contornos e cálculo de centróides.

---

## 🎨 Calibração de cores

A calibração HSV permite ajustar os valores de cor usados para identificar corretamente os objetos no campo.

Essa etapa é importante porque fatores como iluminação, altura da câmera, brilho do campo e tipo de marcador podem alterar a forma como as cores aparecem na imagem.

Com a calibração adequada, o sistema consegue identificar melhor:

* A bola;
* Os marcadores do robô;
* A posição dos objetos;
* A orientação do robô;
* Os elementos usados na tomada de decisão.

---

## 🧭 Funcionamento geral do sistema

O funcionamento do sistema pode ser entendido como uma integração entre câmera, software e hardware.

```text
Campo → Câmera USB → ROS → OpenCV → Algoritmo de controle → ESP8266 → Motores
```

De forma resumida:

1. A câmera captura a imagem superior do campo.
2. O ROS organiza a comunicação entre os processos.
3. O OpenCV processa a imagem capturada.
4. O algoritmo identifica bola, robô e orientação.
5. O sistema calcula os comandos de movimentação.
6. O ESP8266 recebe os comandos.
7. Os motores executam o movimento do robô.

---

## 🤖 Estratégias do sistema

O projeto apresenta estratégias básicas para controle e movimentação dos robôs.

### Robô atacante

O atacante é responsável por buscar a bola e conduzi-la em direção ao gol.

Entre os comportamentos trabalhados estão:

* Ir até a bola;
* Alinhar-se com o alvo;
* Conduzir a bola;
* Corrigir a movimentação quando necessário.

### Robô goleiro

O goleiro atua na região defensiva do campo.

Entre os comportamentos trabalhados estão:

* Permanecer próximo à área do goleiro;
* Retornar para a região defensiva;
* Avançar em direção à bola quando ela se aproxima;
* Proteger a região do gol.

---

## 🖥️ Plataforma web

Além dos códigos e arquivos do projeto, este repositório possui uma interface web desenvolvida com **React + Vite**.

A plataforma foi criada para apresentar o projeto de forma mais visual, organizada e acessível.

Ela contém seções sobre:

* **Sobre o projeto**
* **Como reproduzir**
* **Campo e estrutura**
* **Ambiente ROS**
* **Sistema completo**
* **Autores**
* **Referências**
* **Recursos**

<p align="center">
  <a href="https://cesarauugusto.github.io/Categoria-Very-Small-Size-Soccer-IFCE-Sobral/">
    <strong>🌐 Acessar a plataforma online</strong>
  </a>
</p>

---

## 📚 Referências e materiais de apoio

O projeto utiliza como base documentações, ferramentas e materiais relacionados a:

* **ROS**
* **ROS Noetic**
* **OpenCV**
* **usb_cam**
* **RViz**
* **rqt_graph**
* **IEEE Very Small Size Soccer**
* **Robótica móvel**
* **Visão computacional**
* **Sistemas embarcados**

---

## 📌 Status do projeto

O projeto está em desenvolvimento e pode receber melhorias futuras.

Possíveis continuações incluem:

* Estratégias para múltiplos robôs;
* Melhorias no controle;
* Otimização da visão computacional;
* Melhorias na comunicação;
* Estratégias cooperativas;
* Expansão para uma equipe completa;
* Aplicação de técnicas de inteligência artificial.

---

## ⭐ Apoie o projeto

Se este repositório foi útil, considere deixar uma estrela no GitHub.

Isso ajuda outras pessoas a encontrarem o projeto e contribui para a divulgação de materiais abertos na área de robótica educacional.

---

<p align="center">
  <strong>🤖⚽ VSSS Open Platform</strong>
  <br/>
  Plataforma aberta para estudo, documentação e desenvolvimento de sistemas Very Small Size Soccer.
</p>
