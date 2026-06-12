import "./App.css";
import Header from "./components/Header";
import Footer from "./components/Footer";
import AmbienteROS from "./pages/AmbienteROS";
import CampoEstrutura from "./pages/CampoEstrutura";
import SistemaFuncionamento from "./pages/SistemaFuncionamento";
import Autores from "./pages/Autores";
import Referencias from "./pages/Referencias";

function App() {
  return (
    <main className="app" id="topo">
      <Header />

      <section className="hero">
  <p className="tag">Projeto de pesquisa e desenvolvimento</p>


  <img
    className="logoHeroIeee"
    src={`${import.meta.env.BASE_URL}images/ieee.png`}
    alt="Logo da IEEE"
  />

  <img
    className="logoHeroIfce"
    src={`${import.meta.env.BASE_URL}images/logo-ifce.png`}
    alt="Logo do IFCE"
  />


  <h1>
    Plataforma Open Source para Reprodução de um Sistema de Competição Very Small Size Soccer
  </h1>

        <p className="descricao">
          Esta plataforma organiza de forma pública e didática o desenvolvimento
          de um projeto de futebol de robôs da categoria IEEE Very Small Size
          Soccer, reunindo ambiente ROS, visão computacional, controle PID,
          estrutura física, algoritmos, códigos de teste e resultados.
        </p>

        <div className="botoes">
          <a href="#sobre">Sobre o projeto</a>
          <a href="#reproducao">Como reproduzir</a>
          <a href="#campo-estrutura">Campo e estrutura</a>
          <a href="#ambiente-ros">Ambiente ROS</a>
          <a href="#sistema">Sistema completo</a>
        </div>
      </section>

      <section className="secao" id="sobre">
        <p className="tag">Apresentação</p>

        <h2>Sobre o projeto</h2>

        <p>
          Este projeto apresenta uma plataforma aberta voltada para estudantes,
          pesquisadores e entusiastas da robótica que desejam compreender e
          reproduzir um sistema Very Small Size Soccer. A proposta reúne
          informações sobre o campo, o protótipo do robô, a eletrônica
          embarcada, o ambiente ROS, a visão computacional, o controle PID e as
          estratégias aplicadas aos robôs.
        </p>

        <p>
          A plataforma tem caráter didático e open source. O objetivo não é
          apenas apresentar o resultado final, mas organizar todo o processo de
          desenvolvimento para que outros estudantes possam estudar, reproduzir,
          modificar e aprimorar o sistema.
        </p>
      </section>

      <section className="grid" id="reproducao">
        <div className="card">
          <h3>1. Ambiente ROS</h3>
          <p>
            Instalação do ROS Noetic, configuração do workspace Catkin,
            instalação do pacote usb_cam e testes no RViz.
          </p>
        </div>

        <div className="card">
          <h3>2. Teste de conexão</h3>
          <p>
            Antes do sistema completo, é possível testar a conexão entre o ROS e
            o robô usando o código de controle pelo teclado WASD.
          </p>
        </div>

        <div className="card">
          <h3>3. Teste de câmera</h3>
          <p>
            A câmera pode ser testada com o código de identificação de cor e
            posição do objeto, validando se a imagem está sendo capturada.
          </p>
        </div>

        <div className="card">
          <h3>4. Calibração HSV</h3>
          <p>
            O código de calibração permite ajustar as barras do HSV em tempo
            real até isolar corretamente a cor da bola e dos marcadores do robô.
          </p>
        </div>

        <div className="card">
          <h3>5. Controle e estratégia</h3>
          <p>
            O sistema utiliza PID e estados como GO_TO_BALL, GO_TO_GOAL e
            RETURN_TO_GOAL para controlar atacante e goleiro.
          </p>
        </div>

        <div className="card">
          <h3>6. Resultados</h3>
          <p>
            Os testes são acompanhados no RViz, permitindo observar trajetórias,
            movimentação do robô e resposta das estratégias.
          </p>
        </div>
      </section>

      <CampoEstrutura />

      <section className="secao" id="recursos">
        <p className="tag">Materiais de apoio</p>

        <h2>Recursos principais</h2>

        <p>
          Os links abaixo reúnem parte da documentação utilizada para instalação
          do ROS, configuração da câmera USB e entendimento dos pacotes
          utilizados no projeto.
        </p>

        <div className="lista">
          <a
            href="https://wiki.ros.org/Distributions"
            target="_blank"
            rel="noreferrer"
          >
            Distributions - ROS Wiki
          </a>

          <a
            href="https://wiki.ros.org/noetic/Installation/Ubuntu"
            target="_blank"
            rel="noreferrer"
          >
            Instalação do ROS Noetic
          </a>

          <a
            href="https://wiki.ros.org/usb_cam"
            target="_blank"
            rel="noreferrer"
          >
            usb_cam - ROS Wiki
          </a>

          <a
            href="https://github.com/ros-drivers/usb_cam"
            target="_blank"
            rel="noreferrer"
          >
            Repositório usb_cam
          </a>

          <a
            href="https://github.com/cesarauugusto/Categoria-Very-Small-Size-Soccer-IFCE-Sobral"
            target="_blank"
            rel="noreferrer"
          >
            Repositório do projeto VSSS no GitHub
          </a>
        </div>
      </section>

      <AmbienteROS />

      <SistemaFuncionamento />

      <Autores />

      <Referencias />

      <Footer />
    </main>
  );
}

export default App;