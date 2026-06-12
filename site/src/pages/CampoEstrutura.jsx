const imagePath = (fileName) => `${import.meta.env.BASE_URL}images/${fileName}`;

export default function CampoEstrutura() {
  return (
    <section className="secao" id="campo-estrutura">
      <p className="tag">Estrutura física</p>

      <h2>Campo e estrutura do robô</h2>

      <p>
        A estrutura física do projeto foi desenvolvida considerando as regras da
        categoria IEEE Very Small Size Soccer. Essa etapa envolve o planejamento
        do campo, a modelagem do robô, a organização interna dos componentes e a
        preparação do ambiente para os testes com visão computacional e controle.
      </p>

      <div className="estruturaVertical">
        <article className="estruturaBloco">
          <img
            src={imagePath("campo-vsss.png")}
            alt="Campo da categoria IEEE Very Small Size Soccer"
          />

          <div>
            <h3>Campo da categoria VSSS</h3>

            <p>
              O campo da competição foi construído com base nas regras da
              categoria IEEE Very Small Size Soccer. A estrutura deve possuir
              cor preta, superfície não reflexiva e ser construída em madeira,
              com textura semelhante à de uma mesa de pingue-pongue.
            </p>

            <p>
              As medidas principais do campo são de{" "}
              <strong>1,50 m de comprimento</strong> por{" "}
              <strong>1,30 m de largura</strong>. Além dessas dimensões, o campo
              possui paredes ao redor para evitar que a bola escape durante os
              testes. Essas paredes possuem <strong>5 cm de altura</strong> e{" "}
              <strong>2,5 cm de espessura</strong>.
            </p>

            <p>
              Para evitar que a bola fique presa nos cantos, cada canto do campo
              possui um triângulo retângulo com lados de <strong>7 cm</strong>.
              O gol possui <strong>40 cm de comprimento</strong> e{" "}
              <strong>10 cm de profundidade</strong>. As linhas e marcações
              internas também foram definidas de acordo com as especificações da
              competição.
            </p>

            <ul>
              <li>Campo preto, não reflexivo e construído em madeira.</li>
              <li>Dimensões principais: 1,50 m x 1,30 m.</li>
              <li>
                Paredes laterais com 5 cm de altura e 2,5 cm de espessura.
              </li>
              <li>Cantos com triângulos retângulos de lados iguais a 7 cm.</li>
              <li>Gol com 40 cm de comprimento e 10 cm de profundidade.</li>
              <li>Área do goleiro utilizada nas estratégias defensivas.</li>
            </ul>

            <a
              className="botaoDocumento"
              href="https://drive.google.com/file/d/15ujm5bh02MM3qjBlRszCguBq1YDOuxfm/view?usp=sharing"
              target="_blank"
              rel="noreferrer"
            >
              Ver desenho técnico 2D do campo
            </a>
          </div>
        </article>

        <article className="estruturaBloco">
          <img
            src={imagePath("prototipo-robo.png")}
            alt="Protótipo 3D do robô VSSS"
          />

          <div>
            <h3>Protótipo do robô</h3>

            <p>
              O protótipo do robô foi modelado em software 3D respeitando o
              limite dimensional da categoria. A estrutura foi projetada para
              acomodar os motores, a bateria, a placa controladora e o shield de
              acionamento dos motores.
            </p>

            <p>
              Essa modelagem foi importante para organizar os componentes de
              forma compacta e permitir que o robô pudesse se movimentar dentro
              do campo sem ultrapassar as dimensões permitidas pela categoria.
            </p>
          </div>
        </article>

        <article className="estruturaBloco">
          <img
            src={imagePath("componentes-robo.png")}
            alt="Componentes internos do robô VSSS"
          />

          <div>
            <h3>Organização interna dos componentes</h3>

            <p>
              A parte interna do robô foi organizada para facilitar a montagem,
              a manutenção e o acesso aos componentes. A disposição dos
              elementos permite melhor aproveitamento do espaço disponível dentro
              do protótipo.
            </p>

            <p>
              Dentro da estrutura foram posicionados os motores, a bateria e os
              componentes eletrônicos responsáveis pelo recebimento dos comandos
              e pelo acionamento dos motores.
            </p>

            <a
              className="botaoDocumento"
              href="https://drive.google.com/drive/folders/14HOnzzR1LY0sx_cKD8RnJsX6sFZjANC9?usp=sharing"
              target="_blank"
              rel="noreferrer"
            >
              Acessar arquivos de impressão 3D
            </a>
          </div>
        </article>
      </div>

      <div className="secaoInterna">
        <h3>Relação entre campo, câmera e robô</h3>

        <img
          className="imagemFluxoProjeto"
          src={imagePath("fluxo-campo-camera-robo.png")}
          alt="Diagrama da relação entre campo, câmera e robôs"
        />

        <p>
          Durante os testes, a câmera deve ser posicionada acima do campo, de
          forma centralizada e reta, apontada perpendicularmente para baixo. Esse
          posicionamento é importante para que o campo seja capturado como uma
          imagem praticamente 2D, reduzindo distorções de perspectiva e
          facilitando o processamento por visão computacional.
        </p>

        <p>
          Quando a câmera está alinhada corretamente, o sistema consegue
          identificar com mais precisão a posição da bola, dos robôs e das
          marcações do campo. Por isso, antes dos testes, é necessário ajustar a
          altura, o enquadramento e o alinhamento da câmera para que todo o campo
          apareça na imagem.
        </p>

        <img
          className="imagemFluxoProjeto"
          src={imagePath("visao-superior-campo.png")}
          alt="Exemplo da visão superior do campo capturada pela câmera"
        />

        <div className="fluxoSistema fluxoClaro">
          <span>Campo</span>
          <strong>→</strong>
          <span>Câmera superior</span>
          <strong>→</strong>
          <span>ROS</span>
          <strong>→</strong>
          <span>OpenCV</span>
          <strong>→</strong>
          <span>Robô</span>
        </div>
      </div>
    </section>
  );
}