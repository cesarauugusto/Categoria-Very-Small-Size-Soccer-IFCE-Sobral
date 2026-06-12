import "./SistemaFuncionamento.css";

const repoBase =
  "https://github.com/cesarauugusto/Categoria-Very-Small-Size-Soccer-IFCE-Sobral";

function CodeBlock({ children, label = "Terminal" }) {
  return (
    <div className="terminalBox">
      <div className="terminalLabel">{label}:</div>

      <pre className="codeBlock">
        <code>{children}</code>
      </pre>
    </div>
  );
}

function ImageCard({ src, alt, caption }) {
  return (
    <figure className="imageCard">
      <img src={src} alt={alt} />
      <figcaption>{caption}</figcaption>
    </figure>
  );
}

function RepoCard({ title, description, link }) {
  return (
    <a className="repoCard" href={link} target="_blank" rel="noreferrer">
      <span>GitHub</span>
      <h3>{title}</h3>
      <p>{description}</p>
    </a>
  );
}

export default function SistemaFuncionamento() {
  return (
    <section className="sistemaPage" id="sistema">
      <div className="sistemaHeader">
        <p className="tag">Funcionamento do sistema</p>

        <h2>Como o sistema VSSS funciona na prática</h2>

        <p>
          Depois da instalação do ROS e da configuração da câmera, o sistema
          passa a funcionar como uma arquitetura integrada. A câmera captura o
          campo, o ROS organiza a comunicação entre os nós, o OpenCV processa a
          imagem, o algoritmo calcula a estratégia e os comandos são enviados ao
          robô por comunicação sem fio.
        </p>

        <p>
          Esta seção explica o funcionamento completo do sistema de forma
          didática: primeiro a conexão do robô, depois o teste da câmera, a
          calibração HSV, a identificação dos objetos, o controle PID, as
          estratégias e, por fim, a execução do sistema completo.
        </p>
      </div>

      <div className="sistemaSection">
        <h2>1. Vídeo de apoio sobre ROS e conexão com o robô</h2>

        <p>
          O vídeo abaixo foi utilizado como apoio para compreender melhor a
          conexão entre o robô e o ROS. Ele pode ser assistido diretamente nesta
          plataforma. A ideia é que o estudante veja primeiro o conceito geral
          da comunicação e depois siga para os testes práticos disponíveis no
          repositório.
        </p>

        <div className="videoBox">
          <iframe
            src="https://www.youtube.com/embed/Fc6qsprvia4?start=11"
            title="Vídeo de apoio sobre conexão do robô com ROS"
            allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
            allowFullScreen
          ></iframe>
        </div>
      </div>

      <div className="sistemaSection">
        <h2>2. Organização dos códigos no GitHub</h2>

        <p>
          O repositório do projeto reúne diferentes versões, testes e programas
          finais. Para quem está começando, não é necessário executar tudo de uma
          vez. O melhor caminho é iniciar pelos testes mais simples, validar cada
          parte separadamente e só depois executar o sistema completo.
        </p>

        <div className="repoGrid">
          <RepoCard
            title="Controle do robô pelo teclado - WASD"
            description="Primeiro teste recomendado para verificar a conexão entre o ROS, o computador e o robô. Com ele, é possível enviar comandos simples de movimento pelo teclado."
            link={`${repoBase}/tree/main/Controle%20do%20robo%20pelo%20teclado%20-%20WASD`}
          />

          <RepoCard
            title="Código para o ESP8266"
            description="Código que deve ser gravado no ESP8266 para que o robô receba comandos do computador e acione corretamente os motores."
            link={`${repoBase}/tree/main/Controle%20do%20robo%20pelo%20teclado%20-%20WASD/Codigo_para_ESP`}
          />

          <RepoCard
            title="Identificando cor e posição do objeto"
            description="Código recomendado para testar a câmera e verificar se o sistema consegue identificar cores e posições na imagem antes de aplicar a estratégia completa."
            link={`${repoBase}/tree/main/Identificando%20cor%20e%20posicao%20do%20objeto`}
          />

          <RepoCard
            title="calibration"
            description="Código essencial para calibrar as cores HSV. É nele que o usuário ajusta as barras deslizantes até isolar corretamente a bola e os marcadores do robô."
            link={`${repoBase}/tree/main/calibration`}
          />

          <RepoCard
            title="Programas Finais"
            description="Pasta onde ficam os códigos finais usados para testar o goleiro e o jogador de linha, como goleiro.py e jogador.py."
            link={`${repoBase}/tree/main/Programas%20Finais`}
          />
        </div>

        <a
          className="botaoDocumento"
          href={repoBase}
          target="_blank"
          rel="noreferrer"
        >
          Acessar repositório completo
        </a>
      </div>

      <div className="sistemaSection">
        <h2>3. Primeiro teste: conexão do ROS com o robô</h2>

        <p>
          Antes de executar a visão computacional e as estratégias automáticas,
          recomenda-se testar apenas a comunicação entre o computador e o robô.
          Para isso, pode-se iniciar pelo código da pasta{" "}
          <strong>Controle do robô pelo teclado - WASD</strong>.
        </p>

        <p>
          Esse teste serve para verificar se o robô está recebendo comandos via
          rede e se os motores respondem corretamente. A lógica é simples: o
          usuário pressiona teclas como W, A, S e D, e o robô executa movimentos
          básicos, como avançar, voltar e girar.
        </p>

        <p>
          Para esse teste funcionar, o ESP8266 precisa estar com o código correto
          gravado. Esse código fica na pasta:
        </p>

        <CodeBlock label="Caminho no GitHub">{`Controle do robo pelo teclado - WASD/Codigo_para_ESP`}</CodeBlock>

        <p>
          Depois que o ESP8266 estiver programado, o computador poderá enviar
          comandos para o robô. Essa etapa é importante porque confirma se a
          comunicação e o acionamento dos motores estão funcionando antes de
          avançar para o sistema com câmera e visão computacional.
        </p>

        <ol className="instructionList">
          <li>Ligue o robô.</li>
          <li>Verifique se o robô está conectado à mesma rede do computador.</li>
          <li>Inicie o ROS Master no computador.</li>
          <li>Execute o código de controle pelo teclado.</li>
          <li>Envie comandos WASD e observe se os motores respondem.</li>
        </ol>

        <p>Abra o terminal e execute:</p>

        <CodeBlock>{`roscore`}</CodeBlock>

        <p>
          Depois, em outro terminal, acesse a pasta do teste WASD e execute o
          código correspondente. O nome exato do arquivo pode variar conforme a
          versão presente no repositório.
        </p>

        <CodeBlock>{`cd ~/catkin_ws/src
python3 nome_do_codigo_wasd.py`}</CodeBlock>

        <a
          className="botaoDocumento"
          href={`${repoBase}/tree/main/Controle%20do%20robo%20pelo%20teclado%20-%20WASD`}
          target="_blank"
          rel="noreferrer"
        >
          Acessar teste WASD
        </a>
      </div>

      <div className="sistemaSection">
        <h2>4. Segundo teste: câmera, cor e posição do objeto</h2>

        <p>
          Após validar a conexão com o robô, o próximo teste recomendado é a
          câmera. Para isso, pode-se utilizar o código da pasta{" "}
          <strong>Identificando cor e posição do objeto</strong>. Esse teste
          permite verificar se a câmera está funcionando e se o sistema consegue
          identificar objetos coloridos na imagem.
        </p>

        <p>
          Esse passo é importante porque o sistema depende da imagem para tomar
          decisões. Se a câmera não estiver capturando corretamente, ou se as
          cores não estiverem bem identificadas, o robô pode se orientar de forma
          errada dentro do campo.
        </p>

        <ImageCard
          src="/images/robo-bola-camera.png"
          alt="Robô e bola capturados pela câmera"
          caption="Imagem inicial capturada pela câmera, contendo o robô, os marcadores coloridos e a bola."
        />

        <p>Com o ROS Master ativo, abra outro terminal e execute a câmera:</p>

        <CodeBlock>{`roslaunch usb_cam usb_cam.launch`}</CodeBlock>

        <p>
          Em seguida, execute o código de identificação de cor e posição do
          objeto:
        </p>

        <CodeBlock>{`cd ~/catkin_ws/src
python3 nome_do_codigo_de_cor_e_posicao.py`}</CodeBlock>

        <a
          className="botaoDocumento"
          href={`${repoBase}/tree/main/Identificando%20cor%20e%20posicao%20do%20objeto`}
          target="_blank"
          rel="noreferrer"
        >
          Acessar código de cor e posição
        </a>
      </div>

      <div className="sistemaSection">
        <h2>5. Código de calibração HSV</h2>

        <p>
          O código de calibração é uma das partes mais importantes do projeto.
          Ele permite descobrir os valores HSV de cada cor usada no sistema,
          como a cor da bola e as cores dos marcadores posicionados sobre o
          robô.
        </p>

        <p>
          Na prática, o usuário coloca o robô ou a bola na frente da câmera e
          ajusta as barras deslizantes de <strong>H</strong>,{" "}
          <strong>S</strong> e <strong>V</strong>. À medida que os valores são
          alterados, uma máscara é criada em tempo real. O que não pertence à
          cor desejada fica preto, e a cor escolhida permanece destacada na
          imagem.
        </p>

        <p>
          Esse processo deve ser feito com calma. A iluminação do ambiente, a
          posição da câmera, o brilho do campo e a tonalidade dos objetos podem
          alterar os valores ideais. Por isso, a calibração deve ser feita no
          próprio ambiente de teste, com a câmera na posição real de uso.
        </p>

        <ImageCard
          src="/images/hsv-espectro.png"
          alt="Espectro do sistema de cores HSV"
          caption="Sistema HSV: H representa a cor, S representa a saturação e V representa o brilho."
        />

        <ImageCard
          src="/images/mascaras-hsv.png"
          alt="Máscaras HSV aplicadas aos objetos"
          caption="Máscaras HSV ajustadas em tempo real para isolar a bola e os marcadores do robô."
        />

        <p>
          O objetivo da calibração é encontrar os limites inferiores e superiores
          da cor desejada. Esses limites geralmente aparecem no código como
          variáveis do tipo <strong>lower</strong> e <strong>upper</strong>.
        </p>

        <CodeBlock label="Exemplo conceitual">{`lower_blue = np.array([H_min, S_min, V_min])
upper_blue = np.array([H_max, S_max, V_max])

mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)`}</CodeBlock>

        <p>
          Depois que os valores estiverem ajustados, eles devem ser copiados
          para o algoritmo principal. Assim, o sistema passa a reconhecer
          corretamente a bola e os marcadores do robô durante a execução real.
        </p>

        <a
          className="botaoDocumento"
          href={`${repoBase}/tree/main/calibration`}
          target="_blank"
          rel="noreferrer"
        >
          Acessar código de calibração
        </a>
      </div>

      <div className="sistemaSection">
        <h2>6. Fluxograma real gerado pelo ROS</h2>

        <p>
          O ROS permite visualizar a comunicação entre os nós do sistema usando
          o <strong>rqt_graph</strong>. Essa ferramenta mostra quais nós estão
          ativos e quais tópicos estão sendo utilizados para troca de mensagens.
          Nesta plataforma, a imagem exibida deve ser o fluxograma real gerado
          pelo ROS durante a execução do sistema.
        </p>

        <ImageCard
          src="/images/fluxograma-ros-rqtgraph.png"
          alt="Fluxograma do sistema gerado pelo rqt_graph"
          caption="Fluxograma gerado pelo próprio ROS por meio da ferramenta rqt_graph."
        />

        <p>Para abrir essa visualização, execute:</p>

        <CodeBlock>{`rqt_graph`}</CodeBlock>

        <p>Caso o comando acima não funcione diretamente, utilize:</p>

        <CodeBlock>{`rosrun rqt_graph rqt_graph`}</CodeBlock>

        <p>
          De forma geral, a câmera publica a imagem em um tópico do ROS. O
          algoritmo de visão recebe essa imagem, processa os dados, calcula os
          comandos e publica as velocidades que serão enviadas ao robô.
        </p>

        <div className="fluxoSistema fluxoClaro">
          <span>/usb_cam</span>
          <strong>→</strong>
          <span>/usb_cam/image_raw</span>
          <strong>→</strong>
          <span>/RoboESP32</span>
          <strong>→</strong>
          <span>/velocidade_motores</span>
          <strong>→</strong>
          <span>WeMos ESP8266</span>
        </div>
      </div>

      <div className="sistemaSection">
        <h2>7. Detecção da posição e orientação do robô</h2>

        <p>
          Depois de calibrar as cores, o sistema calcula a posição dos objetos
          detectados. Para isso, são usadas as centróides, que representam o
          centro geométrico de cada objeto identificado na imagem.
        </p>

        <p>
          No robô, um marcador colorido é usado para indicar sua posição, e outro
          marcador ajuda a identificar sua orientação. A partir desses pontos, o
          algoritmo consegue saber onde o robô está e para onde ele está
          apontando.
        </p>

        <ImageCard
          src="/images/orientacao-robo-bola.png"
          alt="Orientação do robô em relação à bola"
          caption="Setas utilizadas para determinar a orientação do robô em relação à bola."
        />

        <p>
          A seta que representa a direção do robô é comparada com a seta que vai
          do robô até a bola. O ângulo entre essas duas direções é o erro usado
          pelo controle. Se esse erro for positivo ou negativo, o controlador
          altera a velocidade das rodas para corrigir a orientação.
        </p>

        <ImageCard
          src="/images/plano-robo-diferencial.png"
          alt="Plano de um robô diferencial"
          caption="Representação do robô diferencial no plano, com posição x, y e orientação θ."
        />
      </div>

      <div className="sistemaSection">
        <h2>8. Detecção das linhas do campo</h2>

        <p>
          Além da bola e do robô, o sistema também precisa reconhecer regiões do
          campo, principalmente a área do goleiro. Para isso, a imagem é
          convertida para tons de cinza e depois é aplicado o filtro{" "}
          <strong>Canny</strong>, que destaca as bordas presentes na imagem.
        </p>

        <ImageCard
          src="/images/filtro-canny-campo.png"
          alt="Filtro Canny aplicado ao campo"
          caption="Filtro Canny aplicado para destacar as bordas das linhas do campo."
        />

        <p>
          Após a detecção das bordas, o algoritmo usa contornos para encontrar
          formas específicas. A área do goleiro é identificada verificando
          características como formato retangular, número de vértices e tamanho
          aproximado da região.
        </p>

        <ImageCard
          src="/images/contorno-goleiro.png"
          alt="Contorno da área do goleiro"
          caption="Detecção de contornos aplicada na área do goleiro."
        />
      </div>

      <div className="sistemaSection">
        <h2>9. Controle PID aplicado ao robô</h2>

        <p>
          O controle PID foi utilizado para corrigir a orientação do robô em
          relação à bola. O erro do sistema é o ângulo entre a direção atual do
          robô e a direção desejada. Quando o robô está alinhado com a bola, o
          erro tende a zero.
        </p>

        <ImageCard
          src="/images/pid-robo-bola.png"
          alt="Controle PID aplicado ao robô"
          caption="Exemplo de correção angular do robô em relação à bola."
        />

        <p>
          No código, o resultado do PID é armazenado na variável{" "}
          <strong>velocity</strong>. Essa variável altera a velocidade das rodas:
          uma roda recebe <strong>max_velocity - velocity</strong> e a outra
          recebe <strong>max_velocity + velocity</strong>. Essa diferença faz o
          robô girar e corrigir sua direção.
        </p>

        <ImageCard
          src="/images/codigo-pid-velocidade.png"
          alt="Código do controle de velocidade dos motores"
          caption="Trecho do código responsável por calcular a velocidade diferencial dos motores."
        />
      </div>

      <div className="sistemaSection">
        <h2>10. Estratégia do robô atacante</h2>

        <p>
          A estratégia do atacante foi organizada em estados. Quando o robô está
          longe da bola, ele entra no estado <strong>GO_TO_BALL</strong>, que faz
          com que ele se direcione até a bola. Quando ele se aproxima o
          suficiente, muda para o estado <strong>GO_TO_GOAL</strong>, levando a
          bola em direção ao gol.
        </p>

        <ImageCard
          src="/images/estrategia-atacante.png"
          alt="Estratégia GO_TO_BALL e GO_TO_GOAL"
          caption="Estados GO_TO_BALL e GO_TO_GOAL aplicados ao robô atacante."
        />

        <p>
          Também foi criada uma verificação para identificar quando o robô fica
          parado, por exemplo, ao colidir com uma parede. O algoritmo compara a
          posição atual com a posição anterior. Se a posição não muda por um
          tempo determinado, o sistema entende que o robô está preso e executa
          uma ação de correção.
        </p>

        <ImageCard
          src="/images/codigo-robo-parado.png"
          alt="Código para verificar se o robô está parado"
          caption="Algoritmo utilizado para verificar se o robô ficou parado."
        />
      </div>

      <div className="sistemaSection">
        <h2>11. Estratégia do goleiro</h2>

        <p>
          O goleiro utiliza uma estratégia diferente. Quando a bola está longe,
          ele permanece ou retorna para a área de defesa por meio do estado{" "}
          <strong>RETURN_TO_GOAL</strong>. Quando a bola se aproxima da área, o
          goleiro muda para <strong>GO_TO_BALL</strong>, avança até a bola e
          tenta afastá-la do gol.
        </p>

        <ImageCard
          src="/images/goleiro-esperando.png"
          alt="Goleiro esperando um ataque"
          caption="Goleiro posicionado na área de defesa aguardando aproximação da bola."
        />
      </div>

      <div className="sistemaSection">
        <h2>12. Resultados observados no RViz</h2>

        <p>
          Os testes foram acompanhados no RViz. A visualização das trajetórias
          permitiu verificar se o robô atacante estava indo até a bola, se
          conduzia a bola ao gol, se conseguia sair de obstáculos e se o goleiro
          retornava corretamente para a área de defesa.
        </p>

        <div className="resultadoGrid">
          <ImageCard
            src="/images/resultado-ataque.png"
            alt="Resultado da estratégia de ataque"
            caption="Resultado I: estratégia de ataque."
          />

          <ImageCard
            src="/images/resultado-obstaculo.png"
            alt="Resultado da estratégia de obstáculo"
            caption="Resultado II: correção quando o robô fica preso."
          />

          <ImageCard
            src="/images/resultado-goleiro.png"
            alt="Resultado da estratégia do goleiro"
            caption="Resultado III: estratégia do goleiro."
          />
        </div>
      </div>

      <div className="sistemaSection destaque">
        <h2>13. Como rodar o sistema completo</h2>

        <p>
          Para executar o sistema completo, é necessário seguir uma ordem de
          inicialização. Primeiro, o robô deve estar com o código correto gravado
          no ESP8266. Depois, no computador, deve-se iniciar o ROS Master,
          executar a câmera USB e, por fim, rodar o código correspondente ao robô
          que será testado.
        </p>

        <p>
          No sistema final, os códigos principais estão na pasta{" "}
          <strong>Programas Finais</strong> do repositório. Para testar o
          goleiro, utiliza-se o arquivo <strong>goleiro.py</strong>. Para testar
          o jogador de linha, utiliza-se o arquivo <strong>jogador.py</strong>.
        </p>

        <p>
          Antes de rodar os códigos no computador, o ESP8266 do robô precisa
          estar com o código embarcado correto. O código que deve ser enviado
          para o ESP está na pasta:
        </p>

        <CodeBlock label="Caminho no GitHub">{`Controle do robo pelo teclado - WASD/Codigo_para_ESP`}</CodeBlock>

        <p>
          Esse código é responsável por permitir que o robô receba os comandos
          enviados pelo sistema e acione os motores. Sem essa etapa, o
          computador até pode processar a imagem e calcular os comandos, mas o
          robô não irá executar corretamente os movimentos.
        </p>

        <a
          className="botaoDocumento"
          href={`${repoBase}/tree/main/Controle%20do%20robo%20pelo%20teclado%20-%20WASD/Codigo_para_ESP`}
          target="_blank"
          rel="noreferrer"
        >
          Acessar código para o ESP8266
        </a>

        <hr className="divisorInterno" />

        <h3>Ordem de execução no computador</h3>

        <p>
          Com o robô ligado, conectado à mesma rede do computador e com o código
          já gravado no ESP8266, execute os passos abaixo.
        </p>

        <p>1. Abra o terminal e inicie o ROS Master:</p>

        <CodeBlock>{`roscore`}</CodeBlock>

        <p>
          Esse comando inicia o servidor principal do ROS. Ele deve permanecer
          aberto durante todo o funcionamento do sistema.
        </p>

        <p>2. Abra um novo terminal e execute a câmera USB:</p>

        <CodeBlock>{`roslaunch usb_cam usb_cam.launch`}</CodeBlock>

        <p>
          Esse comando inicializa o pacote da câmera e publica as imagens
          capturadas em um tópico do ROS.
        </p>

        <p>3. Abra um novo terminal e verifique se os tópicos foram criados:</p>

        <CodeBlock>{`rostopic list`}</CodeBlock>

        <p>
          Verifique se aparecem tópicos relacionados à câmera, como{" "}
          <strong>/usb_cam/image_raw</strong>. Isso indica que a câmera está
          publicando as imagens corretamente.
        </p>

        <p>4. Acesse a pasta dos programas finais:</p>

        <CodeBlock>{`cd ~/catkin_ws/src/Programas\\ Finais`}</CodeBlock>

        <p>
          Caso a pasta esteja em outro local no seu computador, ajuste o caminho
          de acordo com a organização do seu workspace.
        </p>

        <p>5. Para testar o robô goleiro, execute:</p>

        <CodeBlock>{`python3 goleiro.py`}</CodeBlock>

        <p>
          Esse código executa a estratégia do goleiro. O robô deve permanecer ou
          retornar para a área de defesa e avançar até a bola quando ela se
          aproxima da região do gol.
        </p>

        <p>6. Para testar o jogador de linha, execute:</p>

        <CodeBlock>{`python3 jogador.py`}</CodeBlock>

        <p>
          Esse código executa a estratégia do jogador. O robô deve identificar a
          bola, se orientar em relação a ela e executar a movimentação definida
          no algoritmo.
        </p>

        <div className="caixaObservacao">
          <h3>Observação sobre formação de equipe</h3>

          <p>
            Os testes com <strong>goleiro.py</strong> e{" "}
            <strong>jogador.py</strong> podem ser feitos separadamente. Para
            criar uma equipe completa com goleiro e jogadores atuando em
            conjunto, será necessário desenvolver novas estratégias de
            coordenação entre os robôs. Essa etapa fica como continuidade do
            projeto para quem estiver reproduzindo e aprimorando o sistema.
          </p>

          <p>
            A partir desse ponto, o pesquisador pode começar a desenvolver
            estratégias mais avançadas, como divisão de funções entre os robôs,
            tomada de decisão em equipe, controle de posicionamento e futuramente
            aplicações com redes neurais ou aprendizado por reforço.
          </p>
        </div>

        <div className="grupoBotoes">
          <a
            className="botaoDocumento"
            href={`${repoBase}/tree/main/Programas%20Finais`}
            target="_blank"
            rel="noreferrer"
          >
            Acessar Programas Finais
          </a>

          <a
            className="botaoDocumento"
            href={repoBase}
            target="_blank"
            rel="noreferrer"
          >
            Acessar repositório completo
          </a>
        </div>
      </div>
    </section>
  );
}