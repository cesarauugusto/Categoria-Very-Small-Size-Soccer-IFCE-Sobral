import "./AmbienteROS.css";

const imagePath = (fileName) => `${import.meta.env.BASE_URL}images/${fileName}`;

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

function Step({ number, title, children }) {
  return (
    <div className="step">
      <div className="stepNumber">{number}</div>
      <div className="stepContent">
        <h3>{title}</h3>
        {children}
      </div>
    </div>
  );
}

export default function AmbienteROS() {
  return (
    <section className="ambientePage" id="ambiente-ros">
      <div className="ambienteHeader">
        <p className="tag">Ambiente de desenvolvimento</p>

        <h2>Instalação do ROS e configuração da câmera USB</h2>

        <p>
          Esta seção apresenta o processo utilizado para preparar o ambiente de
          desenvolvimento do projeto VSSS. A etapa inclui a instalação do ROS
          Noetic, a configuração do ambiente no Ubuntu, a instalação do pacote
          usb_cam e a validação da câmera USB no RViz.
        </p>

        <p>
          Essa configuração é essencial porque a câmera é responsável por
          capturar as imagens do campo em tempo real. Essas imagens são
          publicadas em tópicos do ROS e utilizadas posteriormente no
          processamento com OpenCV para identificar a bola, o robô e regiões do
          campo.
        </p>
      </div>

      <div className="ambienteGridInfo">
        <div>
          <h3>Sistema utilizado</h3>
          <p>Ubuntu 20.04 LTS</p>
        </div>

        <div>
          <h3>Distribuição ROS</h3>
          <p>ROS Noetic</p>
        </div>

        <div>
          <h3>Captura de imagem</h3>
          <p>Pacote usb_cam</p>
        </div>

        <div>
          <h3>Visualização</h3>
          <p>RViz e rqt_image_view</p>
        </div>
      </div>

      <div className="ambienteSection">
        <h2>1. Escolha da versão do ROS e do Ubuntu</h2>

        <p>
          Antes de instalar o ROS, é necessário verificar qual versão do Linux
          está instalada, pois cada distribuição do ROS possui compatibilidade
          com versões específicas do Ubuntu.
        </p>

        <p>
          Neste projeto foi utilizado o <strong>Ubuntu 20.04 LTS</strong>, pois
          ele é compatível com o <strong>ROS Noetic</strong>. Essa combinação foi
          escolhida por apresentar maior compatibilidade com os pacotes
          utilizados no sistema, como o <strong>usb_cam</strong>, o{" "}
          <strong>RViz</strong> e as ferramentas de integração com visão
          computacional.
        </p>

        <p>
          Portanto, para reproduzir o ambiente utilizado no projeto, recomenda-se
          usar o Ubuntu 20.04 LTS com o ROS Noetic.
        </p>

        <div className="linkList">
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
            noetic/Installation/Ubuntu - ROS Wiki
          </a>
        </div>
      </div>

      <div className="ambienteSection">
        <h2>2. Instalação do ROS Noetic</h2>

        <Step number="01" title="Atualizar os pacotes do Ubuntu">
          <p>
            O primeiro passo é atualizar os índices de pacotes do sistema
            operacional. Isso garante que o Ubuntu reconheça as versões mais
            recentes disponíveis nos repositórios.
          </p>

          <p>Abra o terminal e execute:</p>

          <CodeBlock>{`sudo apt-get update`}</CodeBlock>
        </Step>

        <Step number="02" title="Instalar o Curl">
          <p>
            Durante a instalação do ROS, é necessário utilizar o Curl para
            adicionar a chave do repositório oficial.
          </p>

          <p>Abra o terminal e execute:</p>

          <CodeBlock>{`sudo apt install curl`}</CodeBlock>

          <p>Depois, ainda no terminal, execute o comando abaixo:</p>

          <CodeBlock>{`curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`}</CodeBlock>
        </Step>

        <Step number="03" title="Instalar a versão completa do ROS">
          <p>
            Na instalação, recomenda-se utilizar a versão completa do ROS Noetic.
            Essa versão instala os pacotes principais, bibliotecas, ferramentas
            gráficas e recursos de visualização 2D e 3D.
          </p>

          <p>Abra o terminal e execute:</p>

          <CodeBlock>{`sudo apt install ros-noetic-desktop-full`}</CodeBlock>
        </Step>

        <Step number="04" title="Carregar o ambiente do ROS">
          <p>
            Para utilizar o ROS no terminal, é necessário carregar o arquivo de
            configuração do ambiente.
          </p>

          <p>Abra o terminal e execute:</p>

          <CodeBlock>{`source /opt/ros/noetic/setup.bash`}</CodeBlock>

          <p>
            Para não precisar repetir esse comando toda vez que abrir o
            terminal, abra o arquivo <strong>.bashrc</strong>.
          </p>

          <p>Abra o terminal e execute:</p>

          <CodeBlock>{`sudo gedit ~/.bashrc`}</CodeBlock>

          <p>No final do arquivo, adicione esta linha:</p>

          <CodeBlock label="Arquivo .bashrc">{`source /opt/ros/noetic/setup.bash`}</CodeBlock>

          <div className="imagemExplicativa">
            <img
              src={imagePath("bashrc-source-ros.png")}
              alt="Comando source adicionado ao final do arquivo .bashrc"
            />
            <p>
              Exemplo do comando <strong>source</strong> adicionado ao final do
              arquivo <strong>.bashrc</strong> para carregar automaticamente o
              ambiente do ROS ao abrir o terminal.
            </p>
          </div>

          <p>Depois, salve e feche o arquivo.</p>
        </Step>

        <Step number="05" title="Testar a instalação do ROS">
          <p>
            Para verificar se a instalação foi concluída corretamente, é
            necessário iniciar o servidor principal do ROS.
          </p>

          <p>Abra o terminal e execute:</p>

          <CodeBlock>{`roscore`}</CodeBlock>

          <p>
            Se o servidor ROS Master inicializar sem erros, a instalação foi
            concluída corretamente.
          </p>
        </Step>
      </div>

      <div className="ambienteSection">
        <h2>3. Instalação do pacote usb_cam</h2>

        <Step number="01" title="Instalar o pacote da câmera">
          <p>
            Após instalar o ROS, é necessário instalar o pacote responsável por
            capturar imagens de câmeras USB.
          </p>

          <p>O comando geral é:</p>

          <CodeBlock>{`sudo apt-get install ros-<distro>-usb-cam`}</CodeBlock>

          <p>
            No lugar de <strong>&lt;distro&gt;</strong>, deve-se colocar a
            versão do ROS instalada no sistema. No projeto, como foi utilizado o
            ROS Noetic, o comando ficou:
          </p>

          <p>Abra o terminal e execute:</p>

          <CodeBlock>{`sudo apt-get install ros-noetic-usb-cam`}</CodeBlock>
        </Step>

        <Step number="02" title="Instalar a dependência libv4l">
          <p>
            Também foi instalada a dependência <strong>libv4l-dev</strong>, que
            auxilia no suporte de dispositivos de vídeo no Linux.
          </p>

          <p>Abra o terminal e execute:</p>

          <CodeBlock>{`sudo apt-get install libv4l-dev`}</CodeBlock>
        </Step>

        <Step number="03" title="Baixar o driver usb_cam pelo GitHub">
          <p>
            O pacote usb_cam também pode ser baixado diretamente do repositório
            oficial. Para isso, é necessário acessar a pasta <strong>src</strong>{" "}
            do workspace Catkin.
          </p>

          <p>Abra o terminal e execute:</p>

          <CodeBlock>{`cd ~/catkin_ws/src`}</CodeBlock>

          <p>Depois, ainda no terminal, clone o repositório:</p>

          <CodeBlock>{`git clone https://github.com/ros-drivers/usb_cam.git`}</CodeBlock>

          <p>
            Esse comando baixa o código-fonte do usb_cam para dentro do
            workspace do ROS.
          </p>

          <div className="linkList">
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
              GitHub - ros-drivers/usb_cam
            </a>
          </div>
        </Step>

        <Step number="04" title="Compilar o workspace Catkin">
          <p>
            Após baixar o pacote, volte para a raiz do workspace e compile os
            pacotes.
          </p>

          <p>Abra o terminal e execute:</p>

          <CodeBlock>{`cd ~/catkin_ws
catkin_make`}</CodeBlock>

          <p>Depois da compilação, carregue o ambiente do workspace:</p>

          <CodeBlock>{`source ~/catkin_ws/devel/setup.bash`}</CodeBlock>
        </Step>
      </div>

      <div className="ambienteSection">
        <h2>4. Configuração do arquivo usb_cam.launch</h2>

        <p>
          O arquivo <strong>usb_cam.launch</strong> define os parâmetros usados
          para inicializar a câmera no ROS, como dispositivo de vídeo, resolução,
          formato da imagem e taxa de quadros.
        </p>

        <p>Abra o terminal e execute:</p>

        <CodeBlock>{`gedit ~/catkin_ws/src/usb_cam/launch/usb_cam.launch`}</CodeBlock>

        <p>Apague o conteúdo existente e cole a configuração abaixo:</p>

        <CodeBlock label="Arquivo usb_cam.launch">{`<launch>
  <node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen">
    <rosparam command="load" file="$(find usb_cam)/config/usb_cam.yml" />

    <param name="video_device" value="/dev/video1" />
    <param name="image_width" value="640" />
    <param name="image_height" value="480" />
    <param name="pixel_format" value="mjpeg" />
    <param name="io_method" value="mmap" />
    <param name="frame_rate" value="30" />
    <param name="autofocus" value="false" />
    <param name="autogain" value="true" />
  </node>
</launch>`}</CodeBlock>

        <div className="parameterGrid">
          <div>
            <h3>video_device</h3>
            <p>
              Define o canal da câmera. Câmeras integradas geralmente usam{" "}
              <strong>/dev/video0</strong>. Câmeras USB externas podem usar{" "}
              <strong>/dev/video1</strong> ou <strong>/dev/video2</strong>.
            </p>
          </div>

          <div>
            <h3>image_width e image_height</h3>
            <p>
              Definem a resolução da imagem. Neste projeto foi utilizada a
              resolução <strong>640x480</strong>.
            </p>
          </div>

          <div>
            <h3>pixel_format</h3>
            <p>
              Define o formato de imagem compatível com a câmera e com o ROS.
              Neste caso, foi usado <strong>mjpeg</strong>.
            </p>
          </div>

          <div>
            <h3>frame_rate</h3>
            <p>
              Define a taxa de quadros por segundo. O valor utilizado foi{" "}
              <strong>30 FPS</strong>.
            </p>
          </div>
        </div>

        <p>
          Para verificar quais câmeras estão conectadas ao sistema, abra o
          terminal e execute:
        </p>

        <CodeBlock>{`ls /dev/video*`}</CodeBlock>
      </div>

      <div className="ambienteSection">
        <h2>5. Teste da câmera no ROS</h2>

        <Step number="01" title="Iniciar o ROS Master">
          <p>
            Primeiro, é necessário iniciar o servidor central do ROS. Ele é
            responsável pela comunicação entre os nós do sistema.
          </p>

          <p>Abra o terminal e execute:</p>

          <CodeBlock>{`roscore`}</CodeBlock>
        </Step>

        <Step number="02" title="Executar o pacote usb_cam">
          <p>
            Com o ROS Master em execução, abra outro terminal para inicializar a
            câmera.
          </p>

          <p>Abra um novo terminal e execute:</p>

          <CodeBlock>{`roslaunch usb_cam usb_cam-test.launch`}</CodeBlock>

          <p>Ou, caso esteja usando o arquivo editado, execute:</p>

          <CodeBlock>{`roslaunch usb_cam usb_cam.launch`}</CodeBlock>
        </Step>

        <Step number="03" title="Verificar os tópicos publicados">
          <p>
            Para verificar se a câmera está publicando imagens no ROS, abra um
            novo terminal e execute:
          </p>

          <CodeBlock>{`rostopic list`}</CodeBlock>

          <p>Se estiver funcionando, devem aparecer tópicos como:</p>

          <CodeBlock>{`/usb_cam/camera_info
/usb_cam/image_raw
/usb_cam/image_raw/compressed
/usb_cam/image_raw/theora`}</CodeBlock>

          <p>O tópico principal utilizado para a imagem da câmera é:</p>

          <CodeBlock>{`/usb_cam/image_raw`}</CodeBlock>
        </Step>
      </div>

      <div className="ambienteSection">
        <h2>6. Visualização da câmera no RViz</h2>

        <p>
          O RViz é uma ferramenta gráfica do ROS utilizada para visualizar dados
          de sensores, imagens, transformações e outros elementos do sistema
          robótico.
        </p>

        <p>Abra o terminal e execute:</p>

        <CodeBlock>{`rviz`}</CodeBlock>

        <ol className="instructionList">
          <li>
            No RViz, clique em <strong>Add</strong>.
          </li>
          <li>
            Selecione a opção <strong>Image</strong>.
          </li>
          <li>
            No campo <strong>Image Topic</strong>, selecione ou digite{" "}
            <strong>/usb_cam/image_raw</strong>.
          </li>
          <li>
            Clique em <strong>OK</strong>.
          </li>
          <li>A imagem da câmera deverá aparecer na tela.</li>
        </ol>

        <p>
          Outra forma de visualizar a imagem publicada pela câmera é utilizando o
          rqt_image_view.
        </p>

        <p>Abra o terminal e execute:</p>

        <CodeBlock>{`rosrun rqt_image_view rqt_image_view`}</CodeBlock>
      </div>
    </section>
  );
}