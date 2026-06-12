const referencias = [
  {
    autor: "AUGUSTO, C.",
    titulo: "Categoria Very Small Size Soccer IFCE Sobral.",
    fonte: "GitHub",
    ano: "2024",
    link: "https://github.com/cesarauugusto/Categoria-Very-Small-Size-Soccer-IFCE-Sobral",
  },
  {
    autor: "BERTOCINI, J. P. S.",
    titulo: "Coordenação de robô autônomo por meio de visão computacional.",
    fonte: "Universidade Tecnológica Federal do Paraná, Cornélio Procópio",
    ano: "2022",
    link: "https://repositorio.utfpr.edu.br/jspui/bitstream/1/29473/1/coordenacaoroboautonomovisao.pdf",
  },
  {
    autor: "FAUZI, A.; SYARIF, I.; BADRIYAH, T.",
    titulo:
      "Development of a Mobile Application for Plant Disease Detection using Parameter Optimization Method in Convolutional Neural Networks Algorithm.",
    fonte: "International Journal of Engineering Technology, p. 192-213",
    ano: "2023",
    link: "",
  },
  {
    autor:
      "FERREIRA, R. H. X.; SILVA, P. L.; OLIVEIRA, V. C. R.; JUNIOR, C. D. S.; FREITAS, E. J. R.",
    titulo:
      "Initial development of IEEE Very Small Size Soccer robots: Electromechanics, Computer Vision System and Navigation Strategy.",
    fonte: "The Journal of Engineering and Exact Sciences, p. 18148",
    ano: "2023",
    link: "",
  },
  {
    autor: "FREITAS, E. J. R.; PASSOS, H. A.; PEREIRA, G. A. S.",
    titulo:
      "Desvio de obstáculos por robôs semiautônomos usando planejamento de caminhos.",
    fonte: "XIII Simpósio Brasileiro de Automação Inteligente, p. 1043–1048",
    ano: "2017",
    link: "",
  },
  {
    autor: "HOOPES, D. M.; DAVIS, T.; NORMAN, K.; HELPS, R. G.",
    titulo:
      "An autonomous mobile robot development platform for teaching a graduate level mechatronics course.",
    fonte: "33rd Annual Frontiers in Education, F4E–17",
    ano: "2003",
    link: "",
  },
  {
    autor:
      "KITANO, H.; ASADA, M.; KUNIYOSHI, Y.; NODA, I.; OSAWA, E.; MATSUBARA, H.",
    titulo: "Robocup: A challenge problem for AI.",
    fonte: "AI Magazine, p. 73–73",
    ano: "1997",
    link: "",
  },
  {
    autor: "OGATA, K.",
    titulo: "Modern Control Engineering.",
    fonte: "5ª edição",
    ano: "2009",
    link: "",
  },
  {
    autor: "OLIVEIRA, B. V. N.; MELO, F. T.",
    titulo:
      "Fundamentos da visão computacional: arcabouço teórico do reconhecimento artificial de imagens e vídeos.",
    fonte: "Revista Humanidades e Inovação, p. 312-319",
    ano: "2023",
    link: "",
  },
  {
    autor: "PINTO, A. H. M.",
    titulo: "Regras IEEE Very Small Size Soccer (VSSS) - Série B.",
    fonte: "CBR RoboCup Brasil",
    ano: "2021",
    link: "https://cbr.robocup.org.br/wp-content/uploads/2021/05/vssRules3x321.pdf",
  },
  {
    autor: "PINTO, E. S.",
    titulo:
      "Desenvolvimento de robôs capazes de disputar uma partida de futebol da categoria IEEE Very Small Size.",
    fonte: "Escola de Minas, Universidade Federal de Ouro Preto",
    ano: "2017",
    link: "http://www.monografias.ufop.br/handle/35400000/491",
  },
  {
    autor: "POLOLU.",
    titulo: "Micro Metal Gearmotors.",
    fonte: "Pololu",
    ano: "2024",
    link: "https://www.pololu.com/category/60/micro-metal-gearmotors",
  },
  {
    autor: "QUEIROZ, L. F. B.",
    titulo: "Modelagem e controle PID de sistema automotivo de suspensão ativa.",
    fonte: "Universidade Federal de Ouro Preto",
    ano: "2023",
    link: "https://www.monografias.ufop.br/bitstream/35400000/5375/9/MONOGRAFIA_ModelagemControlePID.pdf",
  },
  {
    autor: "ROSA, J. F.",
    titulo:
      "Construção de um time de futebol de robôs para a categoria IEEE Very Small Size.",
    fonte: "Faculdade de Educação Tecnológica do Estado do Rio de Janeiro",
    ano: "2015",
    link: "https://sirlab.github.io/assets/pdfs/tcc-johnathan.pdf",
  },
  {
    autor: "SANTOS, P. H.; FILHO, A. M. S. T.; FREITAS, E. J. R.",
    titulo:
      "Desenvolvimento de robôs diferenciais futebolistas: uma aplicação de campos vetoriais no planejamento de caminho.",
    fonte: "XIV Simpósio Brasileiro de Automação Inteligente, p. 1313-1318",
    ano: "2019",
    link: "",
  },
  {
    autor: "SILVA, L. L. R.; ROCHA, P. F. F.; OTTONI, A. L. C.; BITTENCOURT, J. C. N.",
    titulo:
      "Modelagem e Controle PID de Robôs Móveis Aplicados à Categoria Very Small Size de Futebol de Robôs.",
    fonte: "Sociedade Brasileira de Computação, p. 97-106",
    ano: "2021",
    link: "",
  },
  {
    autor: "DATACADAMIA.",
    titulo:
      "What are the HSV, HSI and HSL color spaces? Hue, Saturation, Value, Intensity, Lightness.",
    fonte: "Data Analytics & Data Science",
    ano: "2024",
    link: "https://datacadamia.com/data/type/color/hsl",
  },
];

export default function Referencias() {
  return (
    <section className="secao referenciasSection" id="referencias">
      <p className="tag">Base teórica e materiais consultados</p>

      <h2>Referências</h2>

      <p>
        Esta seção reúne as principais referências utilizadas na construção do
        projeto, incluindo trabalhos sobre futebol de robôs, visão computacional,
        controle PID, regras da categoria IEEE Very Small Size Soccer, ROS,
        componentes eletrônicos e materiais de apoio.
      </p>

      <div className="referenciasLista">
        {referencias.map((ref, index) => (
          <article className="referenciaCard" key={`${ref.autor}-${index}`}>
            <span>{String(index + 1).padStart(2, "0")}</span>

            <div>
              <h3>{ref.autor}</h3>

              <p>
                <strong>{ref.titulo}</strong> {ref.fonte}. {ref.ano}.
              </p>

              {ref.link && (
                <a href={ref.link} target="_blank" rel="noreferrer">
                  Acessar referência
                </a>
              )}
            </div>
          </article>
        ))}
      </div>
    </section>
  );
}