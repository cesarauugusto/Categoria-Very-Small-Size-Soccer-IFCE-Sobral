export default function Footer() {
  return (
    <footer className="footerIfce">
      <div className="footerLogos">
        <img src={`${import.meta.env.BASE_URL}images/ieee.png`} alt="Logo da IEEE" />
        <img src={`${import.meta.env.BASE_URL}images/logo-ifce.png`} alt="Logo do IFCE" />
      </div>

      <div>
        <h2>VSSS Open Platform</h2>
        <p>
          Plataforma open source para documentação e reprodução de um sistema
          Very Small Size Soccer, reunindo estrutura física, ambiente ROS, visão
          computacional, controle, códigos de teste e estratégias de
          movimentação.
        </p>
      </div>

      <div className="footerLinks">
        <a href="#topo">Voltar ao topo</a>
        <a href="#ambiente-ros">Ambiente ROS</a>
        <a href="#sistema">Funcionamento</a>
        <a href="#campo-estrutura">Campo e estrutura</a>
        <a href="#autores">Autores</a>
        <a href="#referencias">Referências</a>
      </div>
    </footer>
  );
}