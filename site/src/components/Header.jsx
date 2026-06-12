export default function Header() {
  return (
    <header className="menuSuperior">
      <div className="menuConteudo">
        <a href="#topo" className="menuLogo">
          <div> <img src={`${import.meta.env.BASE_URL}images/logo-ifce-sozinha.png`}/></div>
          <div>
  
          <span>VSSS</span>
          <span> Open Platform</span>
          </div>
          
        </a>

        <nav className="menuLinks">
          <a href="#sobre">Sobre</a>
          <a href="#reproducao">Reprodução</a>
          <a href="#campo-estrutura">Campo</a>
          <a href="#ambiente-ros">ROS</a>
          <a href="#sistema">Sistema</a>
          <a href="#autores">Autores</a>
          <a href="#referencias">Referências</a>
          <a href="#recursos">Recursos</a>
        </nav>
      </div>
    </header>
  );
}