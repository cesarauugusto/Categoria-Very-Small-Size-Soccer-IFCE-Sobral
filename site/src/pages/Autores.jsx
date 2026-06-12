export default function Autores() {
  return (
    <section className="secao" id="autores">
      <p className="tag">Equipe do projeto</p>

      <h2>Autores e orientadores</h2>

      <p>
        Esta plataforma foi organizada para documentar o desenvolvimento do
        sistema Very Small Size Soccer e facilitar sua reprodução por estudantes,
        pesquisadores e novos integrantes do projeto.
      </p>

      <div className="autoresGrid">
        <div className="autorCard destaqueAutor">
          <span>Autor</span>
          <h3>César Augusto</h3>
          <p>
            Responsável pelo desenvolvimento do projeto, organização da
            documentação, implementação dos algoritmos, testes do sistema e
            estruturação da plataforma open source.
          </p>
        </div>

        <div className="autorCard">
          <span>Orientador</span>
          <h3>Prof. Me. Aldinei Aragão</h3>
          <p>
            Orientação técnica e acadêmica no desenvolvimento do sistema,
            contribuindo para a estruturação metodológica e validação do projeto.
          </p>
        </div>

        <div className="autorCard">
          <span>Coorientador</span>
          <h3>Prof. Me. Leonardo Tabosa</h3>
          <p>
            Apoio na orientação do projeto, contribuindo para o desenvolvimento,
            análise técnica e aprimoramento da solução proposta.
          </p>
        </div>
      </div>

      <div className="notaIfce">
        <strong>Instituição:</strong> Instituto Federal de Educação, Ciência e
        Tecnologia do Ceará — IFCE, Campus Sobral.
      </div>
    </section>
  );
}