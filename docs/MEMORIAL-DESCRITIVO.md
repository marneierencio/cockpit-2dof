# Memorial Descritivo — Cockpit de Simulação com Plataforma Móvel de 2 Graus de Liberdade

| Item | Descrição |
|---|---|
| Projeto | Cockpit de simulador para jogos, conversível entre **Fórmula 1** e **caminhão** |
| Usuário-alvo | 170–175 cm, 90 kg (premissa de cálculo) |
| Periféricos | Logitech **G920** (volante + pedais) e **Driving Force Shifter**; óculos de **realidade virtual** (sem monitores) |
| Movimento | 2 graus de liberdade (arfagem e rolagem) por dois motores de passo NEMA 34 com redutor |
| Estrutura | Tubo de aço retangular **30×50×1,2** (maior esforço), **20×30×2,0** (estrutura geral) e **10×20×1,2** (contraventamento leve) |
| Revisão | **R3** — anteprojeto verificado, pronto para revisão independente antes do desenho final |

### Histórico de revisões

| Rev. | Mudança |
|---|---|
| R0 | Modelo conceitual com as duas configurações sobrepostas (peças diferentes para F1 e caminhão) |
| R1 | Peças únicas: banco em quadrilátero articulado, pedaleira basculante e volante em braço pivotado, todos sobre trilhos. Periféricos substituídos pelo Logitech G920 + câmbio. Transição animada via componente dinâmico |
| R2 | Banco com subchassi nivelado, assento inclinável (25° no F1) e encosto em dois segmentos com apoio superior de ombros/cabeça articulado na clavícula |
| **R3** | Detalhada na lista a seguir |

Mudanças da **R3**:

- **Estrutura:**
  - **anel estrutural** treliçado a 930 mm do piso;
  - **barra da porta** removível que abre para cima;
  - **treliça de soleira** sob a longarina esquerda;
  - pivô com 3 travessas 30×50;
  - perfil **30×50×1,2** introduzido.
- **Balanceamento:** automático do CG sobre o pivô nos dois modos, com o conjunto do caminhão recuado.
- **Conversão em um movimento por módulo** (opção A, preparada para atuador único — opção B):
  - **tirante paralelo** no apoio superior;
  - hastes de acoplamento no banco e na pedaleira;
  - came no volante;
  - pinos de trava rápida.
- **Câmbio** fixo no anel.
- **Segurança e motores:**
  - **molas de centragem** de tração como freio econômico;
  - rolagem do F1 ampliada e do caminhão limitada por software.
- **Documentação e verificação:**
  - **identificação** de todas as peças;
  - **lista de materiais**;
  - **plano de corte**;
  - **cálculos**;
  - verificação geométrica automática.

---

## 1. Objetivo e status

Projetar um cockpit único que assume, só por articulações, trilhos e travas, a posição de pilotagem de um F1 e a de
um caminhão, sobre uma plataforma móvel de 2 GDL. A R3 fecha os pontos levantados na revisão da R2:

- resistência estrutural;
- centro de gravidade;
- conversão prática;
- freio;
- identificação e fabricação.

> **Status:** anteprojeto **verificado por cálculo e por checagem geométrica automática** (seção 12), mas ainda
> **não revisado por terceiro** nem testado em protótipo. Os valores de catálogo marcados **[Chutando]** em
> `CALCULOS.md` (curvas de torque, rendimento do redutor) precisam ser trocados pelos dos componentes comprados.

## 2. Documentos do projeto

| Documento | Conteúdo |
|---|---|
| `docs/MEMORIAL-DESCRITIVO.md` | Este memorial: concepção, mecanismos, decisões |
| `docs/CALCULOS.md` | Massas, CG, inércias, envoltória, molas, motores, tensões, pinos, frequências, verificação geométrica |
| `docs/LISTA_DE_MATERIAIS.md` (+ `lista_de_materiais.csv`) | Todas as peças com código, dimensões, corte e massa; comprados agrupados; itens não desenhados |
| `docs/PLANO_DE_CORTE.md` (+ `plano_de_corte.csv`) | Distribuição das peças de tubo em barras de 6 m |
| `modelo/gerar_modelo.py` | Gerador paramétrico (fonte única de toda a geometria e dos números) |
| `ferramentas/verificar.py` | Interferências (transição, porta, manequim) e envoltória de movimento contra a base |
| `ferramentas/analisar.py` | Cálculos, lista de materiais, plano de corte e o bloco `PRESET` do gerador |

Os três documentos de números são **gerados** a partir do modelo. Nenhum valor foi digitado à mão.

## 3. Sistema de coordenadas, convenções e identificação

- Milímetros. **X** lateral (direita do piloto = +X), **Y** longitudinal (piloto olha para +Y; traseira da
  plataforma móvel em Y = 0), **Z** vertical (piso em Z = 0).
- Topo das longarinas do chassi móvel: **ZP = 380 mm**.
- Ângulos sagitais medidos a partir de +Y em direção a +Z. "E" = lado esquerdo e "D" = lado direito do piloto.
- **Códigos de peça:** `<conjunto>-<tipo><nº>`, por exemplo `M-T01`, `BC-PI11` ou `VO-CO05`.
  - Conjuntos: **B** base, **M** chassi móvel, **A** anel, **PO** barra da porta, **BC** banco, **PE** pedaleira,
    **VO** volante, **MO** molas.
  - Tipos: **T** tubo, **CH** chapa/peça usinada, **PI** pino, **CO** comprado, **ES** estofado.
  - No SketchUp, cada peça se chama `<código> <descrição>` (janela *Outliner*). O mesmo código aparece na lista
    de materiais, no plano de corte e na etiqueta da peça na bancada.

## 4. Premissas

| Segmento do manequim | Comprimento (mm) |
|---|---|
| Coxa / perna | 420 / 420 |
| Tronco (quadril → ombros) | 520 |
| Braço / antebraço + mão | 300 / 310 |
| Meia-largura do quadril / ombros | 90 / 195 |

- Estatura resultante ≈ **1730 mm**.
- Massa do piloto **90 kg**, distribuída pelos segmentos (frações de Dempster) para o cálculo do CG.
- **Sem monitores:** a imersão é por óculos de RV, então o modelo não tem monitor nem suporte. O CG fica mais
  baixo e o volante não precisa deixar campo de visão para tela. A margem de visão sobre o aro foi mantida como
  critério (35 mm) por conforto.
- G920: dimensões externas oficiais. As proporções internas (seção do aro, posição dos pedais) são estimadas.

## 5. Descrição geral

```
                ANEL (z = 930) ───────────────────────────────── câmbio (direita)
       porta ↑ (abre para cima, lado esquerdo, y 200 → ~1100)
   ┌──────────────── PLATAFORMA MÓVEL (chassi 30×50, topo z = 380) ────────────────┐
   │  hastes traseiras (y = 60)        cardan (y = 700)            batentes (y = 1700)│
   └──────┬──────────────────────────────┬──────────────────────────────────────────┘
      manivelas + motores            torre do pivô
   ════════════════════ PLATAFORMA-BASE (piso) ═══════════════════════════════════════
```

| Dimensão | Valor |
|---|---|
| Base | 720 × 2050 mm (Y −200 → 1850) |
| Chassi móvel | 570 × 1780 mm |
| Anel | ±295 mm, z = 930 mm (ZP + 550) |
| Topo da cabeça | 1297 mm (F1) / 1825 mm (caminhão) |
| Massa da estrutura completa (sem piloto) | ≈ 179 kg (lista de materiais) |
| Massa móvel + piloto | ≈ 220 kg |

## 6. Plataforma-base (fixa)

- **Quadro:** longarinas 20×30 em X = ±330; 10 travessas; diagonal 10×20.
- **Torre do pivô:**
  - 4 pernas inclinadas 20×30;
  - chapa de topo 160×160×15;
  - **junta cardan** comprada (cruzeta com rolamentos de agulha) com centro em Y = 700, Z = 268. O eixo Y
    (rolagem) fica no garfo inferior e o eixo X (arfagem) no superior.
- **Atuadores** (um por lado):
  - motor NEMA 34 em malha fechada;
  - redutor planetário 20:1;
  - **braço de manivela em chapa de aço de 12 mm** (raio 100 mm). A R2 usava tubo 20×30, que veria
    ≈ 120 MPa cíclicos na força máxima;
  - terminais rotulares M16 e barra roscada M16 até a plataforma (Y = 60, X = ±310).
- **Batentes de borracha** dianteiros em Y = 1700: proteção mecânica além do limite de software.
- **Molas de centragem:** quatro molas de tração com olhais (ver seção 11).

## 7. Plataforma-móvel — estrutura

### 7.1 Chassi

- **Longarinas principais:** 30×50 em X = ±270, Y 0 → 1780.
- **Travessas:**
  - 30×50 em Y = 25;
  - **três travessas 30×50 no pivô** (Y = 650/700/750), sobre a **chapa do pivô 300×140×16** que recebe o
    garfo superior do cardan;
  - demais travessas em 20×30;
  - diagonais de piso 10×20.
- **Trilhos do banco e da pedaleira:** 30×50 deitados. A R2 tinha os trilhos do banco em 10×20, que ficavam
  sobrecarregados.
- **Treliça de soleira (lado esquerdo).** A abertura da porta interrompe a treliça do anel no lado esquerdo. Sem
  reforço, a força das hastes traseiras chegaria ao pivô só pela longarina em flexão: ≈ 620 MPa na força máxima.
  - Por isso há uma treliça rasa sob a longarina esquerda: banzo inferior 30×50, montantes e diagonais 20×30.
  - Nós em Y = 60/300/700/≈1100.
  - Altura de 60 → 130 → **205** → 60 mm: é máxima no pivô, onde o momento é máximo, e baixa nas pontas para
    não tocar na base em nenhuma pose.

### 7.2 Anel estrutural

- **Posição:** cota z = 930 mm (ZP + 550). Fica abaixo dos ombros no F1 e serve de base para os reforços do
  banco, do volante e do câmbio.
- **Laterais:** treliças com montantes e uma diagonal por vão. Estações à direita em Y = 25/300/700/1050/
  1450/1765; à esquerda em Y = 25/200, porta, ≈1100/1450/1765.
- **Quadro dianteiro:** travessa no anel e diagonal.
- **Quadro traseiro rebaixado:** travessa a z = 760 mm, com diagonal. Ela passa por baixo do apoio de cabeça do
  F1, que avança além da traseira do chassi. Uma travessa na altura do anel colidiria com o apoio — o primeiro
  defeito encontrado e corrigido pela verificação automática.
- **Mastro do volante:** suas colunas deslizam ao longo do anel, entre duas chapas-guia (superior e inferior). O
  anel funciona como guia e travamento lateral do mastro.

### 7.3 Barra da porta

- **Construção:** tubo 20×30 de ≈ 870 mm entre a dobradiça (Y = 200, montante traseiro esquerdo) e o fecho
  (Y ≈ 1100).
- **Abertura:** **gira 85° para cima** em torno de um eixo transversal (X), em vez de abrir para fora como uma
  porta de automóvel. Assim não ocupa espaço lateral, e o piloto entra passando por cima da soleira e por baixo da
  barra levantada.
- **Fecho:** forquilha aberta (duas orelhas e apoio) mais pino de trava rápida (*ball lock*) Ø10. A barra desce
  para dentro da forquilha e o pino atravessa as três peças.
- **Animação:** componente dinâmico com atributo próprio **`aberta`**. Clicar na barra com a ferramenta Interagir
  roda `ANIMATE("aberta",0,1)`.
- **Resistência:** atende uma pessoa apoiada no meio do vão com 600 N (93 MPa, ver `CALCULOS.md`). Não é
  degrau.

### 7.4 Câmbio

O câmbio fica fixo no anel direito, sobre uma chapa 190×140×6 com mão-francesa, em Y ≈ 700. Está na posição de
alcance do caminhão. No F1 usam-se as borboletas do G920, e o câmbio fica fora do caminho dos braços (verificado).

## 8. Conversão F1 ↔ caminhão em um movimento por módulo

Cada módulo (banco, pedaleira, volante) tem **um único grau de liberdade**: ao deslizar o carro, todas as outras
peças do módulo assumem a pose correta sozinhas, por hastes ou came. Cada módulo trava por **pinos de trava rápida**
nos trilhos. **Nenhum** ajuste angular é feito à mão.

Esta é a **opção A**. Ela fica pronta para a **opção B** (um atuador linear ou manivela única movendo os três
carros), porque a relação entre os cursos é fixa: banco +58,5 mm, pedaleira −366 mm, mastro +158 mm.

### 8.1 Banco

| Elemento | Função |
|---|---|
| Carro do banco | Desliza nos trilhos 30×50 (curso +58,5 mm) |
| Paralelogramo | 4 bielas de 360 mm; sobem de 2,1° (F1) a 66,0° (caminhão); o **subchassi** contra-gira e fica sempre a 9° |
| Tirantes de arrasto (par, 77 mm, X = ±200) | Ligam o chassi à biela traseira: o curso do carro passa a comandar o ângulo das bielas (1 GDL) |
| Haste do assento (291 mm) | Gira o assento de 25° (F1, paralelo à coxa) para 9° |
| **Par de hastes do encosto** (275 mm, X = ±100) | Gira o encosto lombar de 35° para 75°. Em par para dividir o momento do encosto e apoiar o carro junto às suas longarinas |
| **Tirante paralelo do apoio superior** (par, 10×20) | Fecha um paralelogramo com o eixo do encosto: o apoio de ombros/cabeça fica **sempre a 75° da horizontal**, sem trava nem ajuste |
| Calços de borracha F1 (4) | No F1 as bielas ficam quase deitadas e não devem carregar peso: o subchassi apoia em calços sobre as longarinas principais |
| Quadros de aço | Assento, encosto e apoio têm quadro 20×30 sob o estofado, onde ficam eixos e alavancas |

### 8.2 Pedaleira

- **Carro** nos trilhos 30×50, com curso de −366 mm.
- **Base basculante** gira −35° (sola de ~70° para ~35°) em torno do eixo no carro (Y ≈ 1292, Z = ZP + 130).
- **Haste de comando** de 207 mm: liga uma alavanca no tubo de torção da base a um poste triangulado no chassi.
- A R2 previa um rasgo em came no centro, que colidia com o próprio carro. Foi substituído, e a haste também
  dispensa corte a laser.
- A base foi estreitada para ±200 mm, entre os trilhos, e o G920 sobressai só por cima deles.

### 8.3 Volante

- **Mastro:** duas colunas 30×50 deslizam no chassi (curso +158 mm) e são guiadas pelo anel.
- **Braço:** gira −35° (aro de 80° para 45°) em torno do tubo de torção, a ZP + 700.
- **Acoplamento:** um pino seguidor em cada lado corre em **chapa com rasgo curvo** fixa no anel. O rasgo tem
  exatamente a forma da trajetória do pino, então o giro acompanha o deslize. Aqui o came cabe sem interferência.

### 8.4 Síntese e verificação dos mecanismos

Os pontos fixos das hastes saem de uma **busca com simulação da cinemática real**. Não basta a geometria das duas
posições extremas: o mecanismo é simulado ao longo de todo o curso. O critério é o **maior braço efetivo** (menor
força na haste) com **ângulo de transmissão ≥ 25°** em todo o curso.

- Resultado: menor transmissão de **28°**.
- Erro de posição nas poses finais: **0,0 mm**.
- Tabela completa em `CALCULOS.md` §7.

## 9. Posturas e balanceamento

| Parâmetro | F1 | Caminhão |
|---|---|---|
| Encosto (da horizontal) | 35° | 75° |
| Assento | 25° (paralelo à coxa) | 9° |
| Apoio superior | 75° | 75° |
| Coxa / joelho / tornozelo | +25° / 145° / 100° | +5° / 120° / 90° |
| Plano do aro | 80° | 45° |
| Recuo em relação à v2 (balanceamento) | 52 mm | 198 mm |

O gerador recua cada conjunto (banco + pedaleira + volante) ao longo de Y, sem mudar a ergonomia, até o CG
longitudinal cair sobre o pivô. O erro final é < 1 mm nos dois modos.

- Recuar o caminhão **não** afeta o F1: cada modo tem seu próprio recuo.
- A postura do caminhão é a mesma da v2, exceto pela translação.

## 10. Componentes dinâmicos e animação

| Atributo | Onde | Efeito |
|---|---|---|
| `modo` (0 = F1, 1 = Caminhão) | `Plataforma_Movel` | Clique (Interagir) → `ANIMATE("modo",0,1)`; propagado a todos os filhos por `Parent!modo` |
| `sentido` (Padrão/Invertido) | `Plataforma_Movel` | Inverte o sinal das rotações, se o SketchUp girar ao contrário |
| `aberta` (Fechada/Aberta) | `Porta_Barra` | Clique → `ANIMATE("aberta",0,1)` |

- **Leis de movimento:** cada componente que desliza ou gira usa um polinômio cúbico em `modo`, ajustado à
  cinemática real do mecanismo. Por isso as hastes não "esticam" na animação: o desvio é ≤ 2,3 mm.
- **Manequins:** o do F1 aparece só em `modo = 0` e o do caminhão só em `modo = 1`.
- **Teste:** as fórmulas seguem o mesmo esquema da R2, que foi testada no SketchUp desktop. A R3 acrescenta
  termos polinomiais e o atributo `aberta`, que **ainda não foram testados no desktop** (ponto 13.6).

## 11. Movimento, motores e molas (resumo de `CALCULOS.md`)

| Item | Valor |
|---|---|
| Massa móvel + piloto | 219,6 kg; CG 454 mm (F1) / 647 mm (caminhão) acima do pivô |
| Limites de software | arfagem ±7,8°; rolagem F1 **±16,2°** (pura); rolagem caminhão ±6° |
| Motor / redução | NEMA 34 malha fechada 12 N·m + planetário **20:1**; torque limitado a 6,9 N·m no driver |
| Desempenho estimado | F1 rolagem ±16° até ≈ 1,07 Hz; ±8° até ≈ 1,57 Hz (≈ 79 °/s) |
| Força de projeto na haste | 2,5 kN (limitada no driver) |
| Molas de centragem | par de arfagem: 4,8 N/mm; par de rolagem: 19,2 N/mm; rigidez 1,1 × m·g·h do caminhão |

**Freio econômico:**

- **Molas de tração:**
  - deixam a plataforma estável com os motores desligados;
  - reduzem o torque estático no F1;
  - só puxam para baixo, então o cardan nunca é tracionado.
- **Frenagem dinâmica dos drivers**, sem custo.
- **Botão de emergência**, que corta o *enable*.
- **Batentes de borracha.**
- **Freio eletromagnético:** fica opcional, conforme o resultado do protótipo.

## 12. Verificações realizadas

`ferramentas/verificar.py`:

| Verificação | Resultado |
|---|---|
| Interferência entre peças de componentes diferentes, em 5 pontos da transição | 0 |
| Barra da porta aberta e a meio curso, nos dois modos | 0 |
| Manequim × estrutura, nos dois modos | 0 (mãos no aro desconsideradas) |
| Plataforma × base em 340 poses alcançáveis | único contato: batentes dianteiros a ≈ −8,4° de arfagem (previsto); menor folga restante ≈ 14 mm |

Defeitos da proposta inicial da v3 encontrados e corrigidos por essas verificações e pelos cálculos:

- apoio de cabeça do F1 × travessa traseira do anel;
- cotovelos × diagonal do anel;
- mastro × diagonais;
- came central × carro da pedaleira;
- base da pedaleira × trilhos;
- longarina × placas dos motores;
- haste do encosto × almofada;
- longarina esquerda sem treliça (620 MPa);
- chapa do pivô (151 MPa);
- poste da haste da pedaleira em balanço (587 MPa);
- alavanca do encosto (440 MPa);
- haste do encosto com braço curto (12,8 kN);
- assento/encosto sem quadro de aço.

## 13. Pontos em aberto (verificar antes de fabricar)

1. **Curvas dos motores e rendimento do redutor** são típicas de catálogo. Refazer a tabela da seção 5 de
   `CALCULOS.md` com o datasheet do motor comprado.
2. **Junta cardan:** especificar a cruzeta pela carga de projeto (≈ 11 kN no pico, ver `CALCULOS.md`), com
   capacidade estática ≥ 20 kN. Uma cruzeta automotiva de cardan leve atende, mas os garfos são fabricados.
3. **Molas de tração** de 19,2 N/mm com ≈ 120 mm de alongamento máximo: confirmar com o fabricante. São molas sob
   encomenda, de custo baixo.
4. **Soldas:** a fadiga foi avaliada com Δσ ≤ 50 MPa. Pede solda contínua, sem mordedura, nos nós da treliça de
   soleira e das travessas do pivô.
5. **Buchas** em todos os furos de articulação: os tubos de 1,2 mm não aguentam pino direto por muito tempo.
6. **Animação no SketchUp desktop:** testar `modo` e `aberta` no arquivo R3. Se alguma peça girar ao contrário,
   usar `sentido`.
7. **Comprimentos de corte** são os do eixo, de face a face. Cortar com +2 mm e ajustar no gabarito; nas peças
   com corte em ângulo, conferir o ângulo na montagem.
8. **Estofados:** espessuras e abas são propostas. O assento e o apoio superior são o que mais precisa de prova
   com o usuário real.
9. **Opção B** (atuador único): exige projetar as ligações entre os três carros. Os cursos já estão fixados.

## 14. Organização do modelo 3D

```
Plataforma_Base (grupo)            quadro, torre, cardan, motores, manivelas, molas, batentes
Plataforma_Movel (DC: modo, sentido)
├─ geometria do chassi, anel, soleira, câmbio, chapas de came, calços F1
├─ Carro_Banco → Biela_Banco_* → Banco → Assento / Encosto → Encosto_Superior
│                                      → Tirante_Apoio_E/D
│  Haste_Encosto_E/D, Haste_Assento, Tirante_Carro_E/D
├─ Carro_Pedaleira → Pedaleira_Basculante ; Haste_Pedaleira
├─ Mastro_Volante → Braco_Volante
├─ Porta_Barra (DC: aberta)
└─ Manequim_F1 / Manequim_Caminhao
```

| Cena | Mostra |
|---|---|
| `Vista 3/4` / `Vista Lateral` | Conjunto completo |
| `Mecanismos (sem manequim)` | Mecanismos sem os manequins |
| `Base e Atuadores` | Somente a base, motores, manivelas e molas |

## 15. Regeneração

1. **Localmente**, rode `python3 ferramentas/verificar.py` e `python3 ferramentas/analisar.py`. Eles:
   - rodam as buscas completas dos mecanismos;
   - conferem interferências;
   - regeneram os documentos;
   - gravam o bloco `PRESET` no gerador.
2. **No SketchUp**, envie `modelo/gerar_modelo.py` como `code` do conector Trimble SketchUp MCP (`build_model`,
   `clean: true`) e salve com `save_model`.
   - O ambiente do conector encerra scripts em 30 s. Com `PRESET` preenchido, o gerador só avalia a solução já
     encontrada, sem repetir as buscas.
   - O ambiente também não tem `any()`/`all()` e não guarda variáveis entre chamadas. Se precisar dividir o
     envio, passe os dados por `session_state`.
