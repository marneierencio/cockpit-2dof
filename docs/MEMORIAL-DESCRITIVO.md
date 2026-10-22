# Memorial Descritivo — Cockpit de Simulação com Plataforma Móvel de 2 Graus de Liberdade

| Item | Valor |
|---|---|
| Projeto | Cockpit de simulador para jogos de computador, com duas posições de pilotagem (Fórmula 1 e caminhão) na **mesma estrutura ajustável** |
| Usuário-alvo | Adulto entre 170 e 175 cm de altura |
| Periféricos | Logitech **G920** (volante e pedais) + **Driving Force Shifter** (câmbio) |
| Movimento | 2 graus de liberdade (arfagem e rolagem), por dois motores de passo |
| Modelo 3D | `modelo/cockpit_2dof_ajustavel_G920.skp`, gerado por `modelo/gerar_modelo.py` |
| Unidade | Milímetros (mm), salvo indicação em contrário |
| Revisão | **R1**: posições F1/caminhão alcançadas por mecanismos (articulações, trilhos e travas), periféricos G920 e componentes dinâmicos com ANIMATE |

> **Natureza deste documento.** Este é um memorial de **anteprojeto**. As
> dimensões e a disposição das peças foram definidas por ergonomia e
> geometria, e verificadas por interferência no modelo 3D. **Não houve
> cálculo estrutural nem dimensionamento dos motores.** A seção 12 lista o
> que ainda precisa ser verificado antes de fabricar.

### Histórico de revisões

| Rev. | Mudança |
|---|---|
| R0 | Modelo conceitual com as duas configurações sobrepostas (peças diferentes para F1 e caminhão) |
| R1 | Peças únicas: banco em quadrilátero articulado, pedaleira basculante e volante em braço pivotado, todos sobre trilhos. Periféricos substituídos pelo Logitech G920 + câmbio. Transição animada via componente dinâmico. Postura caminhão ajustada para eliminar colisões |

---

## 1. Objetivo

Definir a estrutura de um cockpit de simulação com plataforma móvel. Ele
deve reproduzir duas posturas de pilotagem bem diferentes, **trocando de
uma para outra só por articulações, trilhos e encaixes**, sem trocar peças:

1. **Fórmula 1:** piloto reclinado, pés acima do quadril e volante quase
   vertical próximo ao peito.
2. **Caminhão:** piloto sentado ereto, quadril alto, pedais tipo "organ"
   e volante inclinado.

A plataforma onde o piloto senta é móvel e reproduz as sensações de
aceleração, frenagem e força G lateral.

## 2. Referências

- Vídeos indicados pelo solicitante:
  - <https://www.youtube.com/watch?v=L0wX_x7r-9k>
  - <https://www.youtube.com/watch?v=YZfblU3dd6k>
  - <https://www.youtube.com/shorts/t1sZWgEWC_w>
- Dimensões externas oficiais do Logitech G920 (página do fabricante):
  - **volante:** 270 (A) × 260 (L) × 278 (P) mm, 2,25 kg;
  - **pedais:** 167 (A) × 428,5 (L) × 311 (P) mm, 3,1 kg.
- Antropometria aproximada de um adulto de 170–175 cm (seção 4).

## 3. Sistema de coordenadas e convenções

| Eixo | Sentido |
|---|---|
| **X** | Lateral. Positivo para a **direita** do piloto |
| **Y** | Longitudinal. Positivo para a **frente** (o piloto olha para +Y). Y = 0 é a traseira da plataforma móvel |
| **Z** | Vertical. Z = 0 é o piso. **Zp = 380 mm** é o topo do chassi móvel |

- Sufixos `_E` / `_D` nos nomes das peças: lado esquerdo / direito do piloto.
- Ângulos de postura são medidos no plano sagital (Y-Z), a partir de +Y
  em direção a +Z (regra da mão direita em torno de +X).
- Cores no modelo:
  - **grafite:** tubo 20×30
  - **laranja:** tubo 10×20
  - **azul:** bielas do banco
  - **cinza-claro:** chapas, mancais, setores de trava e pinos
  - **amarelo:** junta cardan
  - **preto/cinza-escuro:** periféricos G920, motores e estofado

## 4. Premissas antropométricas (manequim)

O modelo inclui um manequim articulado por posição, no estilo *rigged
model*: um segmento para cada osso e uma esfera em cada articulação. As
pernas e o tronco são posicionados por cinemática direta; os braços, por
cinemática inversa de dois segmentos até as mãos no aro do volante.

| Segmento | Comprimento (mm) |
|---|---|
| Coxa / perna | 420 / 420 |
| Tronco (centro do quadril → linha dos ombros) | 520 |
| Braço / antebraço + mão | 300 / 310 |
| Meia-largura do quadril / dos ombros | 90 / 195 |

Estatura em pé resultante: **≈ 1730 mm**.

## 5. Descrição geral

O modelo tem dois conjuntos na raiz:

1. **Plataforma-base** (fixa no piso): quadro, torre do pivô, junta
   cardan, atuadores e batentes. Inalterada desde a R0.
2. **Plataforma-móvel**: chassi e **três mecanismos de ajuste** (banco,
   pedaleira e volante), mais o câmbio, que vai preso ao banco.

As duas plataformas se unem por um **pivô central** (junta cardan, perto do
centro de gravidade) e por **duas hastes traseiras** ligadas a braços de
manivela acionados por motores de passo. Os motores no mesmo sentido
produzem **arfagem**; em sentidos opostos, **rolagem**.

### 5.1 Dimensões gerais

| Item | Valor |
|---|---|
| Base: comprimento × largura (envelope externo) | 2050 × 680 mm (Y −200 a +1850; X ±340) |
| Plataforma móvel: comprimento × largura | 1780 × 560 mm |
| Largura máxima com o câmbio | ~860 mm (o câmbio fica até X = +425) |
| Topo da cabeça (do piso): F1 / caminhão | ~1305 / ~1825 mm |

## 6. Plataforma-base (fixa) — sem alteração na R1

| Peça | Perfil | Qtde | Comprimento unit. |
|---|---|---|---|
| Longarinas (X = ±330) | 20×30 | 2 | 2050 |
| Travessas (Y = −190, −120, 40, 560, 840, 1700, 1840) | 20×30 | 7 | 640 |
| Apoios dos motores (X = ±115) | 20×30 | 2 | 140 |
| Diagonal | 10×20 | 1 | ~1056 |
| Torre do pivô (4 pernas convergindo em chapa a Z ≈ 215–228) | 20×30 | 4 | ~265 |

- **Junta cardan:** em X = 0, Y = 700, Z ≈ 285. Permite **rolagem** (eixo Y)
  e **arfagem** (eixo X). Na fabricação, usar uma cruzeta comercial com
  rolamentos de agulha.
- **Atuadores (×2):** NEMA 34 com redutor planetário, manivela de 100 mm
  (eixo em Y = −40, Z = 160), haste vertical Ø16 com rótulas nas duas
  pontas, em X = ±310, Y = 60.
- **Batentes dianteiros:** colunas em X = ±270 sobre a travessa Y = 1700,
  com borracha no topo em Z = 175. Folga de 175 mm até o chassi móvel.

## 7. Plataforma-móvel

### 7.1 Chassi (fixo em relação à plataforma)

| Peça | Perfil | Qtde | Posição | Comprimento |
|---|---|---|---|---|
| Longarinas principais | 20×30 | 2 | X = ±270, Z 350–380 | 1780 |
| Travessas principais | 20×30 | 3 | Y = 15, 700 (pivô), 1765 | 520 |
| Travessas secundárias | 10×20 | 3 | Y = 350, 1100, 1450 | 520 |
| Trilhos do banco | 10×20 | 2 | X = ±150, Y 5 → 1110 | 1105 |
| Trilhos da pedaleira (**estendidos na R1**) | 10×20 | 2 | X = ±230, Y 880 → 1775 | 895 |

Os trilhos têm furação para os **pinos de trava** dos carros. As
longarinas principais servem de trilho para as abraçadeiras do mastro do
volante.

### 7.2 Princípio dos mecanismos de ajuste

Cada conjunto ajustável vai de F1 para caminhão com **um único giro em
torno de uma articulação** mais **um único deslocamento em trilho**. A
posição de cada articulação foi **calculada** (síntese de mecanismo no
script) para que esse par giro + deslocamento leve a peça **exatamente** da
pose F1 à pose caminhão. O erro verificado é de 0 mm no calcanhar e no
centro do volante. Nas duas pontas, a posição é travada por pino em
**setores furados** (articulações) e em **furos dos trilhos** (carros).

### 7.3 Banco — quadrilátero articulado (paralelogramo) sobre carro

| Elemento | Especificação |
|---|---|
| Carro do banco | 2 tubos 20×30 (X = ±150) + 2 travessas, deslizando nos trilhos do banco |
| Bielas | 4 bielas 20×30 de **360 mm** entre pinos (azuis), em X = ±180, formando um paralelogramo |
| Ângulo das bielas | **7,6° (F1) → 75,6° (caminhão)**, giro de 68° |
| Curso do carro | **+279 mm** para a frente na posição caminhão |
| Assento | 480 × 440 × 60 mm, inclinação **fixa de 9°** (o paralelogramo mantém o ângulo) |
| Encosto | 480 × 590 × 60 mm, articulado no assento (eixo em C), com setor de trava |
| Reclinação | encosto de **35° (F1) → 75° (caminhão)** em relação à horizontal (giro de 40°) |
| Elevação resultante | o quadril sobe **307 mm** (Zp + 290 → Zp + 597) |

O paralelogramo eleva e recua o assento sem incliná-lo; o carro compensa
o recuo. Na R0, a altura do banco dependia de colunas telescópicas que
precisariam ir de ~25 mm a ~330 mm, o que não é fabricável. O quadrilátero
resolve isso.

### 7.4 Câmbio (Driving Force Shifter)

- Fica sobre uma chapa de 140 × 190 mm, num suporte 20×30 parafusado na
  lateral direita do assento: centro em X = +360, 150 mm à frente do canto
  assento/encosto.
- Como o assento não inclina, o câmbio **acompanha o banco e fica sempre
  nivelado**, nas duas posições.
- Dimensões usadas: 130 × 165 × 167 mm (estimativa; o fabricante não
  publica a medida, conferir na peça).

### 7.5 Pedaleira — base basculante sobre carro (pedais G920)

| Elemento | Especificação |
|---|---|
| Carro da pedaleira | 2 tubos 20×30 (X = ±230) + travessa, com duas torres de pivô |
| Pivô da base | Y = 1344 (na posição F1), Zp + 130 |
| Giro da base | **−35°**: a sola passa de ~70° (F1) para ~35° (caminhão) em relação à horizontal |
| Curso do carro | **−220 mm** (recua na posição caminhão) |
| Base | chapa de fixação 436 × 331 mm para a **unidade de pedais G920** (428,5 × 311 × 167 mm), presa por dois braços 20×30 de cada lado (X = ±250) e duas travessas |
| Pedais | embreagem / freio / acelerador com passo de 125 mm (estimativa), centrados entre os pés |

### 7.6 Volante — braço pivotado sobre mastro deslizante (volante G920)

| Elemento | Especificação |
|---|---|
| Mastro | 2 colunas 20×30 em abraçadeiras que correm sobre as longarinas (X = ±270) + travessa 20×30 |
| Pivô do braço | Y = 1240 (na posição F1), **Zp + 700** |
| Giro do braço | **−35°**: o plano do aro passa de **80° (F1) para 45° (caminhão)** |
| Curso do mastro | **+303 mm** (avança na posição caminhão) |
| Braço | 4 tubos 20×30 entre o pivô e a **mesa do volante**; balanço de ~600 mm até o centro do aro |
| Mesa | chapa de 300 × 240 mm onde a base do G920 é fixada (grampos/parafusos) |
| Volante | base G920 260 × 232 × 190 mm (proporção estimada sobre as medidas oficiais), aro Ø270 |

O aro do G920 é fixo em relação à base, a ~80° da face de apoio
(estimativa). Por isso o ajuste de ângulo é feito **inclinando a mesa**.

### 7.7 Verificações de interferência (modelo R1)

| Verificação | Resultado |
|---|---|
| Base e aro do G920 × coxas/joelhos (caminhão) | 45 mm ✔ |
| Travessa do mastro × pernas (F1 / caminhão) | 200 / 323 mm ✔ |
| Travessa do mastro abaixo da linha de visão olho → topo do aro | 34 mm de margem ✔ |
| Base dos pedais acima do topo do chassi (caminhão) | 26 mm ✔ |
| Carro do banco dentro do trilho (Y 5–1110) | ocupa Y 264–969 ✔ |
| Carro da pedaleira dentro do trilho (Y 880–1775) | ocupa Y 1034–1484 ✔ |
| Mastro sobre as longarinas | Y 1240 → 1543 ✔ |

## 8. Posturas de pilotagem

| Parâmetro | Fórmula 1 | Caminhão |
|---|---|---|
| Quadril: Y; altura acima de Zp | 650; 290 | 700; **597** |
| Encosto (a partir da horizontal) | 35° | 75° |
| Inclinação do assento | 9° | 9° |
| Coxa / joelho / tornozelo | +25° / 145° / 100° | +5° / 120° / 90° |
| Sola (base dos pedais) | ~70° | ~35° |
| Plano do aro do volante | 80° | **45°** |
| Pegada das mãos | 9 h e 3 h | 10 h e 2 h |
| Distância ombro → mão | ~428 mm | ~516 mm |

**Ajustes feitos na postura caminhão para eliminar colisões com o G920:**
quadril +60 mm, aro a 45° em vez de 35°, volante 40 mm mais à frente e
132 mm mais alto em relação ao ombro.

## 9. Componentes dinâmicos e animação (ANIMATE)

Os mecanismos são **componentes dinâmicos** do SketchUp, controlados por
um atributo `modo` (0 = F1, 1 = caminhão) no componente `Plataforma_Movel`.

| Componente | Fórmula (resumo) |
|---|---|
| `Plataforma_Movel` | `modo` (lista F1/Caminhão), `sentido` (1 / −1); **onClick = `ANIMATE("modo",0,1)`** |
| `Carro_Banco`, `Carro_Pedaleira`, `Mastro_Volante` | `Y = Y0 + modo × curso` |
| `Biela_Banco_*` | `RotX = modo × sentido × 68,0°` |
| `Banco` | `RotX = −modo × sentido × 68,0°` (contra-rotação: o assento fica nivelado) |
| `Encosto` | `RotX = modo × sentido × (−40°)` |
| `Pedaleira_Basculante` / `Braco_Volante` | `RotX = modo × sentido × (−35°)` |
| `Manequim_F1` / `Manequim_Caminhao` | `Hidden` fora de `modo = 0` / `modo = 1` |

**Como usar (SketchUp desktop):**
1. Selecione a ferramenta **Interagir** (ícone de mão).
2. Clique na plataforma móvel: os carros deslizam, as bielas giram, o
   encosto reclina e a pedaleira e o volante basculam até a posição
   caminhão. Clique de novo para voltar ao F1.
3. Para escolher a posição sem animar, use **Janela → Opções do
   componente** (*Component Options*) e o campo "Posição de pilotagem".

Durante a transição os manequins ficam ocultos: eles representam o piloto
em cada posição final e não são animados.

## 10. Cinemática da plataforma móvel

| Grandeza | Valor | Observação |
|---|---|---|
| Raio da manivela | 100 mm | |
| Distância pivô → hastes (longitudinal / lateral) | 640 / 310 mm | |
| **Arfagem máxima** | **±8,9°** | |
| **Rolagem máxima (geométrica)** | **±17,9°** | Limitar a ±8–10° no software |
| Curso na frente da plataforma | ±169 mm | Folga até os batentes: 175 mm |

## 11. Lista de tubos (estimativa)

Comprimentos medidos no modelo 3D. Cada peça ajustável aparece **uma vez**,
porque agora é a mesma peça nas duas posições.

| Conjunto | Tubo 20×30 | Tubo 10×20 |
|---|---|---|
| Plataforma-base | 17 peças, 10,14 m | 1 peça, 1,06 m |
| Chassi móvel | 5 peças, 5,12 m | 7 peças, 5,56 m |
| Banco (carro + 4 bielas + suporte do câmbio) | 9 peças, 3,02 m | — |
| Pedaleira (carro, torres, braços, travessas) | 11 peças, 3,79 m | — |
| Volante (mastro, travessa, braços) | 7 peças, 3,82 m | — |
| **Subtotal** | **25,9 m** | **6,6 m** |
| **Com 10% de perdas** | **≈ 28,5 m** | **≈ 7,3 m** |

Não incluídos: chapas, mancais, setores de trava, pinos (Ø16–20),
cardan, rótulas, motores e redutores, banco, periféricos.

## 12. Pontos em aberto e recomendações (obrigatório verificar antes de fabricar)

1. **Centro de gravidade no modo caminhão** (*crítico, piorou na R1*). O
   quadril agora sobe 307 mm entre F1 e caminhão, e o CG fica bem acima da
   cruzeta. Recomendações:
   - torre do pivô com altura regulável, **ou**
   - motores e redutores dimensionados pelo modo caminhão.
2. **Componentes dinâmicos não foram testados neste ambiente.**
   - Os atributos foram gravados pela API do SketchUp, que não executa as
     fórmulas. Devem funcionar no SketchUp desktop (ferramenta Interagir).
   - Se o banco, a pedaleira ou o volante girarem **para o lado errado**,
     troque "Sentido" para "Invertido" nas Opções do componente.
   - Não está confirmado que o SketchUp para Web suporta Interagir.
3. **Medidas internas do G920.** Só as dimensões externas são oficiais.
   Antes de furar as chapas, conferir na peça:
   - o ângulo do aro em relação à base;
   - a furação de fixação do volante e dos pedais;
   - o passo dos pedais;
   - as medidas do câmbio.
4. **Braço do volante em balanço (~600 mm).** O G920 tem pouco torque
   (2,3 N·m), mas a vibração e a flexão do braço precisam de verificação.
   Se necessário, use tubos de parede maior ou um tirante diagonal.
5. **Espessura de parede dos tubos.** Mínimo de 2,0 mm no 20×30 das bielas,
   da torre, das longarinas e dos carros; 1,2–1,5 mm no 10×20.
6. **Travas.** Os pinos das bielas e dos setores carregam o peso do piloto:
   - pinos com Ø ≥ 16 mm em aço;
   - bielas com bucha;
   - trava redundante (pino + parafuso) no modo caminhão, em que as bielas
     ficam quase verticais.
7. **Dimensionamento dos motores** (torque estático e dinâmico, margem
   contra perda de passo) e **limites de software** (rolagem ±8–10°, fins
   de curso).
8. **Segurança:**
   - parada de emergência;
   - proteção das manivelas;
   - partida dos motores sempre na posição neutra;
   - nunca trocar de modo com a plataforma energizada.

## 13. Organização do modelo 3D

```
Plataforma_Base (grupo, tag Plataforma_Base)
Plataforma_Movel (componente dinâmico, tag Plataforma_Movel)
├── Chassi_Movel
├── Carro_Banco
│   ├── Biela_Banco_E_Tras
│   │   └── Banco ── Encosto, G920_Cambio
│   ├── Biela_Banco_D_Tras
│   ├── Biela_Banco_E_Diant
│   └── Biela_Banco_D_Diant
├── Carro_Pedaleira
│   └── Pedaleira_Basculante
│       └── G920_Pedais
├── Mastro_Volante
│   └── Braco_Volante
│       └── G920_Volante
├── Manequim_F1        (tag Manequim)
└── Manequim_Caminhao  (tag Manequim)
```

**Cenas salvas no arquivo:**

| Cena | Mostra |
|---|---|
| `Vista 3/4` | Vista dianteira-esquerda |
| `Vista Lateral` | Vista lateral esquerda |
| `Mecanismos (sem manequim)` | Vista lateral sem os manequins |
| `Base e Atuadores` | Somente a plataforma-base |

`G920_Volante`, `G920_Pedais` e `G920_Cambio` são componentes que podem
ser trocados por um modelo detalhado: clique direito → *Recarregar*.

## 14. Regeneração do modelo

`modelo/gerar_modelo.py` é **paramétrico**:

- as posturas estão em `POSE_F1` e `POSE_TRK`;
- as medidas dos periféricos estão nas constantes `G920_*`;
- os pivôs e cursos dos mecanismos são **recalculados** a cada execução.

O resultado da execução traz um relatório de verificação, com erro de
síntese, folgas e cursos. O script roda no ambiente Python do conector
**Trimble SketchUp MCP** (`build_model`), e não no console Ruby do SketchUp
desktop. Veja o `README.md`.
