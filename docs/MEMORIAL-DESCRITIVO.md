# Memorial Descritivo — Cockpit de Simulação com Plataforma Móvel de 2 Graus de Liberdade

| Item | Valor |
|---|---|
| Projeto | Cockpit de simulador para jogos de computador, com duas posições de pilotagem (Fórmula 1 e caminhão) |
| Usuário-alvo | Adulto entre 170 e 175 cm de altura |
| Movimento | 2 graus de liberdade (arfagem e rolagem), por dois motores de passo |
| Modelo 3D | `modelo/cockpit_simulador_F1_caminhao.skp`, gerado por `modelo/gerar_modelo.py` |
| Unidade | Milímetros (mm), salvo indicação em contrário |
| Revisão | R0: modelo conceitual (anteprojeto) |

> **Natureza deste documento.** Este é um memorial de **anteprojeto**. As
> dimensões e a disposição das peças foram definidas por ergonomia e
> geometria, e verificadas por interferência no modelo 3D. **Não houve
> cálculo estrutural nem dimensionamento dos motores.** As seções 10 e 11
> listam o que ainda precisa ser verificado antes de fabricar.

---

## 1. Objetivo

Definir a estrutura de um cockpit de simulação com plataforma móvel. Ele
deve reproduzir duas posturas de pilotagem bem diferentes usando a mesma
estrutura:

1. **Fórmula 1:** piloto quase deitado, quadril baixo, pés acima do
   quadril e volante pequeno quase vertical, próximo ao peito.
2. **Caminhão:** piloto sentado ereto, quadril alto, joelhos em ~120°,
   pedais tipo "organ" articulados no piso e volante grande, bem inclinado.

A plataforma onde o piloto senta é móvel e reproduz as sensações de
aceleração, frenagem e força G lateral.

## 2. Referências

- Vídeos indicados pelo solicitante:
  - <https://www.youtube.com/watch?v=L0wX_x7r-9k>
  - <https://www.youtube.com/watch?v=YZfblU3dd6k>
  - <https://www.youtube.com/shorts/t1sZWgEWC_w>
- Antropometria aproximada de um adulto de 170–175 cm (seção 4).
- Conceito de plataforma 2DOF com pivô central e dois atuadores
  traseiros de manivela, comum em simuladores caseiros.

## 3. Sistema de coordenadas e convenções

| Eixo | Sentido |
|---|---|
| **X** | Lateral. Positivo para a **direita** do piloto |
| **Y** | Longitudinal. Positivo para a **frente** (o piloto olha para +Y). Y = 0 é a traseira da plataforma móvel |
| **Z** | Vertical. Z = 0 é o piso |

- Sufixos `_E` / `_D` nos nomes das peças: lado esquerdo / direito do piloto.
- Ângulos de postura são medidos no plano sagital (Y-Z), a partir de +Y
  em direção a +Z. Exemplo: um encosto a 145° aponta para trás e para
  cima, 35° acima da horizontal.
- Cores no modelo:
  - **grafite:** tubo 20×30
  - **laranja:** tubo 10×20
  - **cinza-claro:** chapas e peças usinadas
  - **amarelo:** junta cardan
  - **preto:** motores, estofado e volante

## 4. Premissas antropométricas (manequim)

O modelo inclui um manequim articulado, no estilo *rigged model*: um
segmento sólido para cada osso e uma esfera em cada articulação. As
juntas são posicionadas por cinemática: direta para as pernas e o tronco,
inversa de dois segmentos para os braços, que partem dos ombros e chegam
às mãos no aro do volante.

| Segmento | Comprimento (mm) |
|---|---|
| Coxa (quadril → joelho) | 420 |
| Perna (joelho → tornozelo) | 420 |
| Tronco (centro do quadril → linha dos ombros) | 520 |
| Braço (ombro → cotovelo) | 300 |
| Antebraço + mão (cotovelo → centro da pegada) | 310 |
| Meia-largura do quadril / dos ombros | 90 / 195 |
| Pé (calcanhar → ponta) | ~265 |

Estatura em pé resultante: tornozelo (~80) + perna (420) + coxa (420) +
tronco (520) + ombro ao topo da cabeça (~290) ≈ **1730 mm**, dentro da
faixa de 170–175 cm.

## 5. Descrição geral

O conjunto (`Cockpit_Simulador`) tem duas partes principais:

1. **Plataforma-base** (fixa no piso): quadro em tubos, torre do pivô,
   junta universal, atuadores e batentes.
2. **Plataforma-móvel**: chassi onde ficam o banco, a pedaleira e o
   volante, com seus mecanismos de ajuste.

As duas partes se unem por **três pontos**:

- **um pivô central**, uma junta universal (cardan) colocada próxima ao
  centro de gravidade da plataforma-móvel com o piloto, que permite
  inclinar para os lados (rolagem) e para frente e para trás (arfagem);
- **duas hastes traseiras**, uma de cada lado, presas aos cantos
  traseiros da plataforma-móvel e ligadas a braços de manivela acionados
  por motores de passo fixos na plataforma-base.

Os dois motores girando no **mesmo sentido** produzem **arfagem**
(aceleração e frenagem). Girando em **sentidos opostos**, produzem
**rolagem** (força G lateral). A rotação em torno do eixo vertical é
impedida pelas próprias hastes.

### 5.1 Dimensões gerais

| Item | Valor |
|---|---|
| Base: comprimento × largura (envelope externo) | 2050 × 680 mm (Y −200 a +1850; X ±340) |
| Plataforma móvel: comprimento × largura | 1780 × 560 mm (Y 0 a 1780; X ±270) |
| Topo do chassi móvel (Zp) | 380 mm do piso |
| Altura máxima (topo da cabeça, modo caminhão) | ~1765 mm |
| Altura máxima (topo da cabeça, modo F1) | ~1250 mm |

## 6. Plataforma-base (fixa)

### 6.1 Quadro

| Peça | Perfil | Qtde | Posição | Comprimento unit. |
|---|---|---|---|---|
| Longarinas (`Base_Longarina_E/D`) | 20×30 | 2 | X = ±330, Y −200 → 1850 | 2050 |
| Travessas (`Base_Travessa_y…`) | 20×30 | 7 | Y = −190, −120, 40, 560, 840, 1700, 1840 | 640 |
| Apoios longitudinais dos motores | 20×30 | 2 | X = ±115, Y −110 → 30 | 140 |
| Diagonal de contraventamento | 10×20 | 1 | (−320; 850) → (320; 1690) | ~1056 |

Os tubos 20×30 do quadro ficam deitados com a face de 30 mm na vertical,
para resistir melhor à flexão sob o peso.

### 6.2 Torre do pivô

- Quatro pernas em 20×30 (`Torre_Perna`) partem das travessas
  Y = 560 e Y = 840 (X = ±200, Z = 30) e convergem para uma chapa de topo
  em Z ≈ 215–228 mm. Comprimento de cada perna: ~265 mm.
- `Torre_Chapa_Topo`: chapa 160 × 160 × 13 mm.
- A torre recebe **todo o peso** da plataforma-móvel e do piloto que não
  vai para as hastes. Por isso usa somente tubo 20×30.

### 6.3 Junta universal (cardan)

Posição: **X = 0, Y = 700, Z ≈ 285 mm** (centro da cruzeta).

| Peça | Descrição |
|---|---|
| `Garfo_Inferior_Base` + 2 orelhas | Preso à chapa de topo da torre; segura o eixo longitudinal da cruzeta |
| `Cruzeta_Eixo_Y` (Ø26 × 120) | Eixo da **rolagem** |
| `Cruzeta_Eixo_X` (Ø26 × 120) | Eixo da **arfagem** |
| `Garfo_Superior_Orelha` ×2 + `Garfo_Superior_Base` | Preso à `Chapa_Pivo` do chassi móvel |

Recomendação de fabricação: usar uma cruzeta comercial de cardan
automotivo ou agrícola (com rolamentos de agulha), ou uma rótula esférica
de grande porte com trava anti-giro. O desenho do modelo é apenas
representativo.

### 6.4 Atuadores traseiros

Um conjunto por lado (`Atuador_E`, `Atuador_D`):

| Componente | Especificação no modelo |
|---|---|
| Motor | Motor de passo **NEMA 34** (corpo 86 × 86 × 150 mm), instalado por dentro, com eixo paralelo a X |
| Redutor | Planetário Ø84 × 95 mm |
| Eixo de saída | Ø20, em X = ±295 a ±318, Y = −40, Z = 160 |
| Braço de manivela | 14 × 34 mm, **raio de 100 mm** (eixo em Y = −40 → rótula em Y = +60) |
| Chapa de suporte do motor | Chapa vertical de 10 mm, 160 × 200 mm, sobre as travessas Y = −120 e Y = 40 |
| Calço do motor | Apoio do corpo do motor sobre o tubo longitudinal X = ±115 |
| Haste de ligação | Ø16, vertical, ~144 mm entre centros de rótula, em X = ±310, Y = 60 |
| Terminais | Rótulas esféricas (*rod ends*) nas duas pontas |

Na posição neutra o braço fica **horizontal** e a haste **vertical**.
Nessa posição o braço tem o maior braço de alavanca e a resposta é a mais
linear em torno do neutro.

### 6.5 Batentes dianteiros

- Duas colunas 20×30 (Z 30 → 140) sobre a travessa Y = 1700, em X = ±270,
  com batentes de borracha Ø48 × 35 mm (topo em Z = 175).
- Folga até a face inferior das longarinas móveis (Z = 350): **175 mm**.
  Isso cobre o curso dianteiro máximo calculado (~169 mm, seção 9) e
  serve de **parada mecânica de segurança** em caso de falha de controle.

## 7. Plataforma-móvel

### 7.1 Chassi (`Chassi_Movel`)

| Peça | Perfil | Qtde | Posição | Comprimento unit. |
|---|---|---|---|---|
| Longarinas principais | 20×30 | 2 | X = ±270, Z 350–380 | 1780 |
| Travessas principais (traseira, pivô, dianteira) | 20×30 | 3 | Y = 15, 700, 1765 | 520 |
| Travessas secundárias | 10×20 | 3 | Y = 350, 1100, 1450 | 520 |
| Trilhos de deslize do banco | 10×20 | 2 | X = ±150, Y 5 → 1110, sobre as travessas | 1105 |
| Trilhos de deslize da pedaleira | 10×20 | 2 | X = ±230, Y 1095 → 1775 | 680 |
| `Chapa_Pivo` | chapa 10 mm | 1 | Sob a travessa Y = 700 | 160 × 160 |
| `Suporte_Haste_E/D` | chapa 20 mm | 2 | Na face externa das longarinas, Y 35–85 | 50 × 50 |

As travessas principais recebem cargas concentradas (pivô e hastes). Por
isso são 20×30. As secundárias só travam o quadro e apoiam os trilhos,
por isso são 10×20.

### 7.2 Banco (ajustável)

- **Composição:** assento de 480 × 440 × 60 mm e encosto de
  480 × 590 × 60 mm, com abas laterais (apoio de quadril e tronco) em
  vermelho.
- **Ajustes:**
  1. **Longitudinal:** carro de banco (`Carro_Banco_E/D`, 20×30) que
     desliza sobre os trilhos 10×20 e trava por pino nos furos dos
     trilhos.
  2. **Altura:** colunas 20×30 (`Coluna_Frontal`, `Coluna_Traseira`,
     `Coluna_Encosto`) entre o carro e o banco. Na prática devem ser
     **telescópicas**, com um tubo interno de 16×26 deslizando dentro do
     20×30 e furação a cada 25 mm.
  3. **Inclinação do encosto:** articulação na aresta assento/encosto,
     com trava por setor furado.
  4. **Inclinação do assento:** furação dupla na coluna dianteira.
- **Travamento diagonal** (`Travamento_Diagonal_E/D`, 10×20) contra o
  balanço longitudinal.
- O quadril do piloto fica **95 mm acima** da superfície do assento e
  **115 mm à frente** do encosto. O canto assento/encosto é calculado a
  partir dessas duas distâncias.

### 7.3 Pedaleira (ajustável)

- **Base inclinável** (`Base_Pedais_Inclinavel`): chapa de 470 × 330 ×
  10 mm, paralela à sola do pé.
- **Pedais:** embreagem (sob o pé esquerdo), freio (115 mm à esquerda do
  acelerador) e acelerador (sob o pé direito). Placas de 76 × 165 mm
  com braço de pedal em 10×20 e **célula de carga** no freio.
- **Ajustes:**
  1. **Longitudinal:** carro da pedaleira (`Carro_Pedaleira_E/D`, 20×30)
     sobre os trilhos 10×20.
  2. **Altura e ângulo:** coluna inferior (articulação da base) e coluna
     superior regulável. Variar a altura relativa das duas muda o ângulo
     da base.

### 7.4 Volante (ajustável)

- **Volante** com aro toroidal, cubo e raios. A versão F1 tem um painel
  central retangular (tipo "volante de F1").
- **Motor direct-drive** Ø110 × 200 mm, coaxial ao volante, sobre uma
  **chapa de fixação inclinável**.
- **Mastro:** duas colunas 20×30 (`Coluna_Volante_E/D`) presas às
  longarinas por **abraçadeiras deslizantes**, uma **travessa** 20×30 e
  dois **suportes** 20×30 entre a chapa do motor e a travessa.
- **Ajustes:**
  1. **Longitudinal:** abraçadeiras correm ao longo das longarinas.
  2. **Altura:** colunas telescópicas.
  3. **Ângulo:** chapa do motor articulada na travessa.
- No modo caminhão, o motor passa **entre os joelhos** (afastados
  ±130 mm) e a travessa fica à frente deles, a 190 mm do joelho.

## 8. Configurações de pilotagem

As duas configurações usam o **mesmo** chassi, trilhos e mecanismos. O
modelo mostra cada uma em sua própria tag (`Config_F1` e
`Config_Caminhao`). Na estrutura real, as peças de cada tag são **as
mesmas peças em posições diferentes**.

### 8.1 Parâmetros de postura

| Parâmetro | Fórmula 1 | Caminhão |
|---|---|---|
| Quadril: posição Y | 650 | 700 |
| Quadril: altura acima do chassi (Zp) | 235 | 537 |
| Tronco/encosto (a partir da horizontal) | 35° (reclinado) | 75° (quase ereto) |
| Inclinação do assento | 12° | 6° |
| Coxa (a partir da horizontal) | +25° | +5° |
| Ângulo interno do joelho | 145° | 120° |
| Ângulo do tornozelo | 100° | 90° |
| Afastamento dos joelhos (±X) | 100 | 130 |
| Pé → ângulo da base dos pedais | ~70° (base quase vertical, pedais "suspensos") | ~35° (pedais tipo organ) |
| Calcanhar acima do chassi | ~269 (pés acima do quadril) | ~143 |
| Volante: diâmetro | 280 | 400 |
| Volante: plano (a partir da horizontal) | 80° | 35° |
| Volante: centro (Y; altura acima de Zp) | 644; 593 | 980; 807 |
| Pegada das mãos | 9 h e 3 h | 10 h e 2 h |
| Distância ombro → mão | ~428 mm (cotovelos dobrados) | ~527 mm |
| Topo da cabeça (do piso) | ~1250 | ~1765 |

### 8.2 Verificações de interferência feitas no modelo

| Verificação | F1 | Caminhão |
|---|---|---|
| Fundo do assento acima do carro do banco | 27 mm ✔ | 326 mm (colunas altas) ✔ |
| Travessa do volante × coxas | Fundo da travessa ~34 mm acima da coxa ✔ (justo) | Travessa à frente dos joelhos ✔ |
| Motor DD × joelhos | Motor acima das coxas ✔ | Motor (Ø110) passa entre joelhos com vão de 164 mm ✔ |
| Pedaleira dentro do chassi | Topo da base em Y ≈ 1701 < 1780 ✔ | ✔ |
| Traseira descendo 100 mm × motores | Travessa traseira em Z = 250 > topo do motor Z = 203 ✔ | idem ✔ |

## 9. Cinemática e cursos de movimento

| Grandeza | Valor | Observação |
|---|---|---|
| Raio da manivela | 100 mm | |
| Distância pivô → hastes (longitudinal) | 640 mm | Pivô em Y = 700, hastes em Y = 60 |
| Distância pivô → hastes (lateral) | 310 mm | |
| Curso vertical em cada haste | ±100 mm | Manivela de −90° a +90° a partir da horizontal |
| **Arfagem máxima** | **±8,9°** | atan(100/640) |
| **Rolagem máxima (geométrica)** | **±17,9°** | atan(100/310). **Limitar a ±8–10° no software** (seção 11) |
| Curso na frente da plataforma (arfagem máx.) | ±169 mm | (1780 − 700) × tan 8,9° |
| Folga até os batentes dianteiros | 175 mm | |

## 10. Lista de tubos (estimativa)

Comprimentos medidos no modelo 3D. As peças ajustáveis foram contadas
**uma vez**, com o comprimento da configuração caminhão, que é a mais
longa.

| Grupo | Tubo 20×30 | Tubo 10×20 |
|---|---|---|
| Plataforma-base | 17 peças, 10,14 m | 1 peça, 1,06 m |
| Chassi móvel | 5 peças, 5,12 m | 7 peças, 5,13 m |
| Banco + pedaleira + mastro do volante (modo caminhão) | 19 peças, 7,40 m | 2 peças, 1,62 m |
| **Subtotal** | **22,66 m** | **7,81 m** |
| **Com 10% de perdas** | **≈ 25 m** | **≈ 8,6 m** |

Não incluídos:

- tubos internos telescópicos das colunas (16×26, ~2,5 m);
- chapas e garfos;
- cardan, rótulas, motores, redutores, motor DD e banco.

## 11. Pontos em aberto e recomendações (obrigatório verificar antes de fabricar)

1. **Pivô × centro de gravidade no modo caminhão** (*crítico*). No modo
   F1 o CG do conjunto fica próximo da altura da junta. No modo caminhão
   o quadril sobe 300 mm e o CG fica **~250–300 mm acima** da cruzeta.
   A plataforma passa a se comportar como um **pêndulo invertido**: os
   motores precisam de torque contínuo para segurá-la e a resposta piora.
   Recomenda-se uma das opções:
   - torre do pivô com **altura regulável** (2–3 posições de furação,
     subindo a cruzeta e as rótulas superiores junto);
   - motores e redutores dimensionados **pelo modo caminhão**.
2. **Espessura de parede dos tubos.** A especificação 20×30 / 10×20 não
   define a parede. Mínimo recomendado: **2,0 mm** no 20×30 da torre,
   das longarinas e das colunas do banco; **1,2–1,5 mm** no 10×20.
   Verificar a flexão das longarinas móveis (vão de ~1080 mm entre o pivô
   e a frente) com 100 kg de piloto mais cargas dinâmicas. Se o resultado
   for insuficiente, considerar 30×50 nas longarinas móveis.
3. **Dimensionamento dos motores.** O NEMA 34 com redutor planetário foi
   escolhido por proporção visual. É necessário calcular:
   - o torque estático (excentricidade do CG em relação ao pivô × massa
     móvel);
   - o torque dinâmico (inércia de arfagem e rolagem × aceleração
     angular desejada);
   - a margem contra perda de passo. Avaliar servomotores ou drivers de
     passo em malha fechada.
4. **Limites de software.** A rolagem geométrica máxima (±17,9°) é
   excessiva para conforto e segurança. Limitar no firmware e prever
   **chaves de fim de curso** nos braços.
5. **Segurança.**
   - Botão de **parada de emergência** ao alcance do piloto.
   - Proteção das manivelas (região de esmagamento junto às hastes).
   - Partida dos motores sempre em posição neutra.
6. **Fixação ao piso.** Pés niveladores com borracha. Opcionalmente,
   ancoragem ou lastro na traseira para conter a reação dos motores.
7. **Itens não modelados:** monitor e suporte (fixar na plataforma-móvel
   para não haver movimento relativo à visão), câmbio sequencial/H, freio
   de mão, eletrônica (fonte, drivers, controlador), passagem de cabos.

## 12. Organização do modelo 3D

```
Cockpit_Simulador
├── Plataforma_Base            (tag Estrutura)
│   ├── Base_Longarina_*, Base_Travessa_*, Base_Diagonal, Base_Suporte_Motor_*
│   ├── Torre_Pivo
│   ├── Junta_Universal_Cardan
│   ├── Atuadores_Traseiros
│   │   ├── Atuador_E
│   │   └── Atuador_D
│   └── Batente_Coluna_*, Batente_Borracha_*
└── Plataforma_Movel
    ├── Chassi_Movel           (tag Estrutura)
    ├── Config_F1              (tag Config_F1)
    │   ├── Manequim_F1_172cm
    │   ├── Banco_F1
    │   ├── Pedaleira_F1
    │   └── Volante_F1
    └── Config_Caminhao        (tag Config_Caminhao)
        └── (mesma estrutura)
```

**Cenas salvas no arquivo:**

| Cena | Mostra |
|---|---|
| `F1 - 3/4` | Configuração F1, vista dianteira-esquerda |
| `F1 - Lateral` | Configuração F1, vista lateral esquerda |
| `Caminhao - 3/4` | Configuração caminhão, vista dianteira-esquerda |
| `Caminhao - Lateral` | Configuração caminhão, vista lateral esquerda |
| `Base e Atuadores` | Somente a estrutura, vista traseira-direita (motores, manivelas, hastes) |

Ao abrir, o arquivo mostra a configuração F1 (tag `Config_Caminhao`
oculta).

## 13. Regeneração do modelo

O arquivo `modelo/gerar_modelo.py` é **paramétrico**. As posturas estão
nos dicionários `CONFIG_F1` e `CONFIG_CAMINHAO`, e banco, pedaleira e
volante são **derivados da postura**. Para gerar uma nova configuração
(por exemplo, "GT"), basta alterar ou adicionar um dicionário.

O script roda no ambiente de execução Python do conector **Trimble
SketchUp MCP** (ferramenta `build_model`), e não no console Ruby do
SketchUp desktop. Veja as instruções no `README.md`.
