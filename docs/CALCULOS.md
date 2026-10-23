# Cálculos — revisão R3

> Gerado por `ferramentas/analisar.py` a partir do modelo paramétrico (`modelo/gerar_modelo.py`).
> Não edite os números à mão: altere as premissas no script e rode de novo.

Marcação de confiança: **[Certo]** sai direto da geometria/massas do modelo; **[Provável]** é cálculo de
engenharia com premissa explícita; **[Chutando]** é valor de catálogo típico, que precisa ser trocado pelo
valor do componente comprado.

## 1. Premissas

| Item | Valor | Confiança |
|---|---|---|
| Aço dos tubos | SAE 1010/1020 conformado a frio, σe ≥ 250 MPa | [Provável] |
| Tensão admissível (cargas de pico) | σe/2 = 125 MPa | [Provável] |
| Faixa admissível em solda (cargas cíclicas) | Δσ ≤ 50 MPa (≈ categoria 50 da EN 1993-1-9) | [Provável] |
| Fator dinâmico em cargas do piloto | 1,5 | [Provável] |
| Piloto | 90 kg, 170–175 cm, frações de massa de Dempster | [Provável] |
| Força no freio / no volante / na barra da porta | 500 / 300 / 600 N | [Provável] |
| Rendimento do redutor | 0,90 | [Chutando] |
| Curvas de torque dos motores | formas típicas de catálogo (tabela no script) | [Chutando] |
| Limite de software do ângulo da manivela | ±60° a partir da horizontal | decisão de projeto |
| Monitores | nenhum (óculos de RV) — não entram em massas nem estrutura | decisão do usuário |

## 2. Massas, centro de gravidade e inércias [Certo]

Somadas peça a peça do modelo (tubos pelo perfil × comprimento, chapas pelo volume, comprados pela massa de catálogo,
piloto pelos segmentos). O pivô (cruzeta do cardan) fica em y = 700 mm, z = 268 mm.

| Conjunto | Massa (kg) | CG x / y / z (mm) | Altura do CG sobre o pivô (mm) | I arfagem (kg·m²) | I rolagem (kg·m²) |
|---|---|---|---|---|---|
| Plataforma móvel + piloto — F1 | 186,0 | 0 / 701 / 737 | 469 | 84,9 | 55,1 |
| Plataforma móvel + piloto — caminhão | 186,0 | 0 / 700 / 949 | 681 | 139,7 | 117,9 |
| Plataforma-base (fixa) | 39,5 | -0 / 308 / 124 | - | 18,0 | 2,6 |

**Balanceamento automático.** O gerador desloca, ao longo de y, o conjunto banco + pedaleira + volante de cada
posição (sem mudar a ergonomia) até o CG longitudinal cair sobre o pivô nos dois modos. Resultado: F1 recuado
17 mm e caminhão recuado 171 mm em relação às posturas da v2. Erro final: 0,6 mm (F1) e 0,3 mm (caminhão).
Com o CG sobre o pivô, o torque estático dos motores com a plataforma nivelada é praticamente nulo nos dois modos.
Recuar o conjunto do caminhão **não** piora o F1: cada modo é balanceado de forma independente. [Certo]

O CG fica **acima** do pivô (pêndulo invertido): 469 mm no F1 e 681 mm no caminhão. Inclinada, a plataforma tende
a cair para o lado da inclinação com rigidez negativa m·g·h = 856 N·m/rad (F1) e 1244 N·m/rad (caminhão). As molas
de centragem (seção 4) compensam isso.

## 3. Envoltória de movimento [Certo]

Hastes a 640 mm do pivô (braço de arfagem) e a ±310 mm do eixo (braço de rolagem); manivela de 100 mm.
Com o ângulo da manivela limitado a ±60° (deslocamento máximo da haste 86,6 mm), os limites de software ficam:

| Movimento | Limite de software | Limite geométrico (manivela no ponto morto) |
|---|---|---|
| Arfagem pura | ±7,8° | ±8,9° |
| Rolagem pura — F1 | ±16,2° | ±18,5° |
| Rolagem — caminhão (limite lógico) | ±6° | — |

Arfagem e rolagem dividem o mesmo curso: |640·sen(arfagem)| + |310·sen(rolagem)| ≤ 86,6 mm em cada haste.
Na prática, o software (SimTools/FlyPT Mover) faz essa mistura e prioriza um eixo.
A rolagem do F1 foi **ampliada**: a R2 recomendava limitar o software a ±8–10° (geométrico ±17,9°); a R3 libera
±16,2° no F1, com a manivela limitada a ±60° para a força na haste não disparar. Um curso maior pediria manivela maior,
o que aumenta o torque (proporcional ao raio) e passa do batente dianteiro na arfagem. Por isso não foi adotado.

Os **batentes de borracha** dianteiros tocam a longarina em ≈ −8,4° de arfagem. Isso fica além do limite de
software (−7,8°): eles são proteção mecânica, não fim de curso de trabalho.

## 4. Molas de centragem — o "freio" econômico [Provável]

Em caso de queda de energia ou de emergência (drivers desabilitados), a plataforma deve voltar ao nível sozinha.
A solução mais barata que garante isso é um **par de molas de tração por eixo**, com olhais entre a base e o chassi:

- **arfagem:** duas molas na linha de centro, em y = 300 e y = 1100;
- **rolagem:** duas molas em x = ±200, na linha do pivô.

Molas de tração (e não de compressão) só puxam o chassi para baixo, então o cardan fica sempre comprimido. A rigidez
rotacional de cada par é 1,10 × m·g·h do modo caminhão = **1368 N·m/rad**. Assim a plataforma é estável nos dois
modos, com os motores desligados.

| Par | Braço (mm) | k de cada mola (N/mm) | Curso ± (mm) | Pré-tensão: alongamento / força | Força máxima (N) | Comprimento instalado (mm) |
|---|---|---|---|---|---|---|
| arfagem | 400 | 4,3 | 54 | 64 mm / 274 N | 506 | 280 |
| rolagem | 200 | 17,1 | 56 | 66 mm / 1126 N | 2082 | 260 |

Pré-carga total das molas sobre o pivô (constante, não cíclica): 2801 N.
Torque estático que os motores fazem contra as molas no fim de curso:

- F1, rolagem ±16,2°: 145 N·m na plataforma. Sem molas seriam 239 N·m a favor da queda, então as molas reduzem a carga
  e tornam a plataforma segura.
- Caminhão, rolagem ±6°: 13 N·m.

Complementos sem custo:

- configurar os drivers para curto-circuitar as fases ao desabilitar, o que dá frenagem dinâmica no motor de passo;
- ligar um botão de emergência que corta o *enable* dos drivers.

Freio eletromagnético no motor fica como opcional, caso o protótipo mostre necessidade.

## 5. Motores e redução [Provável / curvas Chutando]

Modelo: plataforma rígida girando no cardan.

- Momento que o motor fornece: M = I·θ'' + (K_molas − m·g·h)·θ, mais 5 % de atrito.
- Força em cada haste: M / (2·L·cos θ).
- Torque na manivela: F·r·cos φ.
- Torque no motor: T/(N·η) + J_motor·N·φ''.
- Rotação do motor: N·φ'.

O sinal é senoidal, de amplitude A. O motor atende quando o torque necessário fica abaixo de 80 % da curva em todo
o ciclo. O torque do driver é limitado para que a força na haste nunca passe de **1800 N**, valor usado no
dimensionamento estrutural (seção 6). A tabela mostra a **frequência máxima (Hz)** possível para cada movimento.

| Motor | Redução | F1 arfagem ±7,8° | F1 rolagem ±16,2° | F1 rolagem ±8,0° | F1 arfagem ±4,0° | Caminhao arfagem ±7,8° | Caminhao rolagem ±6,0° |
|---|---|---|---|---|---|---|---|
| NEMA 34 malha fechada 12 N.m (80 VCC) | 10:1 | 1,47 | 1,06 | 1,39 | 2,07 | 1,20 | 1,05 |
| NEMA 34 malha fechada 12 N.m (80 VCC) | 15:1 | 1,50 | 1,11 | 1,45 | 2,22 | 1,24 | 1,08 |
| NEMA 34 malha fechada 12 N.m (80 VCC) | 20:1 | 1,48 | 1,12 | 1,44 | 2,20 | 1,24 | 1,07 |
| NEMA 34 malha fechada 12 N.m (80 VCC) | 25:1 | 1,44 | 1,11 | 1,43 | 2,14 | 1,23 | 1,07 |
| NEMA 34 malha fechada 12 N.m (80 VCC) | 30:1 | 1,38 | 1,09 | 1,41 | 2,08 | 1,20 | 1,06 |
| Servo CA 750 W (pico 7,1 N.m ate 3000 rpm) | 10:1 | 1,57 | 1,03 | 1,15 | 1,74 | 1,20 | 0,82 |
| Servo CA 750 W (pico 7,1 N.m ate 3000 rpm) | 15:1 | 1,87 | 1,20 | 1,35 | 2,08 | 1,45 | 1,00 |
| Servo CA 750 W (pico 7,1 N.m ate 3000 rpm) | 20:1 | 1,91 | 1,28 | 1,44 | 2,20 | 1,55 | 1,07 |
| Servo CA 750 W (pico 7,1 N.m ate 3000 rpm) | 25:1 | 1,78 | 1,26 | 1,43 | 2,14 | 1,48 | 1,07 |
| Servo CA 750 W (pico 7,1 N.m ate 3000 rpm) | 30:1 | 1,66 | 1,22 | 1,41 | 2,08 | 1,41 | 1,06 |

Velocidade de pico correspondente: 2π·f·A. Por exemplo, a rolagem F1 de ±8° a 1,44 Hz chega a 72 °/s. [Provável]

**Escolha:** NEMA 34 malha fechada 12 N.m (80 VCC) com redutor planetário **20:1** em cada lado.
Dentro do conjunto que mantém os motores de passo do pedido original, é o que dá o melhor equilíbrio entre torque
(arfagem/rolagem grandes e lentas) e rotação (pequenas e rápidas).

Limite de torque no driver: **5,0 N·m no motor**, o que corresponde a 1800 N na haste com a manivela a 60°.
Se o protótipo pedir respostas mais rápidas, o caminho é o servo CA de 750 W com a mesma redução (linhas da tabela).
A troca não muda a estrutura, que já está dimensionada para a força limitada.

O modo caminhão é mais pesado em inércia (I rolagem 118 contra 55 kg·m² no F1, pelo CG mais alto), mas pede
movimentos menores e mais lentos. A rolagem limitada a ±6° pelo software é coerente com isso. [Provável]

## 6. Esforços e tensões [Provável]

Duas famílias de carga:

- **Pico:** força máxima da haste (1800 N, limitada no driver), peso × 1,5, pré-carga das molas e cargas do piloto.
  Comparada com σe/2.
- **Cíclica:** força nas hastes no sinal representativo (F1, rolagem ±8° mais arfagem ±4°, a 80 % da frequência
  máxima) = 1135 N por haste. A faixa 2×F é comparada com a admissível em solda.

| Elemento | Caso | Valor | Unid. | Limite | Situação | Observação |
|---|---|---|---|---|---|---|
| Trelica soleira E - banzos (longarina 30x50 + banzo inferior 30x50) | pico: forca maxima da haste (1800 N) | 30,2 | MPa | 125 | OK | forca no banzo 5.6 kN |
| Trelica soleira E - banzos | fadiga: faixa 2 x forca ciclica (1135 N) | 38,0 | MPa | 50 | OK | amplitude representativa F1 |
| Trelica soleira E - banzo comprimido (flambagem, L=400) | pico | 5620 | N | 173422 | OK | Pcr/2 = 173 kN |
| Anel lado D (trelica, banzos 20x30) | pico | 17,5 | MPa | 125 | OK | altura util 575 mm |
| Longarina sem trelica (referencia - o que a soleira evita) | pico | 447 | MPa | 125 | NAO ATENDE | mostra por que a soleira E e obrigatoria |
| Travessas do pivo (3 x 30x50) | pico (peso x 1.5 + molas + 2 hastes) | 70,9 | MPa | 125 | OK | P = 9.1 kN |
| Travessas do pivo (3 x 30x50) | fadiga | 35,2 | MPa | 50 | OK |  |
| Chapa do pivo 300x140x8 (sobre 3 travessas) | pico | 76,5 | MPa | 125 | OK | vao 50 mm entre travessas, braco 25 mm |
| Pernas da torre do pivo (4 x 20x30) | pico, compressao | 28,8 | MPa | 125 | OK |  |
| Pernas da torre do pivo (flambagem) | pico | 3294 | N | 106459 | OK | Pcr/2 = 106 kN |
| Bielas do banco (caminhao, 66 graus) | compressao | 404 | N | 37879 | OK | Pcr/3 = 38 kN |
| Tirantes de arrasto (caminhao) | tracao/compressao | 3,3 | MPa | 125 | OK | F = 374 N |
| Hastes comando encosto (par, F1) | compressao (cada) | 2651 | N | 59878 | OK | M = 396 N.m, braco 75 mm |
| Travessas do carro do banco (20x30) - apoio da haste do encosto | flexao | 98,3 | MPa | 125 | OK | suporte a 35 mm da longarina do carro |
| Alavancas encosto (chapa 12x70) | flexao na raiz | 16,2 | MPa | 125 | OK | raiz a ~80% do braco |
| Haste comando assento (F1) | compressao | 909 | N | 57949 | OK |  |
| Haste comando pedaleira (frenagem) | axial | 1692 | N | 111606 | OK | M = 329 N.m |
| Poste + mao-francesa da haste da pedaleira (20x30) | axial (triangulado) | 18,5 | MPa | 125 | OK | F_haste = 1692 N |
| Bracos do volante (2 x 30x50, eixo fraco) | flexao | 69,3 | MPa | 125 | OK |  |
| Pino seguidor came do volante (Ø12, chapa 4) | esmagamento | 20,0 | MPa | 125 | OK |  |
| Colunas do mastro (2 x 30x50, sem contar a guia no anel) | flexao no pe | 61,1 | MPa | 125 | OK | a guia no anel reduz muito este valor |
| Braco da manivela (chapa 10x50) | flexao | 43,2 | MPa | 125 | OK |  |
| Haste M16 (barra roscada) | tracao | 11,5 | MPa | 125 | OK | area resistente 157 mm2 |
| Barra da porta (30x50x1,2, em pe) | pessoa apoiada no meio do vao (600 N) | 54,3 | MPa | 125 | OK | vao 932 mm |
| Trilhos do banco (30x50 deitado) | flexao, vao 350 mm | 33,3 | MPa | 125 | OK |  |

A linha "longarina sem treliça" **não atende de propósito**. Ela mostra por que a v3 ganhou a **treliça de
soleira** sob a longarina esquerda: a abertura da porta interrompe ali a treliça do anel. A força da haste traseira
precisa chegar às travessas do pivô por uma estrutura alta, e não pela longarina sozinha em flexão. Na direita,
essa função é do próprio anel treliçado. [Provável]

### 6.1 Pinos e furos [Provável]

Pino em corte duplo. Esmagamento comparado entre a parede do tubo (1,2 mm) e a bucha soldada (parede 3 mm).

| Articulação | Ø pino (mm) | Força (N) | τ no pino (MPa) | Esmagamento em parede 1,2 mm (MPa) | Esmagamento em bucha 3 mm (MPa) |
|---|---|---|---|---|---|
| Bielas do banco (Ø16) | 16 | 404 | 1,0 | 11 | 4 |
| Tirante de arrasto (Ø12) | 12 | 374 | 1,7 | 13 | 5 |
| Hastes do encosto (Ø16) | 16 | 2651 | 6,6 | 69 | 28 |
| Haste do assento (Ø16) | 16 | 909 | 2,3 | 24 | 9 |
| Haste da pedaleira (Ø16) | 16 | 1692 | 4,2 | 44 | 18 |
| Eixo do volante (Ø25) | 25 | 1922 | 2,0 | 32 | 13 |
| Eixo assento/encosto (Ø20) | 20 | 6039 | 9,6 | 126 | 50 |

**Regra de fabricação:** nenhum pino trabalha direto na parede do tubo. Todo furo de articulação leva **bucha de
aço soldada** (parede ≥ 3 mm, folga H8/f7) ou orelha de chapa ≥ 5 mm. Na parede de 1,2 mm o esmagamento é
aceitável no pico, mas ovaliza com o uso cíclico e gera folga, que se sente no volante e no banco. [Provável]

### 6.2 Frequência natural do braço do volante [Provável]

Dois braços 30x50 em balanço de 598 mm (eixo fraco), com volante G920 + mesa (≈ 3,7 kg) na ponta: f ≈ **34 Hz**.
Isso fica bem acima das frequências de movimento (< 3 Hz). Interação com o force feedback do G920 é improvável.

## 7. Verificação geométrica automática [Certo]

`ferramentas/verificar.py` representa cada peça por uma caixa orientada e procura interferências:

- entre peças de componentes diferentes, em 5 pontos da transição F1 → caminhão;
- com a barra da porta aberta e a meio curso, nos dois modos;
- entre o manequim e a estrutura, nos dois modos;
- entre a plataforma móvel e a base, em todas as 340 poses de arfagem × rolagem alcançáveis pela manivela.

Contatos previstos (pinos, articulações e carros sobre trilhos) são filtrados por pontos de junta.
Resultado: **0 interferências** na transição, porta e manequim.
Na envoltória de movimento, os únicos contatos são os batentes dianteiros (Longarina principal D × Batente borracha D, 6,3 mm a -9°), como previsto. A menor folga
restante é de 17,7 mm (Coluna anel E y25 × Chapa suporte motor 6mm E).

Mecanismos de 1 movimento (síntese e simulação cinemática real, não interpolação):

| Haste | Comprimento (mm) | Menor ângulo de transmissão no curso | Erro de comprimento F1/caminhão (mm) | Desvio da animação DC (mm) |
|---|---|---|---|---|
| HASTE_ENC_E | 286 | 30° | 0,00 | 1,8 |
| HASTE_ENC_D | 286 | 30° | 0,00 | 1,8 |
| HASTE_ASS | 291 | 31° | 0,00 | 0,7 |
| ARRASTO_E | 68 | 31° | 0,00 | 0,1 |
| ARRASTO_D | 68 | 31° | 0,00 | 0,1 |
| HASTE_PED | 210 | 44° | 0,00 | 1,5 |

Critério de projeto: ângulo de transmissão ≥ 25° em todo o curso. O menor valor encontrado é 30°, então nenhuma
haste passa perto de ponto morto. A lei de
movimento de cada componente dinâmico (deslizamento e giro em função de `modo`) é um polinômio ajustado à
cinemática real. Por isso a animação no SketchUp mostra as hastes sem esticar.

