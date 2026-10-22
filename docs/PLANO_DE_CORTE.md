# Plano de corte dos tubos — revisão R3

> Gerado por `ferramentas/analisar.py`. Barras comerciais de 6 m, perda de serra de 3 mm por corte e 10 mm de
> esquadrejamento em cada ponta da barra. Distribuição pelo método *first-fit decreasing*.

**Como ler:** cada linha é uma barra. Corte as peças na ordem listada e marque cada peça com o código, que é o
mesmo do desenho e da lista de materiais.

Os comprimentos são os do eixo da peça no modelo, de face a face das peças que ela encontra. Para peças com corte
em ângulo (diagonais, pernas e bielas), o comprimento é medido no eixo e o ângulo está na lista de materiais.
Recomenda-se cortar com +2 mm e ajustar na montagem (gabarito).

## Tubo 10x20

Barras de 6 m: **1** · peças: 5 · comprimento líquido: 3,34 m · aproveitamento: 56 %

| Barra | Peças (código: comprimento mm) | Usado (mm) | Sobra (mm) |
|---|---|---|---|
| 1 | B-T11: 1057, M-T13: 594, BC-T16: 564, BC-T17: 564, M-T12: 557 | 3348 | 2632 |

## Tubo 20x30

Barras de 6 m: **7** · peças: 93 · comprimento líquido: 41,12 m · aproveitamento: 98 %

| Barra | Peças (código: comprimento mm) | Usado (mm) | Sobra (mm) |
|---|---|---|---|
| 1 | B-T01: 2050, B-T02: 2050, A-T10: 1760, B-T18: 110 | 5979 | 1 |
| 2 | A-T16: 807, A-T20: 800, A-T19: 691, A-T15: 664, A-T14: 654, B-T03: 640, B-T04: 640, B-T05: 640, M-T22: 411 | 5971 | 9 |
| 3 | B-T06: 640, B-T07: 640, B-T08: 640, B-T09: 640, B-T10: 640, A-T18: 635, A-T01: 585, A-T02: 585, A-T03: 585, PE-T12: 360 | 5977 | 3 |
| 4 | A-T04: 585, A-T05: 585, A-T06: 585, A-T07: 585, A-T08: 585, A-T09: 585, A-T17: 582, A-T11: 570, A-T12: 570, PE-T07: 526, M-T19: 155, M-T17: 10, M-T20: 10 | 5969 | 11 |
| 5 | PE-T09: 526, M-T04: 510, M-T08: 510, M-T09: 510, M-T10: 510, M-T11: 510, M-T28: 510, M-T29: 510, M-T23: 442, BC-T13: 440, BC-T14: 440, BC-T15: 440, M-T18: 80 | 5974 | 6 |
| 6 | PE-T05: 440, BC-T01: 427, BC-T02: 427, PE-T10: 400, PE-T11: 400, BC-T05: 390, BC-T06: 390, BC-T07: 390, BC-T08: 390, PE-T16: 380, BC-T09: 360, BC-T11: 360, PE-T15: 320, PE-T06: 298, PE-T08: 298, B-T14: 264 | 5979 | 1 |
| 7 | BC-T20: 292, BC-T03: 280, BC-T04: 280, BC-T21: 280, BC-T22: 280, BC-T18: 276, BC-T19: 276, B-T15: 264, B-T16: 264, B-T17: 264, PE-T13: 240, PE-T01: 230, PE-T03: 230, M-T21: 221, PE-T14: 210, A-T13: 195, VO-T06: 152, VO-T07: 152, B-T12: 140, B-T13: 140, A-T21: 123, BC-T10: 119, BC-T12: 119, B-T19: 110, PE-T02: 90, PE-T04: 90, BC-T23: 68, BC-T24: 68 | 5534 | 446 |

## Tubo 30x50

Barras de 6 m: **3** · peças: 19 · comprimento líquido: 14,81 m · aproveitamento: 82 %

| Barra | Peças (código: comprimento mm) | Usado (mm) | Sobra (mm) |
|---|---|---|---|
| 1 | M-T01: 1780, M-T02: 1780, M-T24: 1110, M-T26: 1110 | 5789 | 191 |
| 2 | M-T25: 1035, M-T27: 1035, PO-T01: 902, VO-T01: 728, VO-T02: 728, M-T03: 510, M-T05: 510, M-T06: 510 | 5979 | 1 |
| 3 | M-T07: 510, VO-T03: 504, VO-T04: 473, VO-T05: 473, M-T16: 457, M-T15: 407, M-T14: 250 | 3092 | 2888 |

## Compra de tubos

| Perfil | Barras de 6 m | Observação |
|---|---|---|
| 10x20 | 1 (+1 de reserva recomendada) | sobras ≥ 300 mm servem para gabaritos e calços |
| 20x30 | 7 (+1 de reserva recomendada) | sobras ≥ 300 mm servem para gabaritos e calços |
| 30x50 | 3 (+1 de reserva recomendada) | sobras ≥ 300 mm servem para gabaritos e calços |
