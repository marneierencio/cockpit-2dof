# Arquivos para o SolidWorks

> **Limite importante.** Os formatos nativos `.SLDPRT` e `.SLDASM` são binários fechados da Dassault. Só o próprio
> SolidWorks consegue gravá-los. Por isso esta pasta traz **STEP (AP214) reais, com sólidos B-rep** e uma **macro**
> que os converte em arquivos nativos dentro do SolidWorks. Renomear um `.step` para `.sldprt` não funciona: o
> SolidWorks recusa o arquivo.

## Conteúdo

| Arquivo | O que é |
|---|---|
| `pecas/*.step` | Um arquivo por peça distinta (73). Conjuntos de tubos soldados, mais as chapas soldadas neles, são **uma peça só**: um sólido fundido. |
| `Cockpit_2DOF_F1.step` | Montagem completa na pose F1, com subconjuntos que seguem a árvore de componentes dinâmicos do SketchUp (base, chassi, carro do banco, bielas, banco, assento, encosto, pedaleira, mastro, volante, porta). |
| `manifesto_pecas.csv` | Código, arquivo, nome, quantidade, instâncias, tipo (soldado/avulsa), nº de tubos, material, volume e massa. |
| `posicoes.csv` | Matriz 4×4 (metros) de cada instância no sistema do modelo (X lateral, Y longitudinal, Z vertical). |
| `juntas.csv` | Cada grupo móvel: pai, tipo (rotação em X ou translação), origem do eixo, giro e curso de F1 para caminhão. |
| `converter_para_solidworks.bas` | Macro VBA: `pecas/*.step` → `sldprt/*.SLDPRT` e `Cockpit_2DOF_F1.step` → `sldasm/Cockpit_2DOF_F1.SLDASM`. |

## Como os tubos foram modelados

- **Ocos**, com a parede do projeto (1,2 mm) e cantos externos arredondados (raio 1,5 × parede = 1,8 mm).
- Os **encontros** seguem as regras da seção 7 do memorial v2: meia-esquadria entre tubos de mesma seção,
  encosto na face do tubo maior, corte reto nas pontas livres.
- Verificação feita antes da exportação: **0 sobreposições** entre tubos do mesmo conjunto soldado
  (`python3 ferramentas/cad.py`); todos os 73 STEP reabrem como sólidos válidos; massa dos tubos ocos −1,9 % em
  relação ao gerador (os cantos arredondados explicam a diferença).
- Cada peça avulsa (pino, bucha, motor, mola…) foi modelada no seu próprio sistema de coordenadas; a posição
  de cada instância está na montagem e em `posicoes.csv`.

## Como obter os arquivos nativos

1. Abra o SolidWorks.
2. **Ferramentas › Macro › Executar…** e escolha `converter_para_solidworks.bas`. Deixe o arquivo nesta pasta.
3. A macro cria `sldprt/` (uma peça por arquivo) e `sldasm/` (montagem). Se o SolidWorks perguntar sobre salvar os
   componentes referenciados, aceite.
4. Se a montagem abrir como peça única: **Ferramentas › Opções › Importar › STEP** e marque a importação como
   montagem.

> A macro **não foi executada** em um SolidWorks (não há instalação disponível onde ela foi escrita). Ela usa só
> chamadas documentadas da API. Teste com um arquivo antes de rodar tudo; erros aparecem na janela *Immediate*.

## Simulação e testes

O STEP não leva *mates*. Para o estudo de movimento:

1. Fixe `SOLD-BASE` (plataforma-base).
2. Em `juntas.csv`, a plataforma móvel gira em torno do cardan (eixo Y em y = 700, z = 268 para rolagem; eixo X
   para arfagem). Crie duas relações de revolução (ou use um *universal joint*) entre `SOLD-BASE` e `SOLD-MOV`.
3. Cada linha de `juntas.csv` é uma relação entre o grupo e o pai: `rotacao_X` = revolução em torno do eixo X que
   passa pela origem indicada; `translacao` = deslizamento ao longo de Y com o curso indicado.
4. Os atuadores são as hastes traseiras (`M-CO` hastes de ligação) acionadas pelas manivelas da base.
5. Use *Interference Detection* na montagem para repetir as verificações de `ferramentas/verificar.py` em outras poses.

## Pontos a revisar

- Chapas que, no modelo, não encostam em nenhum tubo (viram peças avulsas): ver item 10 do memorial v2.
- As orelhas do subchassi do banco ficam **dentro** da seção oca do tubo (item 11).
- O desenho do SketchUp mostra os tubos como sólidos cheios com os mesmos cortes. O STEP é a referência para
  massa e para o SolidWorks.

## Como regenerar

```bash
pip install cadquery
python3 ferramentas/exportar_cad.py     # ≈ 1 min
```
