# cockpit-2dof

Cockpit de simulador para jogos de computador, com **plataforma móvel de 2
graus de liberdade** (arfagem e rolagem) e **duas posições de pilotagem
ajustáveis na mesma estrutura**: Fórmula 1 e caminhão. Dimensionado para
uma pessoa de **170 a 175 cm**.

| Configuração F1 | Configuração caminhão |
|---|---|
| ![F1](imagens/preview_config_F1.png) | ![Caminhão](imagens/preview_config_caminhao.png) |

## Conceito

- **Plataforma-base**, fixa no piso, que sustenta a **plataforma-móvel**,
  onde o piloto senta.
- União por um **pivô central** (junta cardan) próximo ao centro de
  gravidade da plataforma-móvel, mais **duas hastes traseiras** ligadas a
  **braços de manivela acionados por motores de passo**.
- Motores no mesmo sentido produzem arfagem (aceleração/frenagem); em
  sentidos opostos, rolagem (força G lateral).
- Estrutura em **tubo 20×30** nas regiões de maior esforço e **tubo
  10×20** nas de menor esforço.
- **Banco, pedaleira e volante** ajustáveis em posição, altura e ângulo.

Detalhes completos no **[Memorial Descritivo](docs/MEMORIAL-DESCRITIVO.md)**:
dimensões, peças, ajustes, cursos, lista de tubos e pontos em aberto.

> ⚠️ **Status: anteprojeto.** Não houve cálculo estrutural nem
> dimensionamento dos motores. Leia a seção 11 do memorial antes de
> fabricar. O ponto mais crítico é o centro de gravidade alto no modo
> caminhão.

## Estrutura do repositório

| Caminho | Conteúdo |
|---|---|
| `docs/MEMORIAL-DESCRITIVO.md` | Memorial descritivo do projeto |
| `modelo/gerar_modelo.py` | Gerador paramétrico do modelo SketchUp |
| `modelo/cockpit_simulador_F1_caminhao.skp` | Modelo 3D (SketchUp), quando adicionado |
| `imagens/` | Prévias do modelo |

## Abrindo o modelo

Abra o `.skp` no SketchUp (desktop ou web) e use as abas de cena:

| Cena | Mostra |
|---|---|
| `F1 - 3/4` / `F1 - Lateral` | Postura de Fórmula 1 |
| `Caminhao - 3/4` / `Caminhao - Lateral` | Postura de caminhão |
| `Base e Atuadores` | Somente a estrutura, motores, manivelas e hastes |

As duas configurações ficam nas tags `Config_F1` e `Config_Caminhao`. A
estrutura comum fica na tag `Estrutura`.

## Regenerando o modelo

`modelo/gerar_modelo.py` roda no ambiente Python do conector **Trimble
SketchUp MCP**, e não no console Ruby do SketchUp desktop.

1. Envie o conteúdo inteiro do arquivo como `code` de uma chamada
   `build_model` com `clean: true`.
2. Chame `save_model` para obter o `.skp`.

Para mudar uma postura, edite os dicionários `CONFIG_F1` /
`CONFIG_CAMINHAO` no fim do script. Banco, pedaleira e mastro do volante
se reposicionam automaticamente a partir da postura do manequim. O
resultado da execução traz alcance dos braços, folgas e posições-chave
para conferência.
