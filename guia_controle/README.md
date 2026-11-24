# Guia de projeto de controladores (TCC)

Material de estudo: modelagem, cálculo de ganhos, lugar das raízes, controle ótimo,
tempo discreto e a malha interna da PX4, para cada controlador do `aero_pursuit`.
Inclui scripts MATLAB/Simulink para a comparação de desempenho.

```
guia_controle/
├── main.tex, preambulo.tex, referencias.bib
├── capitulos/          # 14 capítulos + apêndice
├── figuras/            # PNGs + gerar_figuras.py (python3 gerar_figuras.py)
│   └── dados/          # logs da simulação antes/depois (Cap. 14)
└── matlab/
    ├── parametros.m, modelo_px4.m, cenario.m, sim_perseguicao.m
    ├── analise_margens.m, projeto_*.m, identificar_px4.m
    ├── otimizar_ganhos.m, comparar_controladores.m
    ├── controladores/  # ctrl_pid, ctrl_lqr, ctrl_lqi, ctrl_smc, ctrl_mpc, ctrl_hermite, ctrl_guiamento
    └── simulink/       # criar_modelo_simulink.m, rodar_simulink.m
```

## Compilar o PDF
```bash
cd guia_controle
latexmk -pdf main.tex      # ou: pdflatex main && bibtex main && pdflatex main && pdflatex main
# Sem TeX instalado (sem sudo), num contêiner:
docker run --rm -v "$PWD":/doc -w /doc ubuntu:24.04 bash -c "apt-get update -qq && \
  DEBIAN_FRONTEND=noninteractive apt-get install -y -qq --no-install-recommends latexmk \
  texlive-latex-extra texlive-lang-portuguese texlive-science texlive-fonts-recommended lmodern \
  && latexmk -pdf main.tex && chown -R $(id -u):$(id -g) /doc"
```
Pacotes usados: babel (brazil), amsmath, siunitx, tcolorbox, listings, cleveref, booktabs.
Todos fazem parte do TeX Live completo (`texlive-full`) ou do MiKTeX/Overleaf.

## MATLAB
Abra o MATLAB em `guia_controle/matlab` e rode, nesta ordem:
`analise_margens`, `projeto_lqr`, os demais `projeto_*`, `simulink/rodar_simulink`,
`otimizar_ganhos('lqr')` (etc.) e `comparar_controladores`.
