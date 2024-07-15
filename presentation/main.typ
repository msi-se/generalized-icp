#import "@preview/minideck:0.2.1"
#import "@preview/lovelace:0.3.0": *


#let (template, slide, title-slide, pause, uncover, only) = minideck.config(
  theme: minideck.themes.simple.with(variant: "light")
)
#show: template

#set text(size: 16pt)
#set text(lang: "de")

#title-slide[
  = Generalized Iterative Closest Point
  Mündliche Prüfung in der Vorlesung Autonome Roboter
  
  bei Prof. Dr.-Ing. Michael Blaich

  #datetime.today().display("[day].[month].[year]")

  // space
  #v(5cm)

  Johannes Brandenburger, Moritz Kaltenstadler, Fabian Klimpel
]

#slide[
  = Agenda

  + Einführung
  + Theorie
  + Demo: Eigene Implementierung in Python
  + Implementierung in ROS
  + Experiment
    + Aufbau
    + Durchführung
    + Ergebnisse
    + ...
  + Fazit
]

#slide[
  = Theorie

  - Einzige wirkliche Quelle: "Generalized-ICP" von Segal, Haehnel & Thrun (2010)
    - Ziel: Iterative-Closest-Point-Algorithmus (ICP) verbessern
    - Standard-ICP & point-to-plane in *generelles Framework* überführen
    - *Probabilistische* Betrachtung
    - Nutzung *Oberflächenstruktur* aus beiden Scans (Kovarianzmatrizen)
]

#slide[
  = Theorie - Standard-ICP, point-to-plane, Generalized-ICP

  #grid(
    columns: 2,
    [
    - "point-to-point" (Standard-ICP)
    - "point-to-plane"
      - vergleicht Punkt mit Ebene durch Normalenvektor
    - Generalized-ICP
      - quasi "plane-to-plane"
      - vergleicht die Kovarianzmatrizen der nächsten Punkte → probabilistisch
      - wenn in Ebene → Kovarianzmatrix ist "flach"
  ], [
    #figure(
      caption: "Kovarianzmatrizen (eigene Darstellung)",
      [
        #image("./assets/cov-matr.png", width: 100%)
      ]
    )
  ])
  
  #v(1cm)
]


#slide[
  = Theorie - Standard-ICP

  - Einzige wirkliche Quelle: "Generalized-ICP" von Segal, Haehnel & Thrun (2010)
    - Ziel: Iterative-Closest-Point-Algorithmus (ICP) verbessern
    - Standard-ICP & point-to-plane in *generelles Framework* überführen
    - *Probabilistische* Betrachtung
    - Nutzung *Oberflächenstruktur* aus beiden Scans (Kovarianzmatrizen)
]

#slide[

  = Theorie - GICP Algorithmus

  #pseudocode-list[
    + $T arrow.l T_0$
    + *while* not converged *do*
      + *for* $i arrow.l 1$ *to* $N$ *do*
        + $m_i arrow.l "FindClosestPointInA"(T dot.op b_i)$
        + *if* $parallel m_i - T dot.op b_i parallel lt.eq d_(max)$ *then*
          + $C_i^A arrow.l "computeCovarianceMatrix"(T dot.op b_i)$
          + $C_i^B arrow.l "computeCovarianceMatrix"(a_i)$
        + *else*
          + $C_i^A arrow.l 0$; #h(1em) $C_i^B arrow.l 0$
        + *end*
        + $d_i^((T)) arrow.l b_i - T dot.op a_i$  #h(3em)  \// Residuum / Abstand
      + *end*
      + $T arrow.l arg min_T {sum_i d_i^(T)^T  (C_i^B + T C_i^A T^T)^(-1) d_i^((T))}$ 
    + *end*     
  ]
]


#slide[
  = Theorie

  - Algorithmus (ausführlicher als Orginal):
  #pseudocode-list[
    + $T$ = $T_0$
    + $"CA"$ = computeCovarianceMatrices($A$)
    + *while* has not converged *do*
      + *for* $i$ = 1 to $N$ *do*
        + $m_i$ = findClosestPointInA($T * b_i$)
        + $c_i$ = computeCovarianceMatrix($T * b_i$)
        + $w_i$ = computeWeightMatrix($c_i, "CA"_i$)
      + *end*
      + $T$ = optimizeLossFunction($T, A, B, W$)
    + *end*
  ]
]


#slide[
  = Demo: Eigene Implementierung in Python

  - Paper sehr mathematisch
  - Algorithmus wurde nie komplett gezeigt
  - zwar Implementierungen auf GitHub, aber nicht wirklich lesbar

  #v(0.5cm)

  - daher eigene Implementierung - vor allem für Verständnis
  - eigene *2D-GICP-Funktion*
    - Input: Punktwolken $A$ und $B$, ...
    - Output: Transformationsmatrix $T$, ...
  - Version 1:
    - Visualisierung mit generierten Input-Wolken
    - iterativ durch die Steps klicken
  - Version 2:
    - Simulation eines Roboters mit LiDAR-Sensor
    - Live-Berechnung der Transformation + Visualisierung

  #v(1cm)

  #emph[$->$ CODE OVERVIEW]

  #v(1cm)

  #emph[$->$ LIVE DEMO]
]
