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

  Moritz Kaltenstadler, Fabian Klimpel, Johannes Brandenburger
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
  = Theorie

  #grid(
    columns: 2,
    [
    - "point-to-point" (Standard-ICP)
    - "point-to-plane"
      - vergleicht Punkt mit Ebene nurch Normalenvektor
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
  = Demo: Eigene Implementierung in Python - Code

  - Paper sehr mathematisch
  - Algorithmus wurde nie komplett gezeigt
  - zwar Implementierungen auf GitHub, aber nicht wirklich lesbar
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

  #emph[$->$ LIVE DEMO]
]

#slide[
  = Demo: Eigene Implementierung in Python - Ergebnis
]