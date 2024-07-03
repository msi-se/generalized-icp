#import "@preview/minideck:0.2.1"


#import "@preview/lovelace:0.3.0": *


#let (template, slide, title-slide, pause, uncover, only) = minideck.config(
  theme: minideck.themes.simple.with(variant: "light")
)
#show: template

#set text(16pt)

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
  = Demo: Eigene Implementierung in Python - Code
]

#slide[
  = Demo: Eigene Implementierung in Python - Ergebnis
]