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
    - Nutzung *Oberflächenstruktur* aus beiden Scans (Kovarianzmatrizen) →  *plane-to-plane*
]

#slide[
  = Theorie - Mathematische Grundlagen

  === Kovarianzmatrix

  - beschreibt die Streuung von Zufallsvariablen
  - für Punkte in Punktwolken: Verteilung der Punkte in der Umgebung

  #v(1cm)

  === Maximum Likelihood Estimation (MLE)

  - Schätzverfahren für Parameter von Wahrscheinlichkeitsverteilungen
  - der Paramter wird ausgewählt, der die beobachteten Daten am wahrscheinlichsten macht  
  - oft verwendet um: $arg max_p ...$ / $arg min_p ...$ zu finden
]


#slide[
  = Theorie - Standard-ICP

  - *Iterative Closest Point* (ICP) ist ein Algorithmus, um die Transformation zwischen zwei Punktwolken zu schätzen
  - vergleicht korrespondierende Punkte in beiden Wolken
  - minimiert die quadratischen Abstände korrespondierender Punkte

  #columns(2)[
    #pseudocode-list[
      + $T arrow.l T_0$
      + *while* not converged *do*
        + *for* $i arrow.l 1$ *to* $N$ *do*
          + $m_i arrow.l$ `FindClosestPointInA`$(T dot.op b_i)$
          + *if* $parallel m_i - b_i parallel lt.eq d_(max)$ *then*
            + $w_i arrow.l 1$
          + *else*
            + $w_i arrow.l 0$
          + *end*
        + *end*
        + $T arrow.l arg min_T {sum_i w_i (parallel T dot.op b_i - m_i parallel)^2}$
      + *end*     
    ]
    #colbreak()

    // icp gif
    #figure(
      caption: "Standard-ICP (https://nbviewer.org/github/niosus/notebooks/blob/master/icp.ipynb)",
      [
        #image("./assets/icp.gif", width: 100%, format: "gif")
      ]
    )

  ]
]


// https://nbviewer.org/github/niosus/notebooks/blob/master/icp.ipynb 

#slide[
  = Theorie - Standard-ICP, point-to-plane, Generalized-ICP

  #grid(
    columns: 2,
    [
    - *point-to-point* (Standard-ICP)
    - *point-to-plane*
      - vergleicht Punkt mit Ebene durch Normalenvektor
    - *Generalized-ICP*
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

  = Theorie - GICP-Algorithmus

  #pseudocode-list[
    + $T arrow.l T_0$
    + *while* not converged *do*
      + *for* $i arrow.l 1$ *to* $N$ *do*
        + $m_i arrow.l$ `FindClosestPointInA`$(T dot.op b_i)$
        + $d_i^((T)) arrow.l b_i - T dot.op m_i$  #h(3em)  #text(gray)[\// Residuum / Abstand]
        + *if* $parallel d_i^((T)) parallel lt.eq d_(max)$ *then*
          + $C_i^A arrow.l$ `computeCovarianceMatrix`$(T dot.op b_i)$
          + $C_i^B arrow.l$ `computeCovarianceMatrix`$(m_i)$
        + *else*
          + $C_i^A arrow.l 0$; #h(1em) $C_i^B arrow.l 0$
        + *end*
      + *end*
      + $T arrow.l arg min_T {sum_i d_i^(T)^T  (C_i^B + T C_i^A T^T)^(-1) d_i^((T))}$  #h(1em) #text(gray)[\// Maximum Likelihood Estimation]
    + *end*     
  ]
]

#slide[

  = Theorie - GICP-Algorithmus - Variationen für Kovarianzmatrizen

  #pseudocode-list(
    line-numbering: none
    )[
    + $C_i^A arrow.l$ `computeCovarianceMatrix`$(T dot.op b_i)$
    + $C_i^B arrow.l$ `computeCovarianceMatrix`$(m_i)$
  ]

  - für *Standard-ICP* (point-to-point):
    - $C_i^A arrow.l 0$
    - $C_i^B arrow.l 1$ #h(3.3em) → keine Oberflächenstruktur berücksichtigt

#v(0.2cm)

- für *point-to-plane*:
    - $C_i^A arrow.l 0$
    - $C_i^B arrow.l P_i^(-1)$ #h(2em) → $P_i$ ist die Projektionsmatrix auf die Ebene (beinhaltet Normalenvektor)

#v(0.2cm)

#columns(2)[

  - für *plane-to-plane* (im Paper vorgeschlagene Methode):
      - `computeCovarianceMatrix` berechnet Kovarianzmatrix unter Betrachtung der nächsten 20 Punkte
        - verwendet *PCA* (Principal Component Analysis/Hauptkomponentenanalyse)

  #colbreak()

  #figure(
    caption: "Plane-to-plane (\"Generalized-ICP\" von Segal et al.)",
    [
      #image("./assets/plane-to-plane.png", width: 100%)
    ]
  )

  ]
]

#slide[
  = Demo: Eigene Implementierung in Python

  - Paper sehr mathematisch
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

  #emph[$->$ LIVE DEMO]

  #v(1cm)

  #emph[$->$ CODE OVERVIEW]
]

