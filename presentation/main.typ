#import "@preview/minideck:0.2.1"
#import "@preview/lovelace:0.3.0": *


// compile:                    typst compile main.typ
// compile with speaker notes: typst compile main.typ --input with-comments="true" with-notes.pdf

#let template
#let slide
#let title-slide
#let pause
#let uncover
#let only

#if sys.inputs.at("with-comments", default: "false") == "true" {
  (template, slide, title-slide, pause, uncover, only) = minideck.config(
    theme: minideck.themes.simple.with(variant: "light"),
    height: 40cm,
  )
} else {
  (template, slide, title-slide, pause, uncover, only) = minideck.config(
    theme: minideck.themes.simple.with(variant: "light"),
  )
}

#show: template

#set text(size: 16pt)
#set text(lang: "de")

#let comment(body) = {
  let showInPDF = sys.inputs.at("with-comments", default: "false") == "true"
  // let showInPDF = true
  if showInPDF == true [
    #place(
      bottom,
      dx: 0cm,
      dy: 0cm,
    )[
      #box(fill: rgb("#ededed"), width: 100%, height: 45%, inset: 0.2cm)[
        = Speaker Notes
        #set text(size: 13pt)
        #columns(2)[
          #body
        ]
      ]
    ]
  ] else {
    none
  }
}

#title-slide[
  #place(
    dx: 8.5cm,
    dy: 14.5cm,
  )[
    #rotate(0deg)[
      #image(
        "./assets/HTWG_IN_Markenzeichen_klein_pos_2C.png",
        width: 30%,
        fit: "cover",
      )
    ]
  ]

  = Generalized Iterative Closest Point
  Mündliche Prüfung in der Vorlesung Autonome Roboter

  bei Prof. Dr.-Ing. Michael Blaich

  #datetime.today().display("[day].[month].[year]")

  #v(5cm)

  Johannes Brandenburger, Moritz Kaltenstadler, Fabian Klimpel


]

#slide[
  = Agenda

  + Einführung
  + Theorie
    + Mathematische Grundlagen
    + Standard-ICP
    + Point-to-Plane-ICP
    + Generalized-ICP
    + Ergebnisse von #cite(<segal-gicp>, form: "author")
  + Demo: Eigene Implementierung in Python
  + Implementierung in ROS - Versuch
    + Versuchsaufbau
    + Parameterisierung
    + Implementierung ICP & GICP
    + Problem
    + Zeitmessung
    + Maps
  + Auswertung
  + Fazit
  + (Bild-)Quellen
]

#slide[
  #set page(background: [
    #place(
      dx: 6cm,
      dy: 12cm,
    )[
      #rotate(20deg)[
        #box(
          fill: rgb("#f3f3f3"),
        )[
          #image(
            "./assets/paper-cover.svg",
            width: 50%,
            fit: "cover",
          )
        ]
      ]
    ]
  ])

  = Einführung

  - ICP: *Iterative Closest Point*
    - Scan-Matching-Algorithmus
    - Schätzung der Transformation zwischen zwei Punktwolken
    - Anwendung in der Lokalisierung mit z.B. LiDAR-Sensoren

  #v(0.5cm)

  - GICP: *Generalized-ICP*
    - veröffentlicht von Segal, Haehnel & Thrun (2009)
    - Stanford University
    - Ziel: ICP-Algorithmus verbessern und verallgemeinern
    - Standard-ICP & point-to-plane in *generelles Framework* überführen
    - *probabilistische* Betrachtung
    - Nutzung *Oberflächenstruktur* aus beiden Scans (Kovarianzmatrizen) → *plane-to-plane*

  #comment[
    - ICP Verfahren schon in Vorlesung
    - grober Überblick
    - ICP steht für Iterative Closest Point
    - Scan-Matching-Algorithmus
    - Schätzung der Transformation zwischen zwei Punktwolken
    - Anwendung in der Lokalisierung mit z.B. LiDAR-Sensoren

    - wurde 2009 von Segal, Haehnel & Thrun verbessert
    - an Stanford University Verfahren entwickelt
    - Generalized-ICP
    - Ziel: ICP-Algorithmus verbessern und verallgemeinern
      - Probleme Standard-ICP:
        - nicht sehr robust
        - sehr empfindlich gegenüber der Parameterwahl
        - daher schlecht in mehreren Szenarien einsetzbar
      - Standard-ICP & point-to-plane in generelles Framework überführen
      - 2 große Änderungen:
        - Wahrscheinlichkeitstheorie
        - Nutzung planarer Strukturen
          - Punktwolke sind nicht random im Raum verteilt
          - haben Struktur zb von Wänden
          - GICP nutzt diese Struktur in Form von Kovarianzmatrizen
            - Überleitung Kovarianzmatrizen -> Mathematische Grundlagen
  ]

]


#slide[
  = Theorie - Mathematische Grundlagen

  === Kovarianzmatrix

  - beschreibt die Streuung von Zufallsvariablen
  - für Punkte in Punktwolken: Verteilung der Punkte in der Umgebung

  #v(1cm)

  === Maximum Likelihood Estimation (MLE)

  - Schätzverfahren für Parameter von Wahrscheinlichkeitsverteilungen
  - der Parameter wird ausgewählt, der die beobachteten Daten am wahrscheinlichsten macht
  - oft verwendet um: $arg max_p ...$ / $arg min_p ...$ zu finden

  #comment[
    - Mathematische Grundlagen sind lediglich für ein gemeinsames Verständnis
    - sodass auch Leute, die nicht die Vorlesung besucht haben, den Vortrag verstehen könnten

    - Kovarianzmatrix beschreibt die Streuung von Zufallsvariablen
    - in unserem Kontext: Verteilung von Punkte in der Umgebung
    - wo unsere Punkte mit welcher Wahrscheinlichkeit liegen

    - Maximum Likelihood Estimation brauchen wir für die Schätzung der Transformation bei ICP und GICP
    - Schätzverfahren für Parameter von Wahrscheinlichkeitsverteilungen
    - versucht quasi die Parameter zu finden, die eine gegebene Wahrscheinlichkeitsverteilung am besten beschreiben
  ]

]



#slide[
  = Theorie - Standard-ICP

  - *Iterative Closest Point* (ICP) ist ein Algorithmus, um die Transformation zwischen zwei Punktwolken zu schätzen
  - vergleicht korrespondierende Punkte in beiden Wolken
  - minimiert die quadratischen Abstände korrespondierender Punkte

  #columns(2)[
    #text(size: 14.9pt)[
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
    ]
    #colbreak()

    // icp gif
    #figure(
      caption: [Standard-ICP @icp-notebook],
      [
        #image("./assets/icp.gif", width: 100%, format: "gif")
      ],
    )

  ]

  #comment[
    - wie schon gesagt: Standard ICP lässt uns Transformation zwischen zwei Punktwolken schätzen
    - dabei ist er sehr einfach und schnell
    - vergleicht korrespondierende Punkte in beiden Wolken
    - minimiert die quadratischen Abstände korrespondierender Punkte
    - schätzt so die Transformation
    - in Pseudocode dargestellt
    - Startwert für Transformation $T_0$
      - kann bereits eine sinnvolle Schätzung sein (z.b. Odometrie)
    - Loop bis der Algorithmus konvergiert (daher auch "Iterative")
    - für jeden Punkt in der Sourcewolke $b_i$ wird der nächste Punkt in der Targetwolke $A$ gesucht
    - dann wird geschaut ob der Abstand kleiner als das Threshold $d_(max)$ ist
      - Parameter steuert also, welche Punkte berücksichtigt werden und welche nicht
      - ist je nach Anwendungsszenario unterschiedlich
        - wenn Roboter schneller fährt, dann muss $d_(max)$ größer sein
        - schwierig einzustellen
    - Punkt wird gewichtet, oder nicht
    - am Ende jedes Durchlaufs wird die Transformation berechnet
      - durch Veränderung der Transformationsparameter
      - so dass die quadratischen Abstände minimiert werden
  ]

]



#slide[
  = Theorie - Point-to-Plane-ICP
  #columns(2)[
    - *Point to Plane ICP* ist eine Erweiterung des ICP Algorithmus
    - vergleicht korrespondierende Punkte in einer Wolke zu Ebenen in der anderen
    - Ebenen wird durch Punkt und Normalenvektor definiert

    #v(1cm)
    $T arrow.l arg min_T {sum_i ((T dot.op b_i - m_i) dot.op bold(n_i))^2}$

    #colbreak()
    #figure(
      caption: [Point-to-Plane-ICP @point-to-plane-icp],
      [
        #image("./assets/point-to-plane.png", width: 80%)
      ],
    )
  ]

  #comment[
    - Point-to-Plane-ICP ist eine Erweiterung des Standard-ICP
    - Standard-ICP betrachtet keine Oberflächenstruktur
      - bei Laserscan zum Beispiel:
        - wenn 2 mal eine Wand gescant
        - Punkte zwar durch die Abtastrate unterschiedlichen Stellen im Raum
        - kann sein, dass die Bewegung zur Wand nicht wirklich groß war
    - um dem entgegenzuwirken, vergleicht Point-to-Plane-ICP Punkte in einer Wolke zu Ebenen in der anderen
    - Ebenen wird durch Punkt und Normalenvektor definiert
    - Optimierungsfunktion in der letzten Zeile des Algorithmus:
      - minimiert quadratische Abstände zwischen Punkt und Ebene
  ]

]


#slide[
  = Theorie - Standard-ICP, Point-to-Plane, Generalized-ICP

  #grid(
    columns: 2,
    [
      - *Point-to-Point*
        - Standard-ICP
        - vergleicht Punkt mit Punkt

      #v(0.2cm)

      - *Point-to-Plane*
        - vergleicht Punkt mit Ebene durch Normalenvektor

      #v(0.2cm)

      - *Generalized-ICP*
        - quasi "Plane-to-Plane"
        - vergleicht die Kovarianzmatrizen der nächsten Punkte → probabilistisch
        - wenn in Ebene → Kovarianzmatrix ist "flach"
    ],
    [
      #figure(
        caption: "Kovarianzmatrizen (eigene Darstellung)",
        [
          #image("./assets/cov-matr.png", width: 100%)
        ],
      )
    ],
  )

  #comment[

    - hier nochmal ein Überblick über die verschiedenen ICP-Verfahren

    - Point-to-Point: Standard-ICP
      - vergleicht Punkt mit Punkt
      - einfach und schnell
      - aber nicht robust sehr empfindlich gegenüber Parameterwahl

    - Point-to-Plane:
      - vergleicht Punkt mit Ebene durch Normalenvektor
      - besser als Standard-ICP
      - nutzt Oberflächenstruktur einer Punktwolke
      - aber eben nur von einer

    - Generalized-ICP:
      - quasi "Plane-to-Plane"
      - vergleicht die Kovarianzmatrizen der nächsten Punkte → probabilistisch
      - wenn in Ebene → Kovarianzmatrix ist "flach"
        - sieht man in Bild rechts
      - nutzt Oberflächenstruktur beider Punktwolken
      - welche Ergebnisse dies hat, dazu später mehr
  ]
]


#let gicp-algo = [
  #pseudocode-list[
    + $T arrow.l T_0$
    + *while* not converged *do*
      + *for* $i arrow.l 1$ *to* $N$ *do*
        + $m_i arrow.l$ `FindClosestPointInA`$(T dot.op b_i)$
        + $d_i^((T)) arrow.l b_i - T dot.op m_i$ #h(3em) #text(gray)[\// Residuum / Abstand]
        + *if* $parallel d_i^((T)) parallel lt.eq d_(max)$ *then*
          + $C_i^A arrow.l$ `computeCovarianceMatrix`$(T dot.op b_i)$
          + $C_i^B arrow.l$ `computeCovarianceMatrix`$(m_i)$
        + *else*
          + $C_i^A arrow.l 0$; #h(1em) $C_i^B arrow.l 0$
        + *end*
      + *end*
      + $T arrow.l arg min_T {
            sum_i d_i^(T)^T (C_i^B + T C_i^A T^T)^(-1) d_i^((T))
          }$ #h(1em) #text(gray)[\// Maximum Likelihood Estimation]
    + *end*
  ]
]

#slide[

  = Theorie - GICP-Algorithmus

  #gicp-algo


  #comment[
    - im Paper wurde Algorithmus nie zusammenhängend dargestellt
      - deshalb hier nochmals selber zusammengebaut
    - Anfang des Algorithmus gleich wie bei Standard-ICP
    - statt Gewichtung mit 0 oder 1 werden Kovarianzmatrizen verwendet
      - wie genau diese berechnet bzw gewählt werden, dazu mehr auf der nächsten Folie
    - Minimierungsfunktion am Ende des Schleifendurchlaufs anders
      - nutzt Gewichtungsmatrix, die aus den Kovarianzmatrizen berechnet wird
        - mittlere Teil der Minimierungsfunktion
    - im Paper wird für die Optimierung der Transformation also für arg min Maximum Likelihood Estimation verwendet
      - wählt Transformationsmatrix T so dass die Verteilung am wahrscheinlichsten ist
  ]

]


#slide[

  #place(
    dx: 14.2cm,
    dy: 1.8cm,
  )[
    #box(
      width: 5.3cm,
      height: 0.9cm,
      fill: rgb("#5a3bf538"),
      radius: 0.1cm,
    )
  ]

  #place(
    dx: -0.2cm,
    dy: 2.6cm,
  )[
    #box(
      width: 10cm,
      height: 1.8cm,
      fill: rgb("#5a3bf538"),
      radius: 0.1cm,
    )
  ]

  #place(
    dx: 13cm,
    dy: -1cm,
  )[
    #rotate(0deg)[
      #set text(size: 8.5pt)
      #gicp-algo
    ]
  ]

  = Theorie - GICP-Algorithmus
  == Variationen für Kovarianzmatrizen

  #pseudocode-list(line-numbering: none)[
    + $C_i^A arrow.l$ `computeCovarianceMatrix`$(T dot.op b_i)$
    + $C_i^B arrow.l$ `computeCovarianceMatrix`$(m_i)$
  ]

  - für *Standard-ICP* (Point-to-Point):
    - $C_i^A arrow.l 0$
    - $C_i^B arrow.l 1$ #h(3.3em) → keine Oberflächenstruktur berücksichtigt (einfache Gewichtung)

  #v(0.2cm)

  - für *Point-to-Plane*:
    - $C_i^A arrow.l 0$
    - $C_i^B arrow.l P_i^(-1)$ #h(2em) → $P_i$ ist die Projektionsmatrix auf die Ebene (beinhaltet Normalenvektor)

  #v(0.2cm)

  #columns(2)[

    - für *Plane-to-Plane* (im Paper vorgeschlagene Methode):
      - `computeCovarianceMatrix` berechnet Kovarianzmatrix unter Betrachtung der nächsten 20 Punkte
        - verwendet *PCA* (Principal Component Analysis/Hauptkomponentenanalyse)

    #colbreak()
    #figure(
      caption: [Plane-to-Plane @segal-gicp],
      [
        #image("./assets/plane-to-plane.png", width: 100%)
      ],
    )
  ]

  #comment[
    - Wahl der Kovarianzmatrizen sind also entscheidend für den GICP-Algorithmus
    - auch der Grund, warum "Generalized" ICP
      - denn es lässt sich auch Standard-ICP und Point-to-Plane-ICP dadurch abdecken
    - für Standard-ICP werden Kovarianzmatrizen einfach auf 0 bzw 1 gesetzt
      - dadurch werden die Punkte einfach gewichtet und es wird keine Oberflächenstruktur berücksichtigt
    - für Point-to-Plane-ICP werden die Source-Kovarianzmatrizen auf Projektionsmatrizen gesetzt
      - diese beinhalten den Normalenvektor der Ebene
      - Oberflächenstruktur der einen Wolke berücksichtigt
    - aber richtig gut erst bei dem im Paper vorgeschlagenen Verfahren
      - quasi "Plane to Plane"
    - hier werden wirklich Kovarianzmatrizen ausgerechnet
      - 20 umliegende Punkte werden betrachtet
      - Verteilung mit Hauptkomponentenanalyse bestimmt
      - wenn hier eine genauere, mathematische Erklärung gewünscht
        - später darauf eingehen
        - Folie vorbereitet
    - allerdings auch etwas mehr Rechenaufwand bei jeder Iteration
    - Berechnen Kovarianzmatrizen geschiet bei beiden Wolken -> Berücksichtigung beider Oberflächenstrukturen

    - Bild rechts zeigt Kovarianzmatrizen für paar Punkte
      - man sieht:
        - Ausrichtung/Wölbung stimmt mit Ebene überein
        - Kovarianzmatrix ist "flach"
  ]
]

#slide[
  = Theorie - GICP-Algorithmus
  == Berechnung der Kovarianzmatrizen bei Plane-to-Plane GICP

  ```py
  covariance_ground = np.array([[epsilon, 0], [0, 1]])
  covariance = np.cov(neighbors, rowvar=False)
  eigenvalues, eigenvectors = np.linalg.eig(covariance)
  ev = eigenvectors[:, np.argmax(eigenvalues)]     # vector with highest eigenvalue
  rotation_matrix = np.array([[ev[0], -ev[1]], [ev[1], ev[0]]])
  covariance_aligned = rotation_matrix @ covariance_ground @ rotation_matrix.T
  ```

  #v(0.5cm)

  #columns(2)[
    - nicht einfach die Kovarianzmatrix der Nachbars-Punkte
    - normalisiert
    - Kovarianzmatrix für die Ebene erstellt:
      $mat(epsilon, 0; 0, 1)$
      - $epsilon$ ist ein kleiner Wert
    - Lage der Punkte im Raum betrachtet
      - Hauptkomponentenanalyse (Eigenvektoren und Eigenwerte)
      - Eigenvektor mit höchstem Eigenwert
    - Einheits-Kovarianzmatrix wird um die Lage der Punkte im Raum gedreht

    #colbreak()

    #figure(
      caption: [Eigenvektoren spiegeln die Lage der Punkte im Raum wider @IDAPILecture14],
      [
        #image("./assets/pca.png", width: 70%)
      ],
    )
  ]

  #comment[
    - es wird nicht einfach die Kovarianzmatrix der Punkte genommen
    - sondern normalisiert erzeugt:
    - zunächst wird eine Kovarianzmatrix für die Ebene erstellt
      - epsilon ist ein kleiner Wert
    - dann wird die Lage der Punkte im Raum betrachtet
      - dies geschieht durch die Hauptkomponentenanalyse (Eigenvektoren und Eigenwerte)
    - die Einheits-Kovarianzmatrix wird dann um die Lage der Punkte im Raum gedreht
    - eine Alternative zur Hauptkomponentenanalyse wäre die Singular Value Decomposition
      - auch Rotation extrahiert werden kann
  ]
]


#slide[
  = Theorie - GICP-Algorithmus
  == Ergebnisse von #cite(<segal-gicp>, form: "author")

  - GICP *genauer* bei simulierten und realen Daten
  - immer noch relativ schnell und einfach
  - Nutzen von Oberflächenstruktur *minimiert Einfluss von falschen Korrespondenzen*
  - Parameter-Wahl für $d_(max)$ nicht mehr so kritisch → leichter einsetzbar in *unterschiedlichen Szenarien*

  #v(1cm)

  #figure(
    caption: [Durchschnittsfehler als Funktion von $d_(max)$ @segal-gicp],
    [
      #image("./assets/paper-results.png", width: 60%)
    ],
  )

  #comment[
    - insgesammt GICP robuster und genauer bei simulierten und realen Daten
    - dabei immer noch relativ schnell und einfach
      - auch wenn mehr Rechenaufwand durch Kovarianzmatrizen
    - Nutzen von Oberflächenstruktur minimiert Einfluss von falschen Korrespondenzen, was bei Standard-ICP ein Problem war
    - dadurch auch Parameter-Wahl für $d_(max)$ nicht mehr so kritisch
      - leichter einsetzbar in unterschiedlichen Szenarien
    - in Bild unten
      - Durchschnittsfehler als Funktion von $d_(max)$
      - Generalized-ICP ganz unten, hat am die niedrigsten Fehler

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


  #comment[
    *Version 1*
    - Visualisierung mit generierten Input-Wolken
    - Source und Target Punktwolke
      - Quadrat und Kreis als Punktwolken
      - Beide Punktwolken haben unterschiedliches Rauschen
      - Unterschiedlich viele Punkte
      - Keine Sortierung
    - Covarianzmatrizzen werden dargestellt
    - Bissle duchsteppen

    *Version 2*
    - Bissle mitm Roboter rumfahren
    - Einmal um Kreis
    => schlechte Rotation estimation
    - Einmal an gerader Wand entlang
    => schlechte Translation estimation
  ]
]

#slide[
  = Demo: Eigene Implementierung in Python
  == Code Overview - GICP

  GICP-Funktion:

  ```python
  def gicp(source_points, target_points, max_iterations=100, tolerance=1e-6,
      max_distance_correspondence=150, max_distance_nearest_neighbors=50):
      for iteration in range(max_iterations):
          # Calculate covariance covariance matrices for weight matrices
          # Will be used in loss function
          ...

          # Minimize the loss function function
          transformation = scipy.optimize.fmin_cg(
              f=loss,
              x0=transformation,
              fprime=grad_loss)

          # Check for convergence
          if delta_loss < tolerance:
              break

      return transformation
  ```
]

#slide[
  = Demo: Eigene Implementierung in Python
  == Code Overview - Roboter

  #v(1cm)
  GICP-Aufruf:
  #v(1cm)
  ```python
  transformation_matrix = gicp(
              _source_points,
              _target_points,
              max_distance_nearest_neighbors=200,
              tolerance=1,
          )
  ```
]

#slide[
  = Demo: Eigene Implementierung in Python
  == Code Overview - Roboter

  #v(1cm)
  Berechnung der neuen Schätzung:
  #v(1cm)

  ```python
  # Get delta x, y and yaw from transformation matrix
  delta_x = -transformation_matrix[0, 2]
  delta_y = -transformation_matrix[1, 2]
  delta_yaw = -np.arctan2(transformation_matrix[1, 0], transformation_matrix[0, 0])

  # Calculate estimations for new position and orientation
  new_estimated_x = last_estimated_x
      + delta_x * math.cos(last_estimated_yaw)
      - delta_y * math.sin(last_estimated_yaw)

  new_estimated_y = last_estimated_y
      + delta_x * math.sin(last_estimated_yaw)
      + delta_y * math.cos(last_estimated_yaw)

  new_estimated_yaw = last_estimated_yaw + delta_yaw
  ```
]

#slide[
  = Implementierung in ROS - Versuch
  == Leitfrage

  #v(4cm)
  #set align(center)
  #text(size: 30pt)[
    Ist Generalized-ICP besser als Standard-ICP und wie verhält sich der Algorithmus in unterschiedlichen Szenarien?
  ]
]

#slide[
  = Implementierung in ROS - Versuch
  == Versuchsaufbau

  - Vorbedingungen:
    - Bag File
    - Skript für Nodes
    - Yaml files als configuration

  #v(1cm)
  - Szenarien:
    - Drei unterschiedliche Maps
    - Unterschiedliche Parameterisierung
    - Fünf Durchgänge mit Standardparameterisierung (ICP & GICP)

  #v(1cm)
  - Auswertung:
    - Bag Files mit Topics
    - Python-Skript zur Auswertung
    - Bokeh für Visualisierung

  #comment[
    - Vorbedingungen:

      - Bag File: beispiel trajektorie wird in ROS-Bag-Datei gespeichert, die als Input für die Experimente dient und immmer den gleichen Datensatz zur Verfügung stellt
      - Skript für Nodes: Skripte, die die notwendigen ROS-Nodes starten und konfigurieren.
      - Yaml-Dateien als Konfiguration: Konfigurationsdateien im YAML-Format, die die Parameter und Einstellungen für die Experimente enthalten.

    - Szenarien:
      - Drei unterschiedliche Maps: Die Experimente werden auf drei verschiedenen Karten durchgeführt, um die Robustheit der Algorithmen in unterschiedlichen Umgebungen zu überprüfen.
      - Unterschiedliche Parameterisierung: Es werden verschiedene Parametereinstellungen getestet, um die Auswirkungen auf die Genauigkeit und Laufzeit der Algorithmen zu analysieren.
      - Fünf Durchgänge mit Standardparameterisierung (ICP & GICP): Gleiches Szenario wird fünfmal mit den Standardparametern für sowohl ICP als auch GICP durchgeführt, um die Varianz der Algorithmen zu überprüfen.

    - Auswertung
      - Bag Files mit Topics: Die während der Experimente generierten Daten werden in ROS-Bag-Dateien gespeichert, die verschiedene Topics enthalten, die für die Auswertung relevant sind.
      - Python-Skript zur Auswertung: Ein Python-Skript analysiert die gespeicherten Daten und berechnet relevante Metriken wie RMSE, Orientation Error, Position Error.
      - Bokeh für Visualisierung: Die Ergebnisse der Auswertungen werden mit Hilfe von Bokeh visualisiert, um die Unterschiede zwischen den Algorithmen und den verschiedenen Szenarien anschaulich darzustellen.

    - Diese strukturierte Vorgehensweise gewährleistet eine umfassende und nachvollziehbare Bewertung der Performance von GICP und ICP in unterschiedlichen Testumgebungen. Desweiteren erlaubt es wiederholbare Experimente und eine einfache Vergleichbarkeit der Ergebnisse. Es wird jeweils nur ein Parameter verändert, um die Auswirkungen auf die Ergebnisse zu analysieren.
  ]
]


#slide[
  = Implementierung in ROS - Versuch
  == Parameterisierung

  ```cpp
  //name des odom topics
  this->declare_parameter("odom_topic", "");
  //name des icp topics
  this->declare_parameter("gicp_result_topic", "");
  //name des zeitmessung topics
  this->declare_parameter("alignment_time_topic", "");
  //parameter ob gicp oder icp verwendet wird
  this->declare_parameter("gicp", false);
  //ob manuelle transformation gepublished wird
  this->declare_parameter("publish_tf", false);

  //icp parameter
  this->declare_parameter("max_correspondence_distance", 0.0);
  this->declare_parameter("maximum_iterations", 0);
  this->declare_parameter("transformation_epsilon", 0.0);
  this->declare_parameter("euclidean_fitness_epsilon", 0.0);
  ```

  #comment[
    -max correspondence distance: maximale distanz mit der ein punkt aus der source wolke mit einem punkt aus der target wolke korrespondieren kann
    -maximum iterations: maximale anzahl an iterationen die der algorithmus
    -transformation epsilon:die maximal zulässige quadratische Differenz zwischen zwei aufeinanderfolgenden Transformationen
    -euclidean fitness epsilon: Der maximal zulässige euklidische Fehler zwischen zwei aufeinanderfolgenden Schritten in der ICP-Schleife
    fehler = durschnitt aller Unterschiede zwischen den korrespondierenden Punkten
  ]
]

#slide[
  = Implementierung in ROS - Versuch
  == Parameterisierung über YAML file

  #text(size: 15pt)[
    #columns(2, gutter: 0pt)[
      ```YAML
      gicp_lio:
        ros__parameters:
          gicp: True
          publish_tf: False
          alignment_time_topic: "galignment_time"
          odom_topic: "glidar_odom"
          gicp_result_topic: "glidar_odom_eval"
          max_correspondence_distance: 0.2
          maximum_iterations: 100
          transformation_epsilon: 0.000000001
          euclidean_fitness_epsilon: 0.00001
      ```
      #colbreak()
      ```YAML
      gicp_lio:
        ros__parameters:
          gicp: False
          publish_tf: False
          alignment_time_topic: "alignment_time"
          odom_topic: "lidar_odom"
          gicp_result_topic: "lidar_odom_eval"
          max_correspondence_distance: 0.2
          maximum_iterations: 100
          transformation_epsilon: 0.000000001
          euclidean_fitness_epsilon: 0.00001
      ```
    ]
  ]
]


#slide[
  = Implementierung in ROS - Versuch
  == Implementierung ICP & GICP

  - ICP:
  ```cpp
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(src);
  icp.setInputTarget(tgt);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
  icp.align(*output);
  ```


  - GICP:
  ```cpp
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
  gicp.setInputSource(src);
  gicp.setInputTarget(tgt);
  pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
  gicp.align(*output);
  ```

  #comment[
    - ICP und GICP sind beide in der PCL-Bibliothek implementiert
    - beide benötigen als Input die Source- und Target-Punktwolke
    - align-Funktion berechnet die Transformation
    - icp.align(output): Führt den ICP-Algorithmus aus, um die beste Übereinstimmung zwischen der Quell- und Zielpunktwolke zu finden und speichert das Ergebnis in der Ausgabepunktwolke
  ]
]


#slide[
  = Implementierung in ROS - Versuch
  == Zeitmessung

  ```cpp
  auto start = std::chrono::high_resolution_clock::now();
  icp.align(*output);
  transformation = icp.getFinalTransformation();
  auto finish = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double> elapsed = finish - start;
  std_msgs::msg::Float64 time_msg;
  time_msg.data = elapsed.count();
  time_publisher_->publish(time_msg);
  ```

  #comment[
    - Zeitmessung wird mit der C++-Chrono-Bibliothek durchgeführt
    - start und finish markieren den Anfang und das Ende der Zeitmessung
    - elapsed berechnet die Dauer der Zeitmessung
    - time_msg wird erstellt und die Zeit wird in die Nachricht geschrieben
    - time_publisher veröffentlicht die Nachricht auf dem entsprechenden Topic
    - überprüfung ob GICP auch wirklich aufwendiger in der Berechnung
    - wie ändern Parameter die Zeit
  ]
]


#slide[
  = Implementierung in ROS - Versuch
  == Problem: Aufsummierende Fehler

  ```cpp
  // reduce tick speed in topic_callback
  tick++;
  if (tick % 3 != 0) {
    return;
  }
  ```
  #grid(
    columns: 2,
    [
      #figure(
        caption: "Trajectory plot with higher tick speed",
        [
          #image("./assets/trajectory_plot_corr_dist_old.png", width: 74%)
        ],
      )
    ],
    [
      #figure(
    caption: "Trajectory plot with lower tick speed",
    [
      #image("./assets/trajectory_plot_variance_round_1.png", width: 74%)
    ] // TODO: hier vielleicht neues bild auch mit corr dist
  )
    ],
  )

  #comment[
    - Problem: Aufsummierende Fehler
    - schlechte Ergebnisse wenn jeder Tick berücksichtigt wird
    - weniger Aufrufe der (G)ICP Algorithmen
    - weniger Datenpunkte
    - deutlich bessere Ergebnisse wenn nur jeder dritte Tick berücksichtigt wird
    - rechenleistung wird gespart
  ]
]


#slide[
  = Implementierung in ROS - Versuch
  == Turtlebot3 World

  #figure(
    caption: "Screenshot Gazebo",
    [
      #image("./assets/turtlebot3_world.jpg", width: 80%)
    ],
  )

  #comment[
    - Turtlebot3 World: Standard Gazebo-Welt für Turtlebot3
    - sechseck und 9 zylinder
    - mittel komplexe Umgebung
    - viele Datenpunkte
  ]
]

#slide[
  = Implementierung in ROS - Versuch
  == Turtlebot3 DQN Stage 1

  #figure(
    caption: "Screenshot Gazebo",
    [
      #image("./assets/turtlebot3_dqn_stage1.jpg", width: 80%)
    ],
  )

  #comment[
    - rechteck
    - keine Hindernisse
    - einfachste Umgebung
    - wenig Datenpunkte
  ]
]

#slide[
  = Implementierung in ROS - Versuch
  == Turtlebot3 ICP World

  #figure(
    caption: "Screenshot Gazebo",
    [
      #image("./assets/turtlebot3_icp_world.jpg", width: 80%)
    ],
  )

  #comment[
    - icp welt von aufgabe 2
    - unterschiedlichste objekte
    - keine begrenzung der umgebung
    - komplexeste Umgebung
    - mittel viele Datenpunkte
  ]
]




#slide[
  = Auswertung
  == Drei unterschiedliche Maps

  #v(3cm)
  #grid(
    columns: 3,
    [
      #image("./assets/evaluation/trajectory_plot_variance_round_1.png", width: 90%)
    ],
    [
      #image("./assets/evaluation/trajectory_plot_turtlebot3_world.png", width: 90%)
    ],
    [
      #image("./assets/evaluation/trajectory_plot_turtlebot3_dqn_stage1.png", width: 90%)
    ],
  )
]

#slide[
  = Auswertung
  == Unterschiedliche Parameterisierung

  #v(3cm)
  #grid(
    columns: 3,
    [
      #image("./assets/evaluation/trajectory_plot_corr_dist.png", width: 90%)
    ],
    [
      #image("./assets/evaluation/trajectory_plot_fit_eps.png", width: 90%)
    ],
    [
      #image("./assets/evaluation/trajectory_plot_iterations.png", width: 90%)
    ],
  )

  #comment[
    *corr_dist*: max correspondence distance
    - icp mit 0,1 deutlich schlechter in position und orientation
    - gicp mit 0,1 sogar am besten
    - keine großen Unterschiede von 0,2 und 0,5
    => gicp robuster gegenüber Parameterwahl
    - alignment time bei gicp deutlich höher
    - alignment time bei gicp und icp parameterunabhängig
    *fit_eps*: euclidean fitness epsilon
    - icp mit 0,1 deutlich schlechter in position und orientation
    - gicp mit 0,00001 am besten
    - rest relativ ähnlich
    - alignment gleich wie bei corr_dist
    => gicp robuster gegenüber Parameterwahl
    *iterations*: maximum iterations
    - gicp mit 10 deutlich schlechter in position und orientation
    - icp mit 10 deutlich schlechter in orientation
    - rest relativ ähnlich
    - alignment time bei gicp 10 iterations niedriger => algorithmus terminiert
    - restliche Zeit bei gicp und icp kaum unterschiede
  ]
]

#slide[
  = Auswertung
  == Fünf Durchgänge mit Standardparameterisierung (ICP & GICP)

  #v(2cm)
  #set align(center)
  #image("./assets/evaluation/trajectory_plot_variance.png", width: 50%)

  #set align(left)
  #comment[
    - durchläufe relativ konstant
    - gibt abweichungen in position und orientation
    - gibt fast perfekt gleiche trajectories
    => Ist Simulation
  ]
]

#slide[
  = Fazit

  - Leitfrage: Ist Generalized-ICP besser als Standard-ICP und wie verhält sich der Algorithmus in unterschiedlichen Szenarien?

  #v(1cm)

  - keine deutlich besseren Ergebnisse bei GICP
  - Parameter-Wahl nicht so kritisch wie bei ICP kann bestätigt werden
  - keine großen Unterschiede zwischen ICP & GICP in den unterschiedlichen Maps
  - GICP ist aufwendiger in der Berechnung
  - Abhängigkeit von der Simulationsumgebung
  - Unterschiede zu Paper, da 3D-Scan und deutlich mehr Punkte
]


#slide[
  = (Bild-)Quellen

  #bibliography(
    "sources.yml",
    style: "apa",
    title: "",
  )
]