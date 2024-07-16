#import "@preview/minideck:0.2.1"
#import "@preview/lovelace:0.3.0": *


#let (template, slide, title-slide, pause, uncover, only) = minideck.config(
  theme: minideck.themes.simple.with(variant: "light"),
)
#show: template

#set text(size: 16pt)
#set text(lang: "de")

#let comment(body) = {

  let showInPDF = sys.inputs.at("with-comments", default: "false") == "true"
  if showInPDF == true {
    slide[
      = Kommentar
      #box(fill: rgb("#ededed"), width: 100%, height: 90%, inset: 1cm)[
        body
      ]
    ]
  } else {
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
  + Demo: Eigene Implementierung in Python
  + Implementierung in ROS
  + Experiment
    + Aufbau
    + Durchführung
    + Ergebnisse
    + ...
  + Fazit
]

#comment[
  Test
]

#slide[
  #set page(background: [
    #place(
      dx: 6cm,
      dy: 13cm,
    )[
      #rotate(20deg)[
        #box(
          fill: rgb("#f3f3f3"),
        )[
          #image(
            "./assets/paper-cover.png",
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
]

#slide[
  = Theorie - Point-to-Plane-ICP
  #columns(2)[
    - *Point to Plane ICP* ist eine Erweiterung des ICP Algorithmus
    - vergleicht korrespondierende Punkte in einer Wolke zu Ebenen in der anderen
    - Ebenen wird durch Punkt und Normalenvektor definiert

    #v(1cm)
    $T arrow.l arg min_T {sum_i ((T dot.op b_i - m_i) * bold(n_i))^2}$

    #colbreak()
    #figure(
      caption: [Point-to-Plane-ICP @point-to-plane-icp],
      [
        #image("./assets/point-to-plane.png", width: 80%)
      ],
    )
  ]
]

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

  #v(1cm)
]

#slide[

  = Theorie - GICP-Algorithmus

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
  == Variationen für Kovarianzmatrizen

  #pseudocode-list(line-numbering: none)[
    + $C_i^A arrow.l$ `computeCovarianceMatrix`$(T dot.op b_i)$
    + $C_i^B arrow.l$ `computeCovarianceMatrix`$(m_i)$
  ]

  - für *Standard-ICP* (point-to-point):
    - $C_i^A arrow.l 0$
    - $C_i^B arrow.l 1$ #h(3.3em) → keine Oberflächenstruktur berücksichtigt (einfache Gewichtung)

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
      caption: [Plane-to-plane @segal-gicp],
      [
        #image("./assets/plane-to-plane.png", width: 100%)
      ],
    )

  ]
]

#slide[
  = Theorie - GICP-Algorithmus
  == Paper Ergebnisse @segal-gicp

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

#slide[
  = Parameterisierung

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
]

#slide[
  = Parameterisierung über YAML file
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
  = Implementierung in Ros

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
]

#slide[
  = Implementierung in Ros - Problem
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
          #image("./assets/trajectory_plot_corr_dist_old.png", width: 80%)
        ],
      )
    ],
    [
      #figure(
    caption: "Trajectory plot with lower tick speed",
    [
      #image("./assets/trajectory_plot_variance_round_1.png", width: 80%)
    ] // TODO hier vielleicht neues bild auch mit corr dist
  )
    ],
  )
  // - schlechte Ergebnisse wenn jeder Tick berücksichtigt wird
  // - deutlich bessere Ergebnisse wenn nur jeder dritte Tick berücksichtigt wird
]

#slide[
  = Implementierung in Ros

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
]

#slide[
  = 3 unterschiedliche Maps

  - jeweils Bild reinmachen
  - roboter war immer der gleiche


]

#slide[
  = Versuchsaufbau

  - fünf Durchgänge mit Standardparameterisierung (ICP & GICP)
  - drei unterschiedliche Maps
  - unterschiedliche Parameterisierung
]

#slide[
  = Turtlebot3 World

  #figure(
    caption: "Screenshot Gazebo",
    [
      #image("./assets/turtlebot3_world.jpg", width: 90%)
    ],
  )
]

#slide[
  = Turtlebot3 ICP World

  #figure(
    caption: "Screenshot Gazebo",
    [
      #image("./assets/turtlebot3_icp_world.jpg", width: 90%)
    ],
  )
]

#slide[
  = Turtlebot3 World

  #figure(
    caption: "Screenshot Gazebo",
    [
      #image("./assets/turtlebot3_dqn_stage1.jpg", width: 90%)
    ],
  )
]


#slide[
  = (Bild-)Quellen

  #bibliography(
    "sources.yml",
    style: "apa",
    title: "",
  )
]