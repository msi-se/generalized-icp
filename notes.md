### Thema 1.3 G-ICP Scan-Matching
Ein weiteres Verfahren zur Punktwolkenregistrierung in der PCL-Bibliothek ist des G-ICP Verfahren.
Dieses Verfahren verwendet nicht nur ein Punkt-zu-Punkt Matching, sondern versucht lokale Formen/Strukturen zu matchen.
Implementieren Sie unter Verwendung der PCL Bibliothek einen weiteren Lidor-Odometry Node gicp-lio Node und vergleichen sie dessen Performance mit der des ICP Scan-Matchers aus den Übungen.

- Grundlagen G-ICP: https://www.roboticsproceedings.org/rss05/p21.pdf

- PCL G-ICP API: https://pointclouds.org/documentation/classpcl_1_1_generalized_iterative_closest_point.html

Alternative GICP Implementierungen:
- https://github.com/koide3/ndt_omp (enthält auch eine G-ICP Implementierung)
- https://github.com/SMRT-AIST/fast_gicp


#### Youtube

[cyrill](https://www.youtube.com/watch?v=2hC9IG6MFD0&ab_channel=CyrillStachniss)

### Vorgehen

- Presentation Erstellen
  - Einführung
  - Theorie
  - Implementierung
  - Vergleich mit normalem ICP (Versuch)
  - Fazit
- Implementierung