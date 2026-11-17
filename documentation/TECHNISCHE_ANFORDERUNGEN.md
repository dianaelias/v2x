# Technische Anforderungen (rückwärts abgeleitet)

## 1. Zweck und Statusbedeutung

Dieses Dokument rekonstruiert technische Anforderungen aus dem Exposé und
gleicht sie mit dem aktuellen Repository ab. Die abgestimmten methodischen
Entscheidungen sind als Zielanforderungen festgehalten; offene technische
Details bleiben ausdrücklich als offene Punkte markiert.
Die Anforderungen beschreiben das **angestrebte Forschungssystem**. Ein
vorhandener Prototyp-Baustein ist nicht automatisch eine erfüllte Anforderung.

- **Erfüllt:** Der Kern ist im aktuellen Code vorhanden.
- **Teilweise:** Es gibt einen Prototyp, aber eine wesentliche Zielbedingung
  fehlt oder ist nur für einen eingeschränkten Testfall umgesetzt.
- **Offen:** Im Repository ist keine passende Umsetzung erkennbar.
- **Zu klären:** Die Anforderung ist im Exposé nicht präzise genug, um sie
  objektiv zu implementieren oder abzunehmen.

## 2. Systemziel und Systemgrenze

Das System soll reproduzierbare Simulationsexperimente ermöglichen, in denen
dieselbe Verkehrssituation mit drei Wahrnehmungs-/Kommunikationsbedingungen
durchlaufen wird:

- **A:** lokale Sichtlinienwahrnehmung; der Akteur wird erst nach Freigabe der
  Sichtlinie erkannt;
- **B:** ideal empfangene, aktuelle CAM-Information;
- **C:** durch konfigurierbare Kommunikationsstörungen verzögerte und/oder
  verlorene CAM-Information.

Der Planer soll regelmäßig auf Basis seines jeweils verfügbaren Umfeldzustands
neu planen. Die resultierenden Pläne sollen hinsichtlich Konsistenz,
Fahrkomfort und Sicherheitsindikatoren verglichen werden.

**Im Exposé außerhalb der primären Systemgrenze:** ein realistischer
Funknetzsimulator und ein detailliertes Fahrzeugregelungsmodell. Die
Latenz-/Verlustmodellierung soll kontrolliert in Python erfolgen; der Regler
wird als ideal angenommen.

## 3. Funktionale Anforderungen

| ID | Anforderung und prüfbares Abnahmekriterium | Quelle | Status im Repository |
|---|---|---|---|
| FR-01 | Das System liest zeitlich zuordenbare Fahrzeugspuren mit Position, Orientierung, Abmessungen und Zeitstempeln ein. Fehlende/inkonsistente Spuren führen zu einer sichtbaren Fehlermeldung. | Exposé: flexible Spur-/Kartenanbindung; bestehender Datenpfad | **Teilweise.** UrbanING-Labels werden in mehreren Skripten verarbeitet; direkte State-JSON- und Lanelet-Exporte existieren. Es gibt noch keine einheitliche datensatzneutrale Schnittstelle. |
| FR-02 | Das System unterstützt UrbanING-Realdaten und CommonRoad-Szenariodaten oder dokumentiert klar, welche Experimente welchen Eingang verwenden. Zeitbasis und Koordinatenbezug sind prüfbar. | Exposé 3.1/4 | **Teilweise.** Beide Quellen werden in getrennten Experimenten genutzt; ein gemeinsamer, validierter Datenadapter fehlt. UrbanING-Lanelets sind nicht in ein SUMO-Netz konvertiert. |
| FR-03 | Aus einem Fahrzeugzustand wird ein synthetisches Python-Dict/JSON-CAM-Modell nach ETSI CAM Release 2 mit nachvollziehbarer Feldzuordnung, Einheiten, `stationId`, `generationDeltaTime`, Position an der Frontmitte, Geschwindigkeit, Heading und Beschleunigung erzeugt. Eine binäre UPER-Kodierung ist für diese Planeruntersuchung nicht erforderlich. | Abgestimmte Entwurfsentscheidung; Exposé 3.1 | **Teilweise.** `urbaning_cam.py` erzeugt lesbare synthetische JSON-Records und transformiert die Referenzposition. Vorhandensein und korrekte Werte der geforderten Felder sind noch systematisch zu testen. Das Modell ist ausdrücklich keine binäre ETSI-Funknachricht. |
| FR-04 | Der CAM-Sender erzeugt Datensätze entsprechend dem festgelegten Sendetakt. Die zeitliche Zuordnung zwischen Erzeugung, Empfang und Simulationszeit ist für jeden Datensatz nachvollziehbar. | Exposé: periodische CAM; 4 | **Teilweise.** Im UrbanING-Label-Experiment entsteht ein Datensatz pro 10-Hz-Trackpunkt. CAM-Sendeplanung und einstellbarer CAM-Takt sind nicht als eigener Mechanismus implementiert. |
| FR-05 | Die Queue hält CAMs gemäß konfiguriertem Verzögerungsmodell zurück. Die erste Reihe verwendet feste Latenzen von 50, 100, 200, 300 und 500 ms; jede Zustellung und jeder Drop ist nachvollziehbar. Eine zweite Reihe ergänzt Jitter mit Normalverteilung (vorläufig `μ = konfigurierte Latenz`, `σ = 20 ms`). | Abgestimmte Entwurfsentscheidung | **Teilweise.** `v2x_experiment.py` unterstützt feste Verzögerung und reproduzierbaren unabhängigen Paketverlust. Auch der CommonRoad-Runner kann C pro CAM reproduzierbar verlustbehaftet betreiben. Der 20-%-Smoke-Test ist erfolgt. Gestufte Matrixläufe, Jitter und ein vollständiges Kommunikationsereignisprotokoll fehlen. CommonRoad quantisiert Verzögerungen auf Szenarioschritte; 50 ms sind bei `dt=100 ms` nicht auflösbar. |
| FR-06 | Die LDM speichert empfangene CAM-Zustände samt Erzeugungs- und Empfangszeit und berechnet AoI. Bei Verzögerung oder Verlust bleibt ein gültiger Zustand erhalten oder wird nach einer noch festzulegenden Regel extrapoliert; ein Timeout (Vorschlag: 1 s) beendet die Verwendbarkeit. | Abgestimmte Entwurfsentscheidung; Exposé 2.2/3.1 | **Teilweise.** Im CommonRoad-Runner bleibt der letzte zugestellte CAM bei Verlust erhalten und wird mit konstanter Geschwindigkeit und Richtung extrapoliert. AoI wird ausgewiesen; ein Timeout sowie ein standardisiertes Kommunikationsereignisprotokoll fehlen. |
| FR-07 | In A bleibt der andere Verkehrsteilnehmer in der Ground Truth vorhanden. Das Ego erhält ihn über die lokale Sensorik erst ab dem Zeitpunkt, an dem die physische Sichtlinie an einer Gebäude-/Hausecke frei ist. B/C erhalten kooperative Information bereits vor diesem Sichtkontakt. Alle übrigen Versuchsbedingungen bleiben gleich. | Abgestimmte Entwurfsentscheidung | **Teilweise.** A lässt den Akteur zunächst aus der Planner-Szene und schaltet ihn ab `d_sight=15 m` zum abgeleiteten Konfliktpunkt hinzu; die Freigabe bleibt aktiv. Das ist die ausdrücklich gewählte geometrische Näherung. Die CommonRoad-XMLs enthalten keine Gebäudeflächen, daher wird keine echte Sichtlinie/Verdeckung modelliert; `TTC < 1,5 s` wurde nicht als Alternative gewählt. |
| FR-08 | Der CommonRoad Reactive Planner plant bei `dt=100 ms` an jedem Simulationsschritt neu (10 Hz); Eingabe, Ergebnis und Fehlschlag jedes Zyklus werden protokolliert. | Abgestimmte Entwurfsentscheidung; Exposé 3.1 | **Teilweise.** Der CommonRoad-T-Kreuzungslauf wird auf Replanning alle 100 ms umgestellt. Die vollständige Kopplung an eine laufende Queue/LDM sowie der zyklische Planer im UrbanING-Pfad fehlen. Vorhandene Ergebnisdateien stammen aus früheren Einstellungen und müssen für neue Aussagen neu erzeugt werden. |
| FR-09 | CommonRoad Reactive Planner 2025.1 ist der festgelegte Werling-Frenét-Planer. Die Arbeit dokumentiert die verwendete Version, Referenzlinie, Frenét-Koordinaten und relevanten Planerparameter. | Abgestimmte Entwurfsentscheidung | **Teilweise.** Die Planner-Version ist gepinnt und wird verwendet. Die konkrete Frenét-Abbildung und die verwendeten Planerparameter müssen in der Methodik belegt und im Experiment nachvollziehbar festgehalten werden. |
| FR-10 | UrbanING-V2X liefert die primäre realitätsnahe Grundlage mit Fahrzeugspuren und Lanelet2-Karte. CommonRoad-Szenarien ergänzen die Untersuchung, wenn zusätzliche kontrollierte Konflikte benötigt werden. Szenarien werden nach `TTC < 3 s` und NLoS-Kreuzungsgeometrie gescreent; Konflikte dürfen reproduzierbar, etwa durch Startzeitverschiebung, zugespitzt werden. | Abgestimmte Entwurfsentscheidung; Exposé 3.2 | **Teilweise.** Ein Szenario (`ZAM_Tjunction-1_533_T-1.xml`) wurde importiert und mit einem reproduzierbaren Zeitversatz von -3,6 s getestet. Automatisches TTC-/NLoS-Screening und Auswahl eines begründeten Pools von fünf Szenarien fehlen; der erste Test ist kein Nachweis realistischer NLoS-Geometrie. |
| FR-11 | Aufeinanderfolgende Planungen werden an gleichen absoluten Zukunftszeitpunkten verglichen. Die longitudinale und laterale Abweichung `Δs` und `Δd` werden in Frenét-Koordinaten ermittelt; Mittelwert und Maximum werden je Lauf ausgewiesen. Eine daraus gebildete euklidische Gesamtabweichung ist separat zu benennen. | Abgestimmte Metrikdefinition; Exposé 3.3 | **Offen.** Vorhanden sind nur paarweise euklidische A/B/C-Positionsunterschiede; keine aufeinanderfolgenden Planungen oder `Δs`/`Δd`. |
| FR-12 | Ruck wird aus longitudinaler und lateraler Trajektorienableitung beurteilt. Neben einer Jerk-Größe in `m/s³` kann der integrierte quadratische Ruck `∫(s'''² + d'''²)dt` als Planungs-/Komfortmaß ausgegeben werden; dessen Einheit ist `m²/s⁵`, nicht `m/s³`. Aggregation und Ableitungsverfahren sind zu dokumentieren. | Abgestimmte Metrikdefinition; Exposé: Ruck/Jerk | **Teilweise.** Der Einzel-Worker bildet eine diskrete Beschleunigungsdifferenz und berichtet Peak-Jerk. Die systematische longitudinale/laterale Auswertung über die geschlossene Replanning-Folge fehlt. |
| FR-13 | Sicherheitsauswertung umfasst den minimalen 2D-Footprint-Abstand und TTC am definierten Konfliktpunkt/-bereich. TTC-Formel, Bezugszeitpunkt, Aggregation und Fälle ohne positive Kollisionsannäherung sind festzulegen. | Abgestimmte Metrikdefinition; Exposé 3.3 | **Teilweise.** Footprint-Abstände sind implementiert; eine TTC-Metrik und deren fachliche Definition fehlen. Abstände sind keine vollständige Sicherheitsgarantie. |
| FR-14 | Alle Runs speichern Eingabeparameter, Szenario-/Akteur-IDs, Zufallsseed, Software-/Planner-Version, Status, Kennzahlen und maschinenlesbare Rohdaten. Ein Run ist mit derselben Konfiguration reproduzierbar. | Exposé: reproduzierbare Experimente; 3.4 | **Teilweise.** Ergebnisse gehen nach CSV/JSON, und Paketverlust nutzt Seeds. Ein zentrales Run-Manifest mit allen Versionen/Parametern und automatisierter Wiederholung fehlt. |
| FR-15 | Die Ausgabe kennzeichnet fehlende CAMs, nicht planbare Fälle und unvollständige Versuche explizit, statt sie als erfolgreiche oder sichere Ergebnisse zu behandeln. | Wissenschaftliche Nachvollziehbarkeit; notwendige Abnahmebedingung | **Teilweise.** Der Worker kennt `no_feasible_trajectory`; die anderen Pfade protokollieren unterschiedliche Fehler-/Abbruchzustände nicht einheitlich. |

## 4. Qualitätsanforderungen

| ID | Qualitätsanforderung | Prüfmöglichkeit | Status |
|---|---|---|---|
| QR-01 | Wiederholbarkeit: identische Eingaben, Konfiguration und Seed ergeben identische Kommunikationsereignisse und dieselben Ergebnisse (abgesehen von dokumentierter Hardware-/Zeitmessstreuung). | Zwei gleiche Läufe vergleichen; Zeitmesswerte getrennt beurteilen. | **Teilweise** |
| QR-02 | Nachvollziehbarkeit: jede Messgröße ist mit Formel, Einheit, Zeitbezug und Datenquelle dokumentiert. | Kennzahl stichprobenartig bis zum Quelldatensatz zurückverfolgen. | **Teilweise** |
| QR-03 | Trennung von Planner-Eingabe und Ground Truth: zukünftige Ground Truth darf nicht als Planner-Eingabe in die CAM-Variante gelangen. | Eingabe-JSON und verwendete Objekte/Aufrufe inspizieren. | **Teilweise umgesetzt** in den zwei Planner-Pfaden; für weitere Adapter als Invariante zu testen. |
| QR-04 | Fehlerhafte Zeitreihen, unbekannte Fahrzeugtypen, fehlende Akteure oder ungültige Parameter erzeugen eine verständliche Fehlermeldung. | Negativtests für jeden Eingabeadapter. | **Teilweise** |
| QR-05 | Umgebungsreproduzierbarkeit: Host- und Planner-Abhängigkeiten sowie unterstützte Plattformen sind festgehalten. | Neue Umgebung gemäß Anleitung aufsetzen und Smoke-Test ausführen. | **Teilweise.** Planner-Image gepinnt; Host-Abhängigkeiten/Versionen nicht vollständig manifestiert. |
| QR-06 | Wartbarkeit: Module haben klare Zuständigkeiten und öffentliche Funktionen/Schnittstellen sind dokumentiert. | Code-Review und gezielte Tests je Modul. | **Teilweise.** Skripte sind bereits getrennt; gemeinsame Schnittstellen und automatisierte Tests fehlen. |
| QR-07 | Datenlizenz und Zitation des UrbanING-Datensatzes bleiben bei Veröffentlichung und Ergebnisweitergabe beachtet. | Lizenz und Zitationsangabe in Arbeit/README dokumentieren. | **Zu beachten.** Kein technisches Verhalten im Code. |

## 5. Abgestimmte Versuchsmatrix

Die erste, deterministische Hauptreihe verwendet konstante Latenz. Die
Paketverlustrate wird pro erzeugter CAM-Nachricht mit einem festen Seed
reproduzierbar realisiert. Geplant sind fünf Konfliktszenarien und fünf
Zufalls-Seeds je Parameterkombination.

| Faktor | Festgelegte Werte | Anzahl |
|---|---|---:|
| CAM-Latenz | 50, 100, 200, 300, 500 ms | 5 |
| Paketverlust | 0 %, 5 %, 10 %, 20 % | 4 |
| Konfliktszenarien | 5 nach TTC-/NLoS-Screening ausgewählte Szenarien | 5 |
| Zufalls-Seeds | 5 je Kombination | 5 |
| **Gestörte Variante C** | **5 × 4 × 5 × 5** | **500 Läufe** |

Die Zahl **500** bezeichnet ausschließlich die vollständigen C-Läufe dieser
Faktormatrix. A (lokale Sichtlinienwahrnehmung) und B (ideale V2X-Übertragung)
sind notwendige Vergleichsbedingungen und werden zusätzlich pro Szenario
ausgeführt; ihre Laufzahl ist nicht in den 500 enthalten. Der genaue Umfang
und die Wiederverwendung dieser Referenzläufe werden im Versuchsprotokoll
festgehalten.

Eine zweite Reihe ergänzt Jitter. Als vorläufiges Modell gilt eine
Normalverteilung mit `μ = jeweilige Latenzstufe` und `σ = 20 ms`. Negative
gezogene Verzögerungen müssen ausgeschlossen oder begrenzt werden; die genaue
Trunkierung sowie der Umfang dieser zusätzlichen Reihe sind vor ihrer
Ausführung festzulegen. Jitter-Läufe sind nicht Teil der oben genannten 500.

### Weitere methodische Festlegungen

- **Daten und Szenarien:** UrbanING-V2X ist die primäre realitätsnahe
  Datengrundlage. Native CommonRoad-Szenarien ergänzen sie für kontrollierte
  Konflikte. Falls reale Spuren zu ruhig sind, dürfen Konflikte durch
  reproduzierbare Startzeitverschiebung zugespitzt werden. Die fünf konkreten
  Szenarien und ein möglicher Mix der Datenquellen sind noch zu benennen.
- **A/B/C:** A modelliert lokale optische Sensorik und macht den Akteur erst
  sichtbar, wenn die physische Sichtlinie an der Gebäudeecke frei ist. B/C
  erhalten CAM-Information bereits vor diesem Zeitpunkt. In allen Varianten
  bleibt der Akteur in der Ground Truth vorhanden.
- **CAM:** Ein synthetisches Python-Dict/JSON nach ETSI-CAM-Feldsemantik reicht
  für die Planeruntersuchung. Binäre ASN.1-UPER-Kodierung ist nicht vorgesehen.
- **Replanning:** CommonRoad Reactive Planner wird alle 100 ms (10 Hz) erneut
  aufgerufen.
- **Pfadkonsistenz:** Aufeinanderfolgende Pläne werden für dieselben absoluten
  Zukunftszeitpunkte verglichen. `Δs` und `Δd` werden in Frenét-Koordinaten
  separat ausgewertet; Mittelwert und Maximum werden berichtet.
- **Ruck:** Jerk hat die Einheit `m/s³`. Das Integral des quadrierten Rucks
  hat eine andere Einheit (`m²/s⁵`) und wird als separates Komfort-/Kostenmaß
  berichtet, nicht als Jerk in `m/s³`.
- **Sicherheit:** Minimaler 2D-Footprint-Abstand und TTC am Konfliktpunkt bzw.
  Konfliktbereich werden erfasst. Eine genaue TTC-Definition ist noch zu
  spezifizieren.

## 6. Noch offene Implementierungs- und Definitionspunkte

Die fachlichen Grundentscheidungen zu Planer, Datenquellen, A/B/C, CAM-Format,
Latenzstufen und Hauptmatrix sind getroffen. Für eine belastbare Umsetzung
bleiben folgende konkrete Punkte offen:

1. **Fünf Szenarien:** Vier weitere Kandidaten ergänzen. Für alle fünf
   Sequenz-/Szenario-IDs, Akteure, Geometrie, TTC und NLoS-Nachweis
   dokumentieren; den bereits getesteten Versatz von -3,6 s nicht ungeprüft
   als endgültigen Forschungsparameter übernehmen.
2. **Sichtlinie in A:** Die vorläufige `15 m`-Sichtschwelle zum geometrisch
   abgeleiteten Konfliktpunkt ist implementiert und dokumentiert. Sie ersetzt
   keine echte Sichtlinie. Für eine physikalische Aussage wären passende
   Gebäude-/Sichtblocker-Flächen und ein Ray-casting-/Verdeckungsmodell nötig;
   alternativ ist die Näherung als bewusst vereinfachte Forschungsannahme
   beizubehalten und zu begründen.
3. **Zeitauflösung:** Die 50-ms-Latenz muss im Kommunikationsmodell darstellbar
   sein. Der CommonRoad-Test mit 100-ms-Szenariozeitschritt rundet Latenz
   derzeit auf ganze Simulationsschritte.
4. **LDM-Regel:** Zwischen Sample-and-Hold und linearer Extrapolation wählen,
   Timeout (Vorschlag: 1 s) und Zustand nach Timeout definieren.
5. **Verlustprozess:** Unabhängiger Drop pro Nachricht oder ein Burst-Modell
   festlegen. Die fünf Seeds steuern die Zufallsfolge; bei 0 % Verlust sind
   Seeds erwartungsgemäß identisch.
6. **Metrikimplementierung:** Frenét-Referenz und zeitliche Zuordnung für
   `Δs`/`Δd`, numerische Ableitung des Rucks sowie TTC-Formel, Konfliktzone und
   Fälle ohne Kollisionsannäherung präzisieren.
7. **Auswertung:** Umgang mit fehlgeschlagenen Planungen, unvollständigen Runs,
   Aggregation über fünf Szenarien/Seeds und statistischer Unsicherheit
   festlegen. 500 Läufe bilden das Versuchsdesign ab, garantieren allein aber
   keine statistische Belastbarkeit oder identifizierbare Schwellenwerte.
8. **Jitter-Zusatzreihe:** Trunkierung negativer Latenzen, Seeds und gewünschte
   Zahl zusätzlicher Läufe festlegen.
9. **Sicherheitsrand/Planner-Interaktion:** Den dynamischen Hindernisrand der
   CommonRoad-Konfiguration (aktuell 0 m) fachlich begründen, sinnvoll
   variieren und dokumentieren. Szenarien müssen die Prognose in den vom
   Planner tatsächlich geprüften Konfliktbereich bringen. Der Rand darf nicht
   allein zur Erzeugung sichtbarer A/B/C-Unterschiede gewählt werden.
10. **Reproduzierbarkeit:** Host-Abhängigkeiten, Run-Konfigurationen,
   Planner-Version, Seeds und Rohdaten vollständig archivieren.

## 7. Vorgeschlagene Abnahmefolge

1. Datenadapter: bekannte Zeitstempel, Fahrzeugmaße und Koordinaten mit
   kleinen Referenzfällen prüfen.
2. CAM-Felder: jede Ableitung gegen die Quellspur prüfen; bei Anspruch auf
   Standardkonformität zusätzlich ASN.1-/UPER-Roundtrip und ETSI-Grenzwerte
   testen.
3. Kommunikationsmodell: Nullverzögerung, definierte Verzögerung,
   0-%-/100-%-Verlust, gleiche Empfangszeit und Nachrichtenreihenfolge testen.
4. LDM/AoI: für einen handgerechneten Zeitverlauf den zuletzt gültigen Zustand
   und AoI vergleichen.
5. Planner-Adapter: mit und ohne ausgewähltes Hindernis sowie mit identischen
   Eingaben wiederholen; Ground Truth muss vom Planner getrennt bleiben.
6. Metriken: geometrische Grenzfälle und numerische Ableitungen mit bekannten
   Trajektorien validieren.
7. Experiment: vollständige Versuchsmatrix ausführen, alle Run-Konfigurationen
   archivieren und Fehler/ausgeschlossene Läufe begründen.
