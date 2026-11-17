# Technische Dokumentation: UrbanING-V2X-Prototyp

## 1. Zweck und Quellen

Dieses Dokument erklärt den aktuellen Stand des Repositorys und die Aufgaben der
einzelnen Programme. Es beschreibt den Code, der tatsächlich vorhanden ist, nicht
den später angestrebten vollständigen Forschungsaufbau.

Die fachliche Zielsetzung stammt aus dem Exposé
[`expose_DianaElias_V2X.pdf`](./expose_DianaElias_V2X.pdf) und der dazugehörigen
Präsentation [`Expose_Präsentation_DianaElias.pptx`](./Expose_Präsentation_DianaElias.pptx).
Der bisherige Arbeits- und Versuchstand ist in
[`PROJEKTSTAND.md`](./PROJEKTSTAND.md) festgehalten. Die daraus rückwärts
abgeleiteten, prüfbaren Anforderungen stehen in
[`TECHNISCHE_ANFORDERUNGEN.md`](./TECHNISCHE_ANFORDERUNGEN.md).

**Kurz gesagt:** Das Repository ist ein Forschungsprototyp mit mehreren
aufeinander aufbauenden, aber noch getrennten Experimenten. Es generiert
synthetische CAM-ähnliche Datensätze, simuliert Verzögerung und Paketverlust,
und bindet den CommonRoad Reactive Planner in zwei verschiedenen Testpfaden an.
Noch gibt es keine durchgehende Pipeline, die alle Komponenten mit echter
ETSI-Kodierung, UrbanING-Karte und den vollständigen Stabilitätsmetriken
verbindet.

## 2. Die Forschungsfrage in Alltagssprache

Ein Fahrzeug kann über V2X eine Cooperative Awareness Message (CAM) mit der
Position und Bewegung eines anderen Fahrzeugs erhalten. Ist die Nachricht
verzögert, passt die gemeldete Position nicht mehr genau zur aktuellen Position.
Die Arbeit soll untersuchen, ob der reaktive Planer deswegen zwischen seinen
aufeinanderfolgenden Planungen stärker schwankt, unangenehme Bewegungen erzeugt
oder Sicherheitsabstände unterschreitet.

Die beabsichtigte Vergleichslogik lautet:

1. **A – lokale Sichtlinienwahrnehmung:** Der andere Verkehrsteilnehmer bleibt
   vorhanden, wird vom Ego-Fahrzeug aber erst erkannt, wenn die physische
   Sichtlinie an einer Gebäude-/Hausecke frei ist.
2. **B – ideale Kommunikation:** aktuelle CAM kommt ohne Verzögerung und ohne
   Verlust an.
3. **C – gestörte Kommunikation:** CAM kommt verzögert und/oder geht verloren.

Für einen fairen Vergleich müssen Szenario, Fahrzeugzustand, Planerparameter und
Auswertung zwischen A, B und C gleich bleiben. Nur die kooperative Information
und die festgelegte Kommunikationsstörung sollen sich unterscheiden.

## 3. Überblick über den aktuellen Projektaufbau

### 3.1 Datenpfad A: UrbanING-Daten und Kommunikations-/LDM-Experiment

```text
UrbanING-Labels + Track-ID-Zuordnung
                 |
                 v
urbaning_cam.py: Trackpunkte -> synthetische CAM-ähnliche JSON-Datensätze
                 |
                 v
v2x_experiment.py: A/B/C, Verzögerungsqueue, Paketverlust
                 |
                 v
Minimaler Empfängerzustand (LDM) -> AoI und Positionsfehler -> CSV/JSON
```

Dieser Pfad prüft die Nachrichtenerzeugung und die Auswirkungen der
Kommunikationsparameter auf die am Empfänger verfügbare Information. Er ruft
keinen Planer auf. Die experimentierten Daten belegen daher noch keine
Planungsstabilität oder Fahrzeugsicherheit.

### 3.2 Datenpfad B: Einzelne UrbanING-Planung

```text
UrbanING-Labels
     |
     v
run_commonroad_planner.py: Zustand zu einem ausgewählten Zeitpunkt
     |                     und A/B/C-CAM-Schnappschüsse vorbereiten
     v
Docker / Python 3.11
     |
     v
commonroad_planner_worker.py: synthetischer Korridor -> je eine Planung A/B/C
     |
     v
Planung, CAM-Prognose und Ground-Truth-Abstände -> JSON
```

Das ist ein Adapter- und Funktionstest: Für einen ausgewählten Zeitpunkt wird
je Variante eine Planung berechnet. Der Straßenkorridor ist künstlich aus der
aufgezeichneten Ego-Route erzeugt. Das vorausgesagte Senderfahrzeug bewegt sich
mit konstanter Geschwindigkeit aus dem empfangenen CAM-Zustand. Die künftige
Sender-Ground-Truth wird separat gehalten und nur nach der Planung zur
Auswertung verwendet.

### 3.3 Datenpfad C: CommonRoad-T-Kreuzung mit zyklischer Neuplanung

```text
CommonRoad-XML + Planner-YAML
     |
     v
commonroad_scenario_experiment.py
     |
     +-- A: Akteur nach lokaler Sichtfreigabe in Planner-Szene aufnehmen
     +-- B: ausgewähltes Hindernis mit aktuellem Zustand ersetzen
     +-- C: CAM-Queue mit Latenz, optionalem Paketverlust und LDM
     |
     v
CommonRoad Reactive Planner: bei 100 ms Szenarioschritt neu planen
     |
     v
Abstand zur tatsächlichen Hindernisbahn + A/B/C-Vergleich -> JSON
```

Dieser Pfad benutzt die Karte und Verkehrsteilnehmer der mitgelieferten
CommonRoad-Benchmarkszene. Der Szenariozeitschritt beträgt 0,1 s. Der Code ist
auf Neuplanung bei jedem Schritt (100 ms, 10 Hz) umgestellt. C kann pro
erzeugtem CAM einen reproduzierbaren Paketverlust simulieren. Ein lokales
Sensorbild für A gibt es nicht. Stattdessen nutzt A die abgestimmte
Sichtschwellen-Näherung: Der Konfliktpunkt wird als erste räumliche
Schnittstelle der Ego-Referenzlinie mit der aufgezeichneten Akteur-Mittellinie
bestimmt. Der Akteur wird lokal sichtbar, sobald sein Mittelpunkt höchstens
`d_sight = 15 m` von diesem Konfliktpunkt entfernt ist. Die Sichtbarkeit bleibt
ab der ersten Freigabe aktiv. Bis dahin bleibt der Akteur in der Ground Truth,
wird aber aus der Planner-Szene herausgehalten; danach wird sein lokal
gemessener Zustand mit Konstantgeschwindigkeits-Prädiktion eingebunden. Die
CommonRoad-Szenarien enthalten keine statischen Gebäudegeometrien. Die
Näherung modelliert daher keine echte Sichtlinie oder Gebäudeverdeckung. Die
alternative Schwelle `TTC < 1,5 s` wird nicht verwendet.
Die mitgelieferte YAML-Datei setzt den dynamischen Sicherheitsrand auf 0 m;
ohne eine passende physische Konfliktgeometrie erzwingt der Test daher keine
Mindestdistanz. Ein positiver Rand muss fachlich begründet und als
Versuchsparameter dokumentiert werden, nicht nur so gewählt werden, dass sich
A/B/C sichtbar unterscheiden.

#### Importiertes Szenario und Konflikt-Tuning-Test

Für einen ersten kontrollierten T-Kreuzungsversuch wurde
`ZAM_Tjunction-1_533_T-1.xml` aus dem lokalen Repository
`commonroad-scenarios/scenarios/recorded/hand-crafted/` nach
`commonroad_scenarios/imported/` kopiert. Die Quelldatei ist unverändert
(SHA-256 `5ec8ef3f6094962e89e1a84f9acaf706911278285158b90286ef4056e3904ad0`).
Die zugehörige `LICENSE.txt` liegt im Importordner; die Quelle nennt die
CommonRoad-Benchmark-Referenz von Althoff, Koschi und Manzinger (2017).

Der Runner unterstützt `--actor-time-offset-ms`. Ein negativer Wert startet
den ausgewählten Akteur relativ zur Ego-Trajektorie an einem späteren Punkt
seiner aufgezeichneten Spur; ein positiver Wert verzögert den Akteur. Der Wert
muss auf dem 100-ms-Szenariozeitschritt liegen. CAM-Erzeugungszeit und
Simulationszeit werden getrennt behandelt, sodass die CAM-Alterung weiterhin
der eingestellten Latenz entspricht.

Der 200-ms-Lauf lässt sich aus dem Projektstamm so wiederholen:

```zsh
docker run --rm --platform=linux/amd64 \
  --entrypoint python \
  -v "$PWD:/workspace" -w /workspace \
  urbaning-commonroad-planner:2025.1 \
  commonroad_scenario_experiment.py \
  --scenario /workspace/commonroad_scenarios/imported/ZAM_Tjunction-1_533_T-1.xml \
  --configuration /workspace/commonroad_scenarios/ZAM_Tjunction-1_42_T-1.yaml \
  --planning-problem-id 60000 --obstacle-id 1 --sight-distance-m 15 \
  --delay-ms 200 --actor-time-offset-ms -3600 --simulation-steps 147 \
  --output /workspace/sumo_input/commonroad_tjunction_533_actor1_advance_3600ms.json
```

Paketverlust wird mit `--packet-loss` als Wahrscheinlichkeit pro erzeugtem CAM
gesetzt; `--seed` macht die Drop-Entscheidungen reproduzierbar. Die LDM hält
den letzten zugestellten Zustand und extrapoliert ihn mit konstanter
Geschwindigkeit und Richtung. Beispiel für 20 % Verlust:

```zsh
docker run --rm --platform=linux/amd64 \
  --entrypoint python \
  -v "$PWD:/workspace" -w /workspace \
  urbaning-commonroad-planner:2025.1 \
  commonroad_scenario_experiment.py \
  --scenario /workspace/commonroad_scenarios/imported/ZAM_Tjunction-1_533_T-1.xml \
  --configuration /workspace/commonroad_scenarios/ZAM_Tjunction-1_42_T-1.yaml \
  --planning-problem-id 60000 --obstacle-id 1 --sight-distance-m 15 \
  --delay-ms 200 --actor-time-offset-ms -3600 \
  --packet-loss 0.2 --seed 7 --simulation-steps 147 \
  --output /workspace/sumo_input/commonroad_tjunction_533_actor1_advance_3600ms_loss20_seed7.json
```

Mit `--obstacle-id 1 --actor-time-offset-ms -3600` wurde die Spur des Akteurs
um 3,6 s vorgezogen. Das ist ein reproduzierbarer, synthetischer
Konflikt-Tuning-Test; es ändert weder die Fahrzeugmaße noch den
Sicherheitsrand oder die aufgezeichnete Bewegungsform. Der aus den
Mittellinien abgeleitete Konfliktpunkt ist `(16,73 m; 2,74 m)`. Bei
`d_sight = 15 m` wird der Akteur im zeitverschobenen Lauf erstmals bei 4,7 s
lokal sichtbar; im unverschobenen Lauf bei 8,3 s. Die Freigabe ist danach
verriegelt (der Akteur bleibt lokal bekannt). Die vollständigen Läufe
mit 147 Planungsschritten und 100-ms-Replanning liegen unter
`sumo_input/commonroad_tjunction_533_actor1_advance_3600ms.json` (200 ms CAM-
Alter) und `sumo_input/commonroad_tjunction_533_actor1_advance_3600ms_delay500ms.json`
(500 ms). Ein unverschobener Vergleichslauf liegt unter
`sumo_input/commonroad_tjunction_533_actor1_baseline.json`.

Im 200-ms-Lauf überlappten weder A noch B/C mit der ausgewerteten
Ground-Truth-Spur; der minimale Abstand betrug 16,6 mm für A/B und 8,7 mm für
C. A und B erzeugten in diesem Szenario dieselbe Trajektorie, weil A schon
4,7 s vor dem Konfliktpunkt lokal sichtbar wird. Die mittlere/maximale
B/C-Abweichung betrug 3,4/12,1 cm. Bei 500 ms betrug sie 41,8 cm im Mittel und
1,17 m maximal. B und C hatten auch dort keine diskrete Überlappung (Minimum
rund 9,6 mm). Millimeter-Abstände sind kein Sicherheitsnachweis.

**Paketverlusttest mit 20 %:** Der Runner wärmt die CAM-Queue mit den
Nachrichten auf, die vor Simulationszeit 0 bereits unterwegs wären. Deshalb
zählen pro Lauf zwei Warm-up-CAMs zu den 114 erzeugten Nachrichten. Bei Seed 7
wurden 26 CAMs verworfen (22,8 % realisierter Verlust), 88 zugestellt; bei
Seed 8 waren es 28 Drops (24,6 %) und 86 Zustellungen. Ein 0-%-Verlust-
Kontrolllauf mit Seed 7 ist unter
`sumo_input/commonroad_tjunction_533_actor1_advance_3600ms_loss0_seed7.json`
gespeichert.

Alle drei Varianten planten jeweils 147 Schritte. Bei Seed 7 änderte der
Paketverlust die B/C-Abweichung gegenüber dem 0-%-Kontrolllauf: der
Positionsunterschied stieg im Mittel von 3,4 cm auf 6,7 cm, maximal von
12,1 cm auf 24,9 cm. C erreichte 9,6 mm Mindestabstand gegenüber 16,6 mm für
B; diskrete Überlappungen gab es in B/C nicht. Bei Seed 8 waren die C-
Trajektorie und die B/C-Abweichung trotz 20 % konfiguriertem Paketverlust
identisch zum 0-%-Lauf. Das zeigt, dass zufällige Verluste nicht zwingend
einen Plannerunterschied auslösen. Die mittlere AoI von C lag bei 604 ms
(Seed 7) bzw. 606 ms (Seed 8); das Maximum war 3,5 s. Eine Wiederholung mit
Seed 7 reproduzierte Trajektorien, Drop-Zahlen und AoI exakt.

Das ist ein kontrollierter Funktionsnachweis, kein statistisch belastbares
Ergebnis: zwei Seeds ersetzen nicht die geplanten fünf Szenarien und fünf
Seeds je Matrixzelle. Die optionale Verlustwahrscheinlichkeit ist im
CommonRoad-Runner an sich pro Nachricht, aber 50-ms-Latenz ist bei
`dt=100 ms` weiterhin nicht darstellbar. Der konstante
Geschwindigkeits-/Richtungsschätzer ist nicht gleichbedeutend mit einer
realistischen LDM-Unsicherheits- oder Timeout-Regel. Die bis zu 3,5 s
gemessene AoI macht insbesondere die noch fehlende Timeout-Entscheidung
relevant.

Diese Läufe demonstrieren nur, dass die zeitliche Konfliktlage und die
Hindernisrepräsentation den Plannerlauf beeinflussen können. Sie sind kein
Ergebnis für die Forschungsfrage: A verwendet nur eine Distanzschwelle statt
einer physischen Sichtlinie; der Runner modelliert weiterhin nicht die
geplante LDM-Timeout-Regel. Vor einer Auswertung ist außerdem zu prüfen, ob der
gewählte Versatz eine realistische Anfangssituation darstellt.

**Die drei Datenpfade sind nicht austauschbar:** Sie unterscheiden sich in
Datenquelle, Kommunikationsmodell, Karte, Planungsumfang und Auswertung.
Ergebnisse dürfen deshalb nicht ohne Weiteres zu einer gemeinsamen
Versuchsreihe zusammengefasst werden.

## 4. Dateien und ihre Aufgaben

| Datei | Aufgabe und wichtigste Eingaben/Ausgaben |
|---|---|
| [`download_urbaning.py`](../download_urbaning.py) | Kommandozeilen-Wrapper für das installierte `urbaning`-Paket. Lädt eine Sequenz oder mit `--all` den ganzen Datensatz. |
| [`urbaning_sumo_data.py`](../urbaning_sumo_data.py) | Liest Fahrzeug-State-JSON-Dateien direkt und exportiert Positions-CSV-Dateien. Liest außerdem Lanelet2-OSM, projiziert Koordinaten nach UTM32N und schreibt Lanelet-Grenzen als JSON. Es erzeugt **kein** SUMO-Netz. |
| [`urbaning_cam.py`](../urbaning_cam.py) | Berechnet Kinematik aus annotierten Tracks und erzeugt lesbare, synthetische CAM-Release-2-*ähnliche* Records als JSON Lines. Keine ASN.1-/UPER-Kodierung. |
| [`v2x_experiment.py`](../v2x_experiment.py) | Simuliert A/B/C auf gemeinsamen Annotationstimestamps. Bekommt die CAMs ohne Verzögerung; C verwendet eine feste Verzögerung und unabhängige reproduzierbare Drop-Entscheidungen. Schreibt Tickdaten und Zusammenfassungen. Kein Planer. |
| [`visualize_encounter.py`](../visualize_encounter.py) | Zeichnet zwei Fahrzeugtracks und berechnet den Abstand ihrer orientierten 2D-Rechtecke. Höhe und Dynamik werden nicht bewertet. |
| [`run_commonroad_planner.py`](../run_commonroad_planner.py) | Bereitet für einen UrbanING-Zeitpunkt die Planner-Eingabe und getrennte Ground Truth auf, baut optional das Docker-Image und startet den Einmal-Worker. |
| [`commonroad_planner_worker.py`](../commonroad_planner_worker.py) | Wandelt den Eingabefall in ein synthetisches CommonRoad-Lanelet- und Planungsproblem um, modelliert das CAM-Fahrzeug als Hindernis und berechnet je A/B/C eine Planung. |
| [`commonroad_scenario_experiment.py`](../commonroad_scenario_experiment.py) | Führt einen geschlossenen Replanning-Versuch auf einer CommonRoad-XML-Szene durch. Verwendet ideales CAM bzw. eine verzögerte, optional verlustbehaftete CAM-Queue mit Sample-and-Hold und Konstantgeschwindigkeits-Extrapolation; misst unter anderem den Ground-Truth-Footprint-Abstand. |
| [`Dockerfile.commonroad`](../Dockerfile.commonroad) | Installiert CommonRoad Reactive Planner 2025.1 in Python 3.11 und definiert den Worker als Container-Einstiegspunkt. |
| [`commonroad_scenarios/`](../commonroad_scenarios/) | Enthält die T-Kreuzungsszene und Planner-Konfiguration sowie unter `imported/` das zusätzlich getestete Benchmark-Szenario mit mitkopierter Lizenz. |
| [`sumo_input/`](../sumo_input/) | Enthält abgeleitete Eingaben und Versuchsergebnisse. Der Verzeichnisname bedeutet nicht, dass die Dateien bereits ein lauffähiges SUMO-Szenario sind. |

Die Python-Module sind ausführbare Skripte und noch kein installiertes,
versioniertes Python-Paket. Die Abhängigkeiten sind nicht durch eine
Repository-Manifestdatei wie `pyproject.toml` oder `requirements.txt`
reproduzierbar festgeschrieben. Die Planner-Abhängigkeit ist separat im
Dockerfile gepinnt.

## 5. Wichtige Daten- und Modellbegriffe

### Fahrzeugspur und Zeit

Ein annotierter Track enthält mehrere Positionen und Orientierungen mit
Zeitstempeln. In `urbaning_cam.py` werden Geschwindigkeit, Beschleunigung,
Gierrate und Krümmung numerisch aus benachbarten Messpunkten abgeleitet. Die
Ableitung nutzt an inneren Punkten eine zentrale Differenz und an den Rändern
eine einseitige Differenz. Ein Positionssprung in den Annotationen kann deshalb
zu einem großen abgeleiteten Geschwindigkeitswert führen; aktuell wird dieser
markiert, nicht korrigiert.

### CAM-Semantik und Koordinaten

Ein synthetischer Datensatz enthält PDU-Header, CAM-Parameter, Position und
Fahrzeugzustand in einer lesbaren JSON-Struktur. Die Fahrzeugposition aus den
Labels ist zunächst ein Bounding-Box-Mittelpunkt. Für die CAM-Referenzposition
wird die Position entlang der Längsachse um die halbe Fahrzeuglänge zur
Frontmitte verschoben. Die lokale Position wird über den dokumentierten
`crossing1`-GPS-Ursprung in geographische Koordinaten umgerechnet.

Die Werte sind nicht in die ETSI-ASN.1-Bereiche quantisiert und nicht als UPER
kodiert. Das JSON ist deshalb weder ein Funkpaket noch ein Beleg vollständiger
ETSI-Konformität. Die Stations-ID ist eine reproduzierbare CRC32-ID aus
Sequenzname und Track-ID, keine reale ITS-Station-ID.

### Verzögerungsqueue, Verlust und LDM

`v2x_experiment.py` legt nicht verlorene Nachrichten in einen Min-Heap. Die
Priorität ist die Empfangszeit; eine Sequenznummer macht die Reihenfolge bei
gleicher Empfangszeit stabil. Pro Tick werden alle fälligen Nachrichten
zugestellt. Die LDM ist hier ein minimales Python-Datenobjekt mit dem zuletzt
zugestellten CAM und dessen Empfangszeit, kein vollständiger ETSI-LDM-Dienst.

- Der **AoI** wird als aktuelle Simulationszeit minus Erzeugungszeit des zuletzt
  empfangenen CAM berechnet.
- Der **Positionsfehler** ist der Abstand zwischen der im CAM gespeicherten
  Frontreferenzposition und der Ground-Truth-Frontreferenzposition des Senders
  am aktuellen Tick.
- Der **Paketverlust** wird als unabhängige Zufallsentscheidung je erzeugtem
  CAM modelliert. Ein fester Seed macht die Folge für die gleichen Parameter
  reproduzierbar.
- Die aktuelle Latenz ist in diesem Harness konstant konfiguriert. Jitter oder
  eine statistische Verzögerungsverteilung werden nicht simuliert.

### Planer und Bewertung

Der Planer erhält in den UrbanING-Experimenten eine aufgezeichnete bzw.
synthetisch daraus erstellte Referenzroute. Im Worker des Einzeltests wird das
CAM-Hindernis mit konstanter Geschwindigkeit fortgeschrieben. Im
T-Kreuzungsversuch wird das ausgewählte CommonRoad-Hindernis durch eine
Konstantgeschwindigkeitsprognose ab aktuellem oder älterem Zustand ersetzt.
Das sind Modellannahmen und keine aus dem Funkkanal oder einem Bewegungsmodell
gemessenen Prädiktionen.

`visualize_encounter.py` bildet Fahrzeuge als gedrehte 2D-Rechtecke ab. Die
Überlappungsprüfung nutzt die Separating-Axis-Methode: Gibt es auf einer
Rechteck-Kantennormalen eine Lücke zwischen den Projektionen, überlappen die
Rechtecke nicht. Ohne Lücke überlappen sie; andernfalls wird der kleinste
Abstand zwischen ihren Kanten bestimmt.

## 6. Ausführen der vorhandenen Experimente

Die Beispiele setzen eine passende Projektumgebung, lokale Datensatzdateien
und für den Planer Docker voraus. Relative Pfade beziehen sich auf das
Repository-Hauptverzeichnis.

```zsh
# Eine UrbanING-Sequenz laden
.venv/bin/python download_urbaning.py --sequence-name 20241126_0017_crossing1_00

# Trajektorien und Lanelet-Grenzen exportieren
.venv/bin/python urbaning_sumo_data.py

# Synthetische CAM-ähnliche JSON-Lines erzeugen
.venv/bin/python urbaning_cam.py

# Kommunikations-/LDM-Experiment, ohne Planner
.venv/bin/python v2x_experiment.py --delay-ms 200 --packet-loss 0.1 --seed 7

# 2D-Fahrzeugabstände darstellen
.venv/bin/python visualize_encounter.py

# Einen A/B/C-Einzel-Replan im Docker-Worker ausführen
.venv/bin/python run_commonroad_planner.py --planning-time 12.0 --delay-ms 200
```

Der zyklische CommonRoad-T-Kreuzungslauf wird direkt im Planner-Container
gestartet:

```zsh
docker run --rm --platform=linux/amd64 \
  --entrypoint python \
  -v "$PWD:/workspace" -w /workspace \
  urbaning-commonroad-planner:2025.1 \
  commonroad_scenario_experiment.py \
  --scenario /workspace/commonroad_scenarios/ZAM_Tjunction-1_42_T-1.xml \
  --configuration /workspace/commonroad_scenarios/ZAM_Tjunction-1_42_T-1.yaml \
  --planning-problem-id 60000 --obstacle-id 1 \
  --delay-ms 500 --simulation-steps 147 \
  --output /workspace/sumo_input/commonroad_tjunction_actor1_500ms.json
```

## 7. Bedeutung der Messwerte und Grenzen

| Messwert im aktuellen Code | Was er aussagt | Was er nicht aussagt |
|---|---|---|
| AoI und Sender-Positionsfehler | Frische und räumliche Abweichung der zuletzt empfangenen CAM-Information im Kommunikationsharness | Keine Planerreaktion und kein Sicherheitsnachweis |
| Paarweiser A/B/C-Positionsunterschied | Geometrische Abweichung der ausgeführten Planner-Punkte zwischen Varianten | Keine Frenet-aufgelöste Aufteilung in `Δs` und `Δd`; ein einzelner Summenabstand erklärt nicht die Ursache |
| Planungs-Jerk im Einzel-Worker | Maximale Betragsänderung der im Planer ausgegebenen Beschleunigung pro Zeitschritt | Keine zeitliche Jerk-Auswertung über eine lange geschlossene Replanning-Folge |
| Footprint-Clearance | Mindestabstand orientierter 2D-Fahrzeugrechtecke an ausgewerteten Zeitpunkten | Keine TTC, keine Höhe, Unsicherheit, Bremsbarkeit oder vollständige Sicherheitsbewertung |
| CommonRoad-Clearance | Abstand des Ego-Footprints zur tatsächlichen Spur des ausgewählten Akteurs | Die Planung selbst nutzt nicht die Zukunfts-Ground-Truth; sie bewertet aber auch nicht automatisch alle realen Sicherheitsaspekte |
| Planner-Kollisionsprüfung | Verwirft Trajektorien, die den modellierten dynamischen Hindernissen geometrisch zu nahe kommen (mit dem konfigurierten Sicherheitsrand) | Garantiert keinen bestimmten Mindestabstand, wenn der konfigurierte Rand 0 m ist |

## 8. Abgestimmtes Forschungsdesign und aktueller Umsetzungsstand

Als Hauptreihe sind konstante Latenzen von 50, 100, 200, 300 und 500 ms sowie
Paketverlustraten von 0 %, 5 %, 10 % und 20 % festgelegt. Zusammen mit fünf
ausgewählten Konfliktszenarien und fünf Seeds je Kombination ergibt das für
Variante C:

> **5 Latenzstufen × 4 Verlustraten × 5 Szenarien × 5 Seeds = 500 C-Läufe**

Diese 500 Läufe zählen nur die gestörte Variante C. A (lokale
Sichtlinienwahrnehmung) und B (ideale V2X-Kommunikation) sind separate
Vergleichsbedingungen und werden zusätzlich ausgeführt. Eine zweite,
zusätzliche Versuchsreihe soll Jitter mit einem vorläufigen Normalmodell
`μ = Latenzstufe`, `σ = 20 ms` untersuchen; sie ist nicht in den 500 enthalten.
Wie negative Jitter-Ziehungen behandelt werden, muss vor der Implementierung
festgelegt werden.

UrbanING-V2X ist die primäre realitätsnahe Szenariogrundlage. Native
CommonRoad-Szenarien ergänzen sie für kontrollierte Konflikte. Falls
aufgezeichnete Spuren zu ruhig sind, kann die Startzeit reproduzierbar
verschoben werden. Fünf konkrete Szenarien müssen noch ausgewählt und mit
`TTC < 3 s` sowie NLoS-Kreuzungsgeometrie begründet werden.

CAMs sind für diese Untersuchung als synthetisches Python-Dict/JSON mit
ETSI-CAM-Feldsemantik ausreichend; UPER-Binärkodierung ist nicht vorgesehen.
Die LDM soll empfangene Zustände verwalten. Noch offen ist, ob sie bei
Kommunikationsausfall den letzten Zustand per Sample-and-Hold hält oder bis zu
einem vorgeschlagenen 1-s-Timeout linear extrapoliert.

Die methodischen Festlegungen sind nicht gleichbedeutend mit ihrer
Implementierung: `v2x_experiment.py` hat bisher nur ein festes Delay je Lauf;
der CommonRoad-Plannerlauf unterstützt inzwischen optionalen Paketverlust,
quantisiert Delay aber auf 100-ms-Szenarioschritte. Insbesondere sind 50 ms
daher noch nicht korrekt abbildbar. Die detaillierte Statuszuordnung steht in der
[Anforderungsspezifikation](./TECHNISCHE_ANFORDERUNGEN.md).

Ein gezielter Szenario-Smoke-Test zeigte außerdem: Im T-Kreuzungsbenchmark
blieben die A/B/C-Trajektorien für den untersuchten Akteur und die getesteten
Verzögerungen identisch; beim engsten geprüften Akteur lag der
Ground-Truth-Footprint-Abstand bei etwa 1,60 m. In einem Diagnosefenster um
diesen Begegnungszeitpunkt verwarf der Planner keine Kandidaten aufgrund einer
Kollision. Der Standard-Sicherheitsrand der geladenen CommonRoad-Konfiguration
beträgt 0 m. Das Ergebnis deutet deshalb auf fehlende Planner-Interaktion im
aktuellen Szenario-/Konfigurationssetup hin, nicht auf eine allgemeine
Wirkungslosigkeit von Latenz. Ein zeitlich abgestimmter Zusatztest mit dem
importierten Szenario 533 erzeugte dagegen A/B-Abweichungen und vermied in
B/C diskrete Ground-Truth-Überlappungen, aber keinen ausreichenden
Sicherheitsabstand. Die aktualisierten 0-%-Verlust-Läufe zeigten B/C-
Abweichungen von 3,4 cm im Mittel (200 ms) bzw. 41,8 cm (500 ms). Ein
20-%-Verlusttest erhöhte die Seed-7-Abweichung auf 6,7 cm im Mittel; bei
Seed 8 entstand keine zusätzliche Plannerabweichung gegenüber 0 % Verlust.
Das ist kein Sicherheitsnachweis. Details und Grenzen sind im
[Projektstand](./PROJEKTSTAND.md) vermerkt.

## 9. Bekannte offene Punkte vor belastbaren Masterarbeitsergebnissen

1. Eine klare, durchgängige Pipeline und gemeinsame Versuchsschnittstellen für
   UrbanING- und CommonRoad-Szenarien festlegen.
2. Variante A als lokale Sensorwahrnehmung mit Sichtlinien-/Sichtverdeckungs-
   modell implementieren; den Akteur nicht aus der Ground Truth entfernen.
3. Die 50-ms-Latenz trotz 100-ms-Szenariozeitschritt korrekt abbilden und die
   festgelegte 500-Läufe-Matrix inklusive Paketverlust, Seeds und Run-Manifest
   automatisieren.
4. Sample-and-Hold oder lineare Extrapolation sowie Timeout der LDM auswählen
   und umsetzen.
5. Fünf kritische Szenarien reproduzierbar auswählen, nach TTC/NLoS begründen
   und erforderliches Startzeit-Tuning dokumentieren.
6. Pfadkonsistenz in Frenet-Koordinaten, Jerk über Replanning-Schritte und eine
   klar definierte TTC-/Konfliktzonenmetrik implementieren und validieren.
7. Jitter als separate Versuchsreihe mit dokumentierter Behandlung negativer
   Latenzen implementieren.
8. Szenarien und Planner-Konfigurationen so validieren, dass die ausgewählte
   kooperative Information tatsächlich in den Prognose-/Konfliktbereich der
   Kandidatentrajektorien fällt. Sicherheitsrand fachlich festlegen und
   Sensitivitätsanalyse ausweisen, statt Werte zur Erzwingung sichtbarer
   A/B/C-Unterschiede zu tunen.
9. Fehlerfälle und Planner-Ausfälle getrennt erfassen und die statistische
   Auswertung festlegen.
10. Abhängigkeiten, Testfälle und Ausführungsanleitungen für eine unabhängige
   Reproduktion festschreiben.

## 10. Einstieg für das Lesen des Codes

Eine sinnvolle Lesereihenfolge ist:

1. Dieses Dokument und die Forschungsanforderungen lesen.
2. [`download_urbaning.py`](../download_urbaning.py) und
   [`urbaning_sumo_data.py`](../urbaning_sumo_data.py) lesen, um Datenquellen,
   Zeitstempel und Koordinaten zu verstehen.
3. [`urbaning_cam.py`](../urbaning_cam.py) lesen: zuerst Kinematik, dann
   Umrechnung in CAM-Felder, zuletzt Dateiausgabe.
4. [`v2x_experiment.py`](../v2x_experiment.py) lesen: Zeitachse, Queue,
   Zustellung, LDM/AoI und anschließend Zusammenfassung.
5. [`visualize_encounter.py`](../visualize_encounter.py) für die unabhängige
   Footprint-Geometrie lesen.
6. [`run_commonroad_planner.py`](../run_commonroad_planner.py) zusammen mit
   [`commonroad_planner_worker.py`](../commonroad_planner_worker.py) lesen:
   Host bereitet Dateien vor, Container plant.
7. [`commonroad_scenario_experiment.py`](../commonroad_scenario_experiment.py)
   als getrennten zyklischen CommonRoad-Versuch lesen.

Die Kommentare im Quellcode sollen Entscheidungen, Einheiten und nicht
offensichtliche Transformationen erklären. Sie ersetzen weder diese
Systemübersicht noch die detaillierten Anforderungen.
