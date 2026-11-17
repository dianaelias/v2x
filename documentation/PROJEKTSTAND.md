# UrbanING-V2X: Projektstand und Einstieg

Dieses Dokument hält fest, was im Projekt bisher eingerichtet wurde, wie die ersten Daten-Reader funktionieren und was SUMO, CARLA und Lanelets bedeuten.

Für den Einstieg in die gesamte Codebasis siehe außerdem die
[technische Dokumentation](./TECHNISCHE_DOKUMENTATION.md). Die aus dem Exposé
abgeleiteten, mit dem aktuellen Implementierungsstand abgeglichenen Ziele stehen
in der [technischen Anforderungsspezifikation](./TECHNISCHE_ANFORDERUNGEN.md).

## Kurzfassung

- Heruntergeladen wurde eine einzelne UrbanING-V2X-Sequenz: `20241126_0017_crossing1_00`.
- Die heruntergeladenen Dateien sind mehrteilige 7z-Archive. Beide Teile werden zum vollständigen Entpacken benötigt.
- Die Sequenz wurde in `datasets/UrbanIng-V2X` entpackt.
- `urbaning_sumo_data.py` exportiert Fahrzeugposen als CSV und Lanelet2-Kartengrenzen als JSON.
- `urbaning_cam.py` erzeugt lesbare synthetische CAM-Release-2-Datensätze aus den annotierten Objekttracks.
- `v2x_experiment.py` vergleicht keine V2X-Nachrichten, ideale CAMs und verzögerte/verlorene CAMs bis zur LDM-Eingabe.
- `run_commonroad_planner.py` und `commonroad_planner_worker.py` leiten drei empfangene CAM-Varianten an den offiziellen CommonRoad Reactive Planner weiter; der Worker läuft isoliert in Docker/Python 3.11.
- `commonroad_scenario_experiment.py` testet zyklisches Replanning auf einer CommonRoad-T-Kreuzung mit idealen CAMs bzw. verzögerter, optional verlustbehafteter Zustellung und Konstantgeschwindigkeits-Extrapolation.
- Die CommonRoad-T-Kreuzung ist ein kontrollierter Planner-Testfall, aber noch kein belastbarer Wirkungsnachweis: frühere Läufe blieben identisch; der aktuelle Queue-Test zeigt Verzögerungs- und seedabhängige Paketverlust-Effekte.
- Die abgestimmte Neuplanungsrate ist 100 ms (10 Hz); der T-Kreuzungs-Runner plant pro 0,1-s-Schritt neu. Ältere Ergebnisdateien können noch aus dem vorherigen 300-ms-Takt stammen; vor dem Vergleich ist ihr gespeichertes `replanning_period_s` zu prüfen. Die aktuellen Szenario-533-Läufe sind mit 100 ms erzeugt.
- Das erzeugte Karten-JSON ist noch kein SUMO-Netz. Eine Umwandlung in SUMOs Netzformat steht noch aus.
- Die synthetische CAM-Ausgabe ist kein ASN.1-UPER-Funkpaket. Der CommonRoad-Runner hat nun eine einfache CAM-Queue für Latenz und optionalen Paketverlust; der separate UrbanING-Kommunikationsversuch ist weiterhin nicht an den zyklischen Planner angebunden.

## Abgestimmtes Forschungsdesign

- **Planer:** CommonRoad Reactive Planner 2025.1 ist als Werling-Frenét-Planer festgelegt.
- **Daten:** UrbanING-V2X ist die primäre realitätsnahe Grundlage; native CommonRoad-Szenarien ergänzen sie für kontrollierte Konflikte. Reproduzierbares Startzeit-Tuning ist bei zu ruhigen Realdaten möglich.
- **Variante A:** Der Akteur bleibt in der Ground Truth, wird aber erst lokal in die Planner-Szene aufgenommen, wenn sein Abstand zum abgeleiteten Konfliktpunkt höchstens 15 m beträgt. Diese angenommene Sichtfreigabe bleibt aktiv. Das ist eine dokumentierte Näherung, keine echte Gebäude-/Sichtlinienberechnung; die XML enthält keine Gebäudeflächen. Die TTC-Alternative `TTC < 1,5 s` ist nicht gewählt.
- **CAM:** Ein synthetisches Python-Dict/JSON mit den relevanten ETSI-CAM-Feldern reicht für den Planner-Versuch. Binäre ASN.1-UPER-Kodierung ist nicht vorgesehen.
- **Hauptmatrix für Variante C:** 5 Latenzen (50, 100, 200, 300, 500 ms) × 4 Verlustraten (0 %, 5 %, 10 %, 20 %) × 5 Konfliktszenarien × 5 Seeds = **500 Läufe**. A/B-Referenzläufe kommen zusätzlich hinzu und sind in dieser Zahl nicht enthalten.
- **Zusatzreihe:** Jitter ist als separate Reihe mit Normalverteilung `μ = jeweilige Latenz`, `σ = 20 ms` vorgesehen; sie gehört nicht zu den 500 Läufen.
- **Metriken:** Pläne sollen an gleichen absoluten Zukunftszeitpunkten in Frenét-`Δs`/`Δd` verglichen werden; dazu kommen Jerk, minimaler 2D-Footprint-Abstand und TTC am Konfliktpunkt. Für die LDM ist noch zwischen Sample-and-Hold und linearer Extrapolation bis zum vorgeschlagenen 1-s-Timeout zu entscheiden.

Die Festlegungen und ihr Umsetzungsstatus stehen in der
[Anforderungsspezifikation](./TECHNISCHE_ANFORDERUNGEN.md). Die 500-Läufe-
Matrix ist noch nicht als automatisierter Batch-Runner implementiert. Die
CommonRoad-Ausführung muss außerdem 50-ms-Latenz trotz 100-ms-Zeitschritt
korrekt unterstützen, bevor diese Stufe ausgewertet wird.

## Was liegt wo?

```text
UrbanING-V2X/
├── .venv/                         # Python-Umgebung des Projekts
├── datasets/UrbanIng-V2X/          # heruntergeladene und entpackte Daten
│   ├── dataset/
│   │   └── 20241126_0017_crossing1_00/
│   │       ├── vehicle1_state/     # zeitgestempelte JSON-Posen
│   │       ├── vehicle2_state/
│   │       ├── timesync_info.csv   # Zeitzuordnung der Sensoren
│   │       └── ...                 # Kameras, LiDAR, Kalibrierung, Wetter
│   ├── labels/                     # Objektlabels der Sequenz
│   ├── crossings_lanelet2map.osm   # HD-Karte im Lanelet2/OSM-Format
│   ├── av_vehicle_data.json        # Abmessungen und Fahrzeugparameter
│   └── labels_av_track_ids.json    # Zuordnung der Track-IDs
├── download_urbaning.py            # Sequenz oder Gesamtdatensatz herunterladen
├── urbaning_sumo_data.py           # Posen und Lanelet-Karte exportieren
├── urbaning_cam.py                 # Annotationstracks als synthetische CAMs modellieren
├── v2x_experiment.py               # A/B/C-Kommunikations- und LDM-Vergleich
├── Dockerfile.commonroad           # isolierte Planner-Umgebung (Python 3.11/Linux amd64)
├── run_commonroad_planner.py       # Host-Launcher und CAM-Fallvorbereitung
├── commonroad_planner_worker.py    # CommonRoad-Szenarioaufbau und Einmal-Replan
├── commonroad_scenario_experiment.py # geschlossener CommonRoad-Replan mit CAM-Alterung
├── commonroad_scenarios/           # TUM/CommonRoad-Beispielszene und Planner-YAML
└── sumo_input/                     # erzeugte CSV-, JSON- und JSONL-Dateien
```

## Download und Entpacken

`download_urbaning.py` verwendet die Download-Funktionen aus dem installierten `urbaning`-Paket.
Das Paket ist in der Projektumgebung `.venv` installiert. Für eine neue Umgebung wäre die Installation:

```zsh
.venv/bin/python -m pip install urbaning
```

```zsh
.venv/bin/python download_urbaning.py
```

Ohne zusätzliche Optionen lädt das Skript nur die voreingestellte Sequenz `20241126_0017_crossing1_00`. Für eine andere Sequenz kann ein Name angegeben werden:

```zsh
.venv/bin/python download_urbaning.py --sequence-name 20241126_0008_crossing1_00
```

`--all` lädt den gesamten Datensatz und sollte nur bewusst verwendet werden:

```zsh
.venv/bin/python download_urbaning.py --all
```

Die Dataverse-Dateiliste enthält 108 Einträge für den Gesamtdatensatz. Eine einzelne Sequenz besteht aus mehreren zusammengehörigen Archivteilen (`.7z.001`, `.7z.002` und gegebenenfalls weiteren Teilen) sowie einer Label-JSON-Datei. Alle Archivteile einer Sequenz sind nötig, um sie vollständig zu entpacken.

Zum Entpacken wird 7-Zip benötigt. Falls es noch nicht installiert ist, kann es auf dem Mac über Homebrew installiert werden:

```zsh
brew install sevenzip
```

Auf diesem Mac wurde das Programm unter `/opt/homebrew/bin/7zz` gefunden; der Aufruf für die Daten lautet:

```zsh
.venv/bin/python -c "from urbaning.data import unzip_dataset; unzip_dataset(dataset_folder='datasets/UrbanIng-V2X', sevenz_executable='/opt/homebrew/bin/7zz')"
```

## Was bedeutet die Datenstruktur?

- `vehicle1_state` und `vehicle2_state` enthalten Fahrzeugzustände als JSON-Dateien. Der Zeitstempel steht im Dateinamen in Millisekunden.
- In jeder State-Datei enthält `gTv` eine 4×4-Transformationsmatrix. Die letzte Spalte gibt die Fahrzeugposition `x`, `y`, `z` an; der Rotationsanteil beschreibt die Orientierung.
- Die State-Dateien liegen in der getesteten Sequenz im Abstand von 10 ms vor, also mit 100 Zuständen pro Sekunde.
- `timesync_info.csv` ordnet Zeitpunkte verschiedener Sensoren ein. Die Beispielzeilen liegen dort 100 ms auseinander. Der erste Export liest direkt die State-Dateien und verwendet diese Timesync-Tabelle noch nicht.
- `labels/<sequenz>.json` enthält Objektlabels. Diese sind nicht dasselbe wie die Fahrzeugpose-Dateien.
- `crossings_lanelet2map.osm` ist eine XML-Datei mit geografischen Kartenpunkten und Lanelet2-Objekten. Sie ist keine SUMO-Netzdatei.

## Prüfung: Lanelet2-Karte nach SUMO

SUMO 1.28.0 ist in `.venv` installiert. Die direkte Probe mit SUMOs Standard-OSM-Importer:

```zsh
.venv/bin/netconvert --osm-files datasets/UrbanIng-V2X/crossings_lanelet2map.osm --output-file sumo_input/netconvert_probe/map.net.xml --verbose
```

bricht mit `Error: No nodes loaded.` ab. Die Karte enthält 595 `type=lanelet`-Relationen, 62 `type=regulatory_element`-Relationen und Ways, die mit Lanelet2-Tags wie `type=curbstone` oder `type=line_thin` markiert sind. Es sind nicht die üblichen getaggten OSM-Straßenkanten, die SUMOs Standardimporter als Fahrnetz importiert. Daher ist kein gültiges SUMO-Netz entstanden. Die Original-OSM-Datei blieb unverändert.

Das heißt nicht, dass die Karte unbrauchbar ist: Für eine korrekte Umwandlung brauchen wir einen Lanelet2-kompatiblen Konverter oder müssen aus den Lanelet-Grenzen ein SUMO-Netz mit Kanten, Fahrtrichtungen und Verbindungen aufbauen. Zusätzlich müssen wir den richtigen Kartenausschnitt und die Verkehrsregeln prüfen.

## Was macht `urbaning_sumo_data.py`?

Das Skript liest die Zustands-JSONs direkt, ohne die fehlerhafte Label-Verarbeitung des höheren `urbaning.Sequence`-Laders zu verwenden.

`load_vehicle_trajectory(root_folder, sequence_name, vehicle_name)`:

- liest die State-Dateien eines Fahrzeugs in Zeitreihenfolge;
- berechnet `time_s` relativ zum ersten Zustand der Sequenz;
- übernimmt Position und Orientierung aus `gTv`;
- gibt pro Zustand Zeitstempel, Position, Winkel und eine für SUMO umgerechnete Winkelspalte zurück.

`write_trajectory_csv(trajectory, output_path)` schreibt diese Werte als CSV.

`load_lanelet_map(osm_path)`:

- liest die OSM-Nodes, Ways und Lanelet-Relationen;
- projiziert Längen- und Breitengrade nach UTM Zone 32N;
- verschiebt die Koordinaten auf den GPS-Ursprung von `crossing1`, damit sie zur Pose dieser Sequenz passen;
- gibt je Lanelet linke und rechte Begrenzungslinien samt Kartentags zurück.

**Koordinatenhinweis:** Die Map- und Fahrzeugwerte werden in Metern relativ zum Ursprung von `crossing1` ausgegeben. Eine Probe für beide Fahrzeuge lag wenige Meter von OSM-Knoten entfernt. Das ist ein Plausibilitätscheck, aber noch keine vollständige Prüfung der Karten- und Trajektorienausrichtung.

**Winkelhinweis:** `heading_rad` ist der aus der Matrix berechnete Winkel in Radiant. `sumo_angle_deg` ist die umgerechnete Winkelkonvention für SUMO. Bei der späteren Integration muss die Fahrtrichtung anhand der konkreten SUMO-Funktion geprüft werden.

**Kartenhinweis:** Der aktuelle Reader exportiert alle Lanelet-Relationen, die er in der gemeinsamen OSM-Karte findet. Er filtert die Karte noch nicht auf den Ausschnitt der ausgewählten Sequenz.

## Erzeugte Dateien und erneuter Export

Ein erstmaliger Lauf hat die CSV- und JSON-Dateien bereits erzeugt. Dieselben Dateien lassen sich jederzeit erneut schreiben:

```zsh
.venv/bin/python urbaning_sumo_data.py
```

Die Standardwerte sind die oben gezeigte Sequenz und der Ausgabeordner `sumo_input/`. Der Lauf hat für jedes Fahrzeug 2.800 Posen und insgesamt 595 Lanelets exportiert:

- `sumo_input/20241126_0017_crossing1_00_vehicle1_trajectory.csv`
- `sumo_input/20241126_0017_crossing1_00_vehicle2_trajectory.csv`
- `sumo_input/lanelet_map.json`

Diese Dateien sind abgeleitete Arbeitsdateien. Die Originaldaten liegen weiterhin unter `datasets/UrbanIng-V2X/`.

## SUMO, CARLA und Lanelets für Einsteiger

### SUMO

SUMO bedeutet **Simulation of Urban MObility**. Es ist ein Verkehrsfluss-Simulator: Fahrzeuge bewegen sich auf einem Straßennetz, und Python kann die Simulation über TraCI steuern. SUMO eignet sich unter anderem, wenn es um Verkehr, Fahrzeugbewegungen, Kommunikationsereignisse und Latenzen geht. Es ist keine 3D-Welt mit fotorealistischen Kamerabildern.

### CARLA

CARLA ist ein stärker auf autonome Fahrzeuge ausgerichteter 3D-Simulator. Es kann eine virtuelle Umgebung, Fahrzeuge und Sensoren wie Kameras oder LiDAR darstellen. Dafür ist es meist aufwendiger als eine leichte 2D-Verkehrssimulation.

Die README des UrbanING-Repos erwähnt einen `digital_twin`-Ordner für CARLA. Dieser Ordner ist in der bisher heruntergeladenen und entpackten Sequenz nicht vorhanden. Ob und wie dieser Teil für das konkrete Experiment genutzt werden kann, wäre separat zu prüfen.

### Lanelets

Ein Lanelet ist ein Stück befahrbarer Fahrspur, beschrieben durch linke und rechte Begrenzung. Eine Karte besteht aus vielen solchen Stücken und zusätzlichen Regeln oder Tags. Lanelet2 ist also ein **Kartenformat bzw. eine Art, Straßen zu beschreiben**, kein Simulator.

Lanelets sind nicht nötig, weil man UrbanING-V2X nicht in CARLA ausführen könnte. Sie sind hier relevant, weil die verfügbare HD-Karte in Lanelet2/OSM vorliegt. SUMO erwartet für seine Simulation ein eigenes Straßennetz, typischerweise als `net.xml`. Zwischen diesen Formaten muss eine passende Umwandlung oder ein eigener Netzaufbau erfolgen.

## ETSI-CAM: Was liegt im Datensatz vor?

Im Repo und in den geprüften Dateien der heruntergeladenen Sequenz wurde keine ETSI-CAM-Nachricht oder CAM-Protokollkodierung gefunden. Die CAM-Suche im Repo fand keine Treffer; in den lokalen Dateien gab es keine CAM-Felder wie `stationID`, `generationDeltaTime` oder `referencePosition`. Der Texttreffer `CAM` in `calibration.json`/`timesync_info.csv` stammt aus Bezeichnungen mit `camera`, nicht aus einer Cooperative Awareness Message.

Es gibt aber Bewegungsdaten, aus denen sich **ein Teil der Inhalte einer CAM-Nachricht** ableiten lässt:

- `vehicle1_state` und `vehicle2_state`: Pose (`gTv`), Geschwindigkeit (`vV`), Beschleunigung (`vA`) und Rotationsraten (`W`);
- `av_vehicle_data.json`: Fahrzeuglänge, -breite, -höhe und Achsabstände;
- `timesync_info.csv`: Sensor-Zeitpunkte und Zeitzuordnung;
- Labels: getrackte Objekte mit Typ, Abmessungen, Zeitstempeln, Positionen und Orientierungen.

Das ist nicht dasselbe wie echte gesendete/empfangene CAMs. Insbesondere enthält der Datensatz hier keine ETSI-Nachrichten mit deren Station-ID, CAM-Erzeugungszeit, Kodierung und Empfangszeit. Für eine V2X-Latenzstudie könnten wir später aus den Ground-Truth-Bewegungsdaten **synthetische CAM-Nachrichten nach ETSI-Inhalt modellieren** und Sende-/Empfangs- sowie Verzögerungsereignisse in einer Simulation erzeugen. Das wäre ein eigenes Simulationsmodell und müsste klar von gemessenen CAM-Nachrichten unterschieden werden.

Welche CAM-Parameter im Experiment gebraucht werden, hängt von der konkreten Forschungsfrage und der anzuwendenden ETSI-CAM-Version ab. Vor einer Implementierung sollten wir die benötigten Felder und ihre Quellen einzeln festlegen, statt anzunehmen, dass die Rohdaten bereits standardkonforme Nachrichten darstellen.

### Erster synthetischer CAM-Export

Die Normreferenz wurde geprüft: **ETSI TS 103 900 V2.2.1** beschreibt den Cooperative Awareness Service, Release 2, und dessen CAM-Format. Die Release-2-ASN.1-Definition importiert gemeinsame ITS-Datentypen aus dem ETSI Common Data Dictionary. Die zugehörige ASN.1-CAM-Nachricht wird für echte Nachrichten als ASN.1 mit UPER kodiert. Unsere erste Datei ist bewusst ein menschenlesbares semantisches JSON-Modell und keine solche Bitkodierung.

Das Skript erzeugt sie mit:

```zsh
.venv/bin/python urbaning_cam.py
```

Standardmäßig liest es `labels/20241126_0017_crossing1_00.json` und `labels_av_track_ids.json`, wählt `vehicle2` als Sender und schreibt eine CAM pro Track-Zeitpunkt als JSON Lines nach `sumo_input/synthetic_cam_messages.jsonl`. Für die Beispielsequenz ist `vehicle2` Track 115 und `vehicle1` Track 116. Der Standardexport enthält deshalb 200 Records von Track 115. `--sender vehicle1` wählt stattdessen Track 116. `--all-vehicles` ist eine ausdrückliche Option, um wieder alle unterstützten annotierten Tracks einzubeziehen.

In jedem Record sind die CAM-Pflichtteile als Hierarchie abgebildet: PDU-Header, `generationDeltaTime`, `basicContainer` mit Stationstyp und Referenzposition sowie der Fahrzeug-`highFrequencyContainer`. Die Header-Felder heißen in der verwendeten ETSI-ASN.1-Schreibweise `messageId` und `stationId` (kleines `d`), auch wenn man sie umgangssprachlich oft als `messageID`/`stationID` schreibt. Die Stations-ID ist eine reproduzierbare CRC32-ID aus Sequenzname und Track-ID, keine reale ITS-Station-ID.

Die Position und Orientierung kommen aus den 10-Hz-Annotations. Geschwindigkeit, Längsbeschleunigung, Gierrate und Krümmung werden numerisch aus den Trackpunkten abgeleitet. Die Annotationposition ist der Mittelpunkt der Bounding Box; das Skript verschiebt sie für `referencePosition` entlang der Fahrzeuglänge zur Bodenprojektion der Frontmitte. Länge, Breite und die Fahrzeugklassenzuordnung kommen aus den Labels. `Truck` wird vorläufig als `heavyTruck`, `Van` als `lightTruck` abgebildet; das ist eine Annahme und sollte fachlich bestätigt werden.

Messunsicherheiten liegen nicht in diesen Ground-Truth-Annotations vor und werden daher als `unavailable` markiert. Die Datenwerte sind lesbar in SI-Einheiten abgelegt und mit Einheiten-Suffixen benannt. Der Export ist noch nicht nach ASN.1-Datentypen quantisiert oder UPER-kodiert und damit kein direkt sendbares ETSI-Paket.

Eine Plausibilitätsprüfung ergab einen Geschwindigkeitssprung bei Track 150: Die annotierte Position springt über ein 100-ms-Intervall etwa 8 m und im folgenden Schritt wieder zurück. Der abgeleitete Wert wird nicht gekappt, sondern im Record mit `derived_speed_above_55_mps` als Qualitätswarnung markiert. Die 55-m/s-Grenze ist hier nur ein Screening-Hinweis, kein ETSI-Grenzwert.

Dieses CAM-Generierungsskript allein setzt keine Verzögerungsqueue, Paketverlust, Scheduling oder Empfänger-LDM um. Dafür gibt es den separaten Kommunikationsprototyp `v2x_experiment.py`; dieser ist noch nicht mit dem zyklischen Planner verbunden. Für das vollständige Experiment müssen Sendezeit, Empfangszeit und Ground-Truth-Zeit eines CAM-Samples gemeinsam eindeutig behandelt werden.

## Erster A/B/C-Kommunikationsversuch (Kommunikationsprototyp)

`v2x_experiment.py` verbindet die annotierten Rollen mit einer einfachen diskreten Ereignissimulation. `vehicle2`/Track 115 erzeugt pro Ground-Truth-Tick eine CAM; `vehicle1`/Track 116 ist der Ego-Empfänger. Die Queue hält CAMs bis zur konfigurierten Empfangszeit zurück. Bei Ankunft wird die neueste Nachricht in einer minimalen LDM gespeichert. Der Versuch läuft auf den gemeinsamen 10-Hz-Zeitpunkten der Annotationen.

Die folgenden Varianten beschreiben nur den frühen Kommunikations-/LDM-
Prototyp. Insbesondere ist dessen `A_no_v2x` mit leerer LDM **nicht** die
abgestimmte Masterarbeits-Baseline A (lokale Sensorik mit Sichtlinienfreigabe).

Varianten:

- `A_no_v2x`: Es wird keine CAM gesendet und die LDM bleibt leer.
- `B_ideal`: CAMs werden ohne Verzögerung und Verlust direkt in die LDM übernommen.
- `C_latency_test`: CAMs durchlaufen eine Queue mit fester Verzögerung und optional reproduzierbarem Paketverlust.

Start mit 200 ms Verzögerung, 10 % Paketverlust und festem Zufalls-Seed:

```zsh
.venv/bin/python v2x_experiment.py --delay-ms 200 --packet-loss 0.1 --seed 7
```

Der Paketverlust ist eine Wahrscheinlichkeit pro erzeugter Nachricht. Derselbe Seed und dieselben Parameter erzeugen wieder dieselben Drop-Entscheidungen. `--packet-loss 0` schaltet Verlust aus. Die drei Varianten schreiben ihre Tickdaten in `sumo_input/experiment/experiment_steps.csv` und eine Zusammenfassung in `sumo_input/experiment/experiment_summary.json`.

Erfasste Kommunikations-/LDM-Messwerte sind: Anzahl erzeugter, empfangener und verlorener CAMs; noch unterwegs befindliche Pakete am Simulationsende; LDM-Verfügbarkeit; AoI des zuletzt empfangenen CAMs; sowie der Positionsfehler zwischen der im letzten CAM enthaltenen Sender-Frontreferenzposition und der aktuellen annotierten Referenzposition des Senders. `in_window_delivery_ratio` zählt nur Pakete, die noch innerhalb des 20-s-Szenarios ankamen; `eventual_delivery_ratio` zieht nur die simulierten Verluste ab und berücksichtigt damit ein bei Szenarioende noch unterwegs befindliches Paket.

Ein geprüfter Lauf mit den obigen Parametern ergab 200 Ticks je Variante. A hatte keine LDM-Einträge. B empfing 200/200 Nachrichten und AoI/Positionsfehler waren 0. C verlor 20 Nachrichten, empfing 179 innerhalb des Szenariofensters und hatte am Ende eine Nachricht noch unterwegs; der mittlere AoI war rund 0,229 s, der mittlere Senderpositionsfehler rund 0,755 m. Das ist nur ein Funktions-/Plausibilitätstest für die Kommunikationskette, kein Forschungsergebnis.

### Szenario-Screening und zweiter Sequenzlauf

Die Mittelpunktsabstände der verbundenen Fahrzeuge wurden über die 17 `crossing1`-Labeldateien verglichen. In der zuerst verwendeten Sequenz `20241126_0017_crossing1_00` beträgt der minimale Abstand etwa 15,79 m; sie eignet sich daher eher als Kommunikationsprobe als als enger Sicherheitskonflikt. Zwei Sequenzen mit weniger als 3,1 m Abstand (`20241126_0008_crossing1_00` und `_01`) erwiesen sich bei genauer Prüfung als nahezu stationäre Tracks über die gesamten 20 Sekunden und wurden nicht als dynamischer Konflikt gewählt.

Als erster bewegter Annäherungsfall wurde `20241126_0022_crossing1_09` gewählt. Die verbundenen Fahrzeuge bewegen sich dort aufeinander zu und erreichen bei etwa 12,5 s einen annotierten Mittelpunktabstand von rund 7,13 m. Für den Test wurde nur deren 3,33-MB-Label-JSON geladen; Kamera- und LiDAR-Archive dieser Sequenz sind nicht erforderlich. Die Sequenz hat 200 gemeinsame 10-Hz-Ticks.

Der Versuch lässt sich so wiederholen:

```zsh
.venv/bin/python v2x_experiment.py --sequence 20241126_0022_crossing1_09 --delay-ms 200 --packet-loss 0.1 --seed 7 --output sumo_input/experiment_0022_09
```

Die Ausgaben liegen in `sumo_input/experiment_0022_09/`. Im geprüften Lauf verhielt sich A wie definiert (keine CAMs/LDM-Einträge), B lieferte 200 frische CAMs, und C hatte dieselbe Kommunikationsbilanz wie beim ersten Szenario: 20 simulierte Verluste und ein Paket noch in der Queue am Szenarioende. In der Annäherungsphase von 10 bis 15 s stieg der maximale Fehler der zuletzt empfangenen Senderposition bei C auf etwa 1,70 m; B blieb bei null.

Der Abstand von 7,13 m ist zunächst nur der euklidische Abstand der annotierten Bounding-Box-Mittelpunkte. Vor einer Aussage über tatsächliche Kollisionsnähe müssen Orientierung und Fahrzeugabmessungen zur Berechnung des Rechteckabstands berücksichtigt werden. Der Lauf belegt weiterhin nur die Kommunikations-/LDM-Wirkung; ein Planer und dessen Sicherheitsreaktion fehlen noch.

### Fahrzeugumrisse visualisieren

`visualize_encounter.py` stellt beide Tracks mit orientierten 2D-Fahrzeugrechtecken dar und berechnet den geometrischen Abstand zwischen den Karosserie-Footprints. Der Code verwendet die annotierten Mittelpunktpositionen, Fahrzeuglänge/-breite und Heading; Überlappung wird mit dem Separating-Axis-Theorem geprüft. Für getrennte Rechtecke wird der kleinste Kantenabstand berechnet.

```zsh
.venv/bin/python visualize_encounter.py
```

Für `20241126_0022_crossing1_09` ergaben sich über 200 Zeitpunkte: minimaler Mittelpunktabstand 7,15 m, minimaler 2D-Footprint-Abstand 3,95 m und keine Rechtecküberlappung. Die engste Stelle liegt bei etwa 12,6 s. Grafik, Tick-CSV und JSON-Zusammenfassung liegen in `sumo_input/encounter/`.

Das ist eine planare Geometrieprüfung, keine vollständige Sicherheitsbewertung: Höhe, Messunsicherheit, dynamische Bremsbarkeit und erforderlicher Sicherheitsabstand werden nicht berücksichtigt. Die in der Grafik gezeigte 2-m-Referenz ist nur ein frei gewählter Vergleichswert, kein Standardgrenzwert. Der Kandidat ist damit ein dynamischer Annäherungsfall, aber kein tatsächlicher 2D-Kontakt und noch kein nachgewiesener kritischer Konflikt.

**Wichtige Grenze des Kommunikationsharness:** `v2x_experiment.py` bleibt ein reiner Kommunikations-/LDM-Test und berechnet keine Planänderungen, Temporal Consistency, Ruck/Jerk oder plannerabhängige TTC. Die Planneranbindung ist ein separater Einmal-Replan-Adapter, siehe unten.

## CommonRoad Planner: erster Anschluss

Der offizielle CommonRoad Reactive Planner 2025.1 verlangt Python `>=3.9,<3.12`; ein Teil seiner Collision-Checker-Binaries wird für Linux x86_64 veröffentlicht. Die Projektumgebung läuft auf Python 3.14 und bleibt deshalb unangetastet. Auf dem Apple-Silicon-Mac läuft der Planner in einem `linux/amd64` Docker-Container mit Python 3.11.

Einmaliges Image bauen und den Vergleich ausführen:

```zsh
.venv/bin/python run_commonroad_planner.py
```

Der erste Lauf baut das Image und startet danach eine Planung je Variante. Ein erneuter Lauf kann den Build überspringen:

```zsh
.venv/bin/python run_commonroad_planner.py --skip-build --planning-time 12.0 --delay-ms 200
```

Der Host bereitet Folgendes vor:

- `vehicle1`-Zustand zur gewählten Planungszeit;
- dieselbe aufgezeichnete Ego-Referenzroute für A, B und C;
- A ohne CAM, B mit dem aktuellen CAM von `vehicle2`, C mit dem letzten CAM, das bei der gewählten festen Verzögerung bereits angekommen wäre;
- eine separate Ground-Truth-Datei für die nachträgliche Bewertung.

Der Container baut aus der Ego-Referenz einen **synthetischen 4,5-m-Lanelet-Korridor** und ein CommonRoad-Planning-Problem. B/C bekommen den CAM-Zustand als dynamisches Fahrzeughindernis; die Zukunftsannotation des Senders wird nicht an den Planner übergeben. Für die Kollisionsprüfung sagt der Worker dieses Hindernis mit konstanter Geschwindigkeit aus dem empfangenen CAM voraus. Nach der Planung darf die getrennte Ground Truth zur Auswertung verwendet werden.

Die Planner-Ausgabe liegt in `sumo_input/commonroad_planner/planner_results.json`. Ein geprüfter Einmal-Replan bei 12,0 s erzeugte eine zulässige Trajektorie in allen drei Varianten. In B/C waren jeweils genau ein dynamisches CommonRoad-Hindernis vorhanden; CAM-Alter waren 0 bzw. 0,2 s. Alle Varianten wählten dieselbe Trajektorie. Der kleinste Abstand zur CAM-basierten Hindernisprognose in B war rund 3,85 m und in C rund 4,03 m; mit dem eingestellten 0,2-m-Sicherheitsrand entstand kein Grund für eine andere Bahn. Die Ground-Truth-Bewertung ergab für die ausgewählte Bahn in allen Varianten minimal rund 4,10 m Footprint-Abstand.

Das ist der erste echte Anschluss, aber noch kein belastbarer Stabilitätsversuch: Es wird nur **ein** Replan-Zeitpunkt analysiert, und der synthetische Korridor ist nicht die echte UrbanING-Laneletkarte. Dass A/B/C hier gleich sind, ist angesichts der verbleibenden Abstände plausibel und bedeutet nicht, dass der Planner oder die CAM-Queue ignoriert wird. Für die Forschungsfrage brauchen wir mehrere Replan-Ticks sowie Szenarien/Latenzen, bei denen die empfangene Information tatsächlich die Kollisions- oder Kostenbewertung ändert. Der Docker-Linux/x86_64-Build läuft unter CPU-Emulation auf dem M4; Laufzeiten müssen für spätere Auswertungen geprüft werden.

### Welche Simulation passt?

- Für eine leichte 2D-Simulation von Fahrzeugbewegungen und V2X-Latenzen ist SUMO ein plausibler Startpunkt.
- Für 3D-Sensorik, visuelle Szenen oder realistischere Sensor-Simulation wäre CARLA näher am Ziel.
- Lanelet2 kann als Kartenquelle dienen; es zwingt nicht zur Wahl eines bestimmten Simulators.

Die bisherigen Exporte lösen noch nicht die Wahl zwischen SUMO und CARLA. Sie bereiten die echten Fahrzeugtrajektorien und Kartengeometrie vor, damit die Wahl anhand des gewünschten Experiments getroffen werden kann.

## Bekannter Stolperstein

Beim Test von `Sequence(root_folder, sequence_name)` ist der installierte UrbanING-Reader beim Laden der Objektlabels mit einem SciPy-Fehler abgebrochen: `Expected last dimension of angles to match number of sequence axes specified, got 200.` Deshalb liest `urbaning_sumo_data.py` die Fahrzeugposes direkt aus den JSON-Dateien. Der SDK-Fehler wurde noch nicht behoben.

## Sinnvolle nächste Schritte

1. CAM-Record und Qualitätswarnungen stichprobenartig gegen die Quellannotations prüfen.
2. Sendeintervall, künstliche Verzögerungsverteilung, Queue-Verhalten und LDM-Update-Regel für die Forschungsfrage festlegen.
3. Entscheiden, ob SUMO für die Forschungsfrage genügt oder ob der CARLA-Digital-Twin benötigt wird.
4. Die Lanelet2-Karte geeignet umwandeln und ein kleines SUMO-Netz (`net.xml`) erstellen, bevor Fahrzeuge mit TraCI simuliert werden.

Die Datensatz-Lizenz und die Zitierhinweise aus dem UrbanING-Repository sind bei der Nutzung in der Masterarbeit zu beachten.

## CommonRoad-T-Kreuzung: kontrollierter Planner-Test

In `commonroad_scenarios/` liegt die ursprüngliche Planner-Beispielszene
`ZAM_Tjunction-1_42_T-1.xml` mit ihrer YAML-Konfiguration. Sie hat ein
Planning Problem (ID 60000), fünf dynamische Fahrzeuge und `dt=0,1 s`.
Zusätzlich wurde eine nicht redundante Szene aus dem Seminararbeits-Repository
importiert: `commonroad_scenarios/imported/ZAM_Tjunction-1_533_T-1.xml`.
Die Datei wurde unverändert kopiert (SHA-256
`5ec8ef3f6094962e89e1a84f9acaf706911278285158b90286ef4056e3904ad0`); die
BSD-3-Clause-`LICENSE.txt` liegt direkt daneben. CommonRoad empfiehlt für
seine Benchmarks die Referenz von Althoff, Koschi und Manzinger (2017).

Der geschlossene Runner nutzt die CommonRoad-Karte und Szenariospuren. Er
plant alle 100 ms neu, mit einem 2-s-Planungshorizont. B ersetzt den
gewählten Akteur durch den aktuellen CAM-Zustand. C erzeugt pro Szenariotakt
ein CAM, legt es um die konfigurierte Latenz in eine Queue und kann es mit
`--packet-loss` reproduzierbar verwerfen. Die LDM hält den letzten
zugestellten Zustand und extrapoliert mit konstanter Geschwindigkeit und
Richtung. Nicht ausgewählter Verkehr bleibt in der Szene. Die wirkliche
Akteurspur wird nur für die nachträgliche Clearance-Auswertung verwendet. Der
Parameter `--actor-time-offset-ms`
verschiebt die ausgewählte aufgezeichnete Spur relativ zu Ego: negative Werte
ziehen sie vor, positive Werte verzögern sie. Der Wert muss auf dem 100-ms-
Raster liegen. Der Versatz ändert keine Maße oder Sicherheitsränder.

**Konflikt-Tuning-Test:** Für Szenario 533, Obstacle 1 wurde die Akteurspur um
3,6 s vorgezogen (`--actor-time-offset-ms -3600`). Der Konfliktpunkt ist der
erste Schnittpunkt zwischen Ego-Referenzlinie und Akteur-Mittellinie:
`(16,73 m; 2,74 m)`. Bei `d_sight=15 m` wird der Akteur in A ab 4,7 s
lokal sichtbar; ohne Zeitversatz ab 8,3 s. Danach bleibt die Sichtbarkeit
aktiv. Alle Varianten absolvierten 147 Schritte und Planungen. Bei 200 ms
überlappten A/B/C nicht mit Ground Truth; der minimale Abstand war 16,6 mm
für A und B sowie 8,7 mm für C. A und B hatten identische Trajektorien;
B/C wichen 3,4 cm im Mittel und 12,1 cm maximal ab. Bei 500 ms betrug die
B/C-Abweichung 41,8 cm im Mittel und 1,17 m maximal; der minimale Abstand
lag bei 9,6 mm. Der Sicherheitsrand blieb unverändert.

Reproduzierbare Ergebnisdateien:
- `sumo_input/commonroad_tjunction_533_actor1_baseline.json` (ohne Zeitversatz,
  200 ms)
- `sumo_input/commonroad_tjunction_533_actor1_advance_3600ms.json` (200 ms)
- `sumo_input/commonroad_tjunction_533_actor1_advance_3600ms_delay500ms.json`
  (500 ms)
- `sumo_input/commonroad_tjunction_533_actor1_advance_3600ms_loss20_seed7.json`
- `sumo_input/commonroad_tjunction_533_actor1_advance_3600ms_loss20_seed8.json`
- `sumo_input/commonroad_tjunction_533_actor1_advance_3600ms_loss0_seed7.json`

**20-%-Paketverlusttest:** Je Lauf wurden zwei vor t=0 erzeugte Warm-up-CAMs
zusätzlich in die Kommunikationsqueue geladen. Damit gab es 114 generierte
CAMs je Lauf. Bei Seed 7 wurden 26 (22,8 %) verworfen; bei Seed 8 28 (24,6 %).
Alle Varianten planten jeweils 147 Schritte. Gegen den 0-%-Kontrolllauf stieg
die B/C-Positionsabweichung bei Seed 7 von 3,4 cm im Mittel/12,1 cm maximal
auf 6,7 cm/24,9 cm. Bei Seed 8 war die C-Trajektorie trotz 20 % konfiguriertem
Verlust identisch zum 0-%-Lauf. C erreichte im Seed-7-Lauf 9,6 mm minimalen
Ground-Truth-Abstand (B: 16,6 mm), ohne diskrete Überlappungen. Die AoI von C
lag im Mittel bei 604 ms (Seed 7) bzw. 606 ms (Seed 8), maximal bei 3,5 s.
Eine Wiederholung mit Seed 7 reproduzierte Trajektorien, Drop-Zahlen und AoI
exakt.

Der Konflikttest ist ein Funktionsnachweis für Startzeit-Tuning, aber kein
belastbares Forschungsergebnis. A verwendet eine Distanzschwelle als
Sichtlinien-Näherung; beide CommonRoad-Szenarien besitzen keine statischen
Hindernisse/Gebäude, aus denen sich physische Verdeckung berechnen ließe.
Millimeter-Clearance ist keine ausreichende Sicherheitsmarge.
Die 20-%-Testreihe zeigt eine B/C-Abweichung, ist aber mit zwei Seeds und
einem Konfliktfall kein belastbarer Befund. Ein LDM-Timeout, der NLoS-Nachweis
und ein Pool von fünf begründeten Konfliktszenarien fehlen weiterhin.

In früheren Läufen mit der ursprünglichen Szene 42 blieben A/B/C bei sieben
getesteten Akteur-/Latenzkombinationen numerisch identisch; der engste
Ground-Truth-Footprint-Abstand lag bei etwa 1,60 m. Ein ausschließlich
diagnostischer Sensitivitätstest mit 2 m dynamischem Sicherheitsrand (kein
empfohlener Forschungswert) ließ A bis zum Ende planen, B/C scheiterten nach
53 Replans. Das Ergebnis belegt nur, dass die Hindernisrepräsentation
Plannerentscheidungen beeinflussen kann; Sicherheitsparameter dürfen nicht
allein zur sichtbaren Erzeugung von Variantenunterschieden gewählt werden.