# MMR E-Driverless Node Visualizer (`mmr_ros_graph`)

> Live browser cockpit for **ROS 2 Humble** computation graphs — built for **More Modena Racing (MMR)** E-Driverless.

[🇬🇧 English](#-english-documentation) | [🇮🇹 Italiano](#-documentazione-in-italiano)

---

## 🇬🇧 English Documentation

Live browser view and diagnostic tool for a ROS 2 Humble system. Nodes appear as draggable cards with input/output ports, topics render as Bezier wires, and the interface updates continuously via Server-Sent Events (SSE) while nodes start, stop, or reconfigure.

### Why Use This Tool
- Zero Qt/XQuartz rendering freezes on macOS VM setups (`ros2-mac`).
- Stays continuously live via DDS discovery instead of manual `rqt_graph` refreshes.
- Uses Humble's native `rclpy.qos.qos_check_compatible` to flag silent/incompatible wires immediately.
- Combines graph visualization, node parameters, topic `Hz`/echo, services/actions, and `/rosout` logs in one lightweight browser app with no frontend build step.

### Prerequisites
- **ROS 2 Humble** with `rclpy`, `rcl_interfaces`, and `rosidl_runtime_py` (included in `ros-humble-desktop`).
- **Python 3.10+** (standard library HTTP/SSE server; no external Node/npm dependencies needed to run).

### Installation & Workspace Build

Clone the repository into your ROS 2 workspace `src/` folder and build with `colcon`:

```bash
cd ~/ros2_ws/src
git clone https://github.com/saishshinde15/MMR_E-Driverless-Node-Visualizer.git mmr-ros-graph
cd ..
colcon build --symlink-install --packages-select mmr_ros_graph
source install/setup.bash
```

*(Note: `--symlink-install` is recommended so static UI updates and edits take effect immediately.)*

### How to Run

#### Ubuntu / Linux (or Windows with ROS 2 sourced)
```bash
ros2 run mmr_ros_graph live_graph
```
Or directly from the repository folder:
```bash
python3 live_graph        # Linux / macOS
py -3 live_graph          # Windows
```

#### macOS (ROS 2 running inside Linux VM / Colima)
Run from the **Mac terminal** (not inside `ros-shell`) so the page opens in your Mac browser while communicating over stdio JSON-lines with a single warm `live_graph worker` inside the container:
```bash
cd ~/ROS2/projects/mmr-ros-graph
python3 live_graph
```

Open `http://127.0.0.1:8765/` (automatically opens in your default browser; falls back to a free port if `8765` is busy). Press `Ctrl-C` in the terminal to stop both the HTTP server and the ROS 2 worker cleanly without leaving orphan nodes.

#### CLI Options & Subcommands
| Flag / Command | Description |
| --- | --- |
| `serve` *(default)* | Start local web server and open live graph UI |
| `snapshot` | Print one JSON graph snapshot to stdout and exit (`0` if healthy) |
| `stream` | Stream newline-delimited JSON snapshots when the graph changes |
| `worker` | Persistent bidirectional stdio RPC + stream ROS 2 node |
| `--port 8765` | Preferred HTTP port |
| `--period 1.0` | Discovery polling cadence in seconds |
| `--no-browser` | Print URL without automatically opening a browser tab |

### Features & Controls

#### Graph View Modes & Filters (`rqt_graph` parity)
- **View Mode selector**:
  - `Nodes & Topics (all)`: All nodes and topics, including clickable dashed stub wires for leaf topics (topics missing a publisher or subscriber).
  - `Nodes & Topics (active)`: Only active topics (at least 1 publisher and 1 subscriber) and connected nodes.
  - `Nodes only`: Direct node-to-node links with single ports and collapsed multi-topic counts.
- **Toggles**: `system` (`/rosout`, `/parameter_events`), `hide leaf`, `hide dead` (isolated nodes), and `group ns` (draggable namespace enclosures).

#### Node-RED-Style Canvas & Shortcuts
- **Drag & Snap**: Drag any node card (or namespace group header) freely; positions persist in browser `localStorage`. Hold `Shift` while dragging to snap to the 24px grid, or move a focused node with arrow keys.
- **Pan & Zoom**: Scroll wheel to zoom around cursor, drag empty canvas background to pan, or use the bottom-right toolbar (`−`, `100%`, `+`, `Fit`, `Layout` topological auto-arrange, `Map`).
- **Slide-in & Resizable Sidebar**: Click **Panel** (header), `»` (sidebar), `◀ Panel` (when collapsed), or press `Ctrl+Space` to slide the sidebar in/out. Drag the left border divider (`280px`–`680px`) to resize.
- **Shortcuts**:
  - `Ctrl+F` or `/`: Search nodes, topics, services, and actions
  - `Ctrl+Space`: Toggle inspector sidebar
  - `F`: Fit graph to viewport
  - `M`: Toggle minimap
  - `+` / `-` / `0`: Zoom in / out / reset 100%
- **Hover Lineage & Export**: Hover over any node or wire to highlight connected paths; export the current graph via **JSON** or **SVG** in the header.

#### Inspector Tabs
1. **Selection**:
   - **Node view**: Publishes/subscribes with QoS badges (`reliability`, `durability`, `liveliness`, `deadline`), services/clients, reconstructed actions, duplicate-node warnings, and a live **Parameters** editor (`rcl_interfaces/srv/*`).
   - **Wire / Topic view**: Publisher vs. subscriber QoS comparison table, `ok` / `warning` / `error` status, and **Monitor Live (Hz / Echo)** for real-time frequency, `KB/s`, and JSON message previews.
2. **Console**: Live `/rosout` log stream with severity filter (`ALL`, `INFO+`, `WARN+`, `ERROR+`), selected-node filter, pause, and clear.
3. **Health**: Actionable list of QoS mismatches, QoS warnings, and duplicate node name conflicts.

### Important Operational Notes
See [docs/cases.md](docs/cases.md) for detailed explanations of common ROS 2 behaviors:
- **Parameters update RAM, not disk**: Editing parameters in the UI updates the running node immediately in memory via `SetParameters`, not `.yaml` or `.launch.py` files on disk. Use `ros2 param dump <node> > params.yaml` to save tuned values.
- **QoS `history: UNKNOWN / depth: 0`**: Normal Fast-DDS discovery behavior (DDS propagates `reliability`, `durability`, `liveliness`, `deadline`, and `lifespan`).
- **Domain & stale CLI cache**: Ensure `ROS_DOMAIN_ID` / `ROS_LOCALHOST_ONLY` match your target robot/sim (`ROS_LOCALHOST_ONLY=1` by default on Mac VM). Run `ros2 daemon stop` if stale closed nodes linger in `ros2 node list`.

### Running Tests
```bash
python3 -m unittest -v
```
Inside a sourced ROS 2 workspace:
```bash
colcon build --symlink-install --packages-select mmr_ros_graph
colcon test --packages-select mmr_ros_graph --event-handlers console_direct+
```

---

## 🇮🇹 Documentazione in Italiano

Interfaccia browser live e strumento di diagnostica per grafi **ROS 2 Humble**, sviluppata per il team **More Modena Racing (MMR) E-Driverless**. I nodi vengono mostrati come blocchi trascinabili con porte di ingresso/uscita, i topic come collegamenti Bezier e la pagina si aggiorna automaticamente via Server-Sent Events (SSE) mentre i nodi vengono avviati, fermati o riconfigurati.

### Perché Usare Questo Strumento
- Evita i blocchi grafici di Qt/XQuartz su macOS con VM Linux (`ros2-mac`).
- Resta sempre aggiornato in tempo reale tramite la discovery DDS, senza dover ricaricare manualmente come su `rqt_graph`.
- Usa direttamente `rclpy.qos.qos_check_compatible` di ROS 2 Humble per evidenziare subito collegamenti incompatibili ("silent wires").
- Unisce visualizzazione del grafo, modifica parametri live, monitoraggio `Hz`/payload dei topic, servizi/azioni e log `/rosout` in un'unica app web senza dipendenze frontend esterne.

### Prerequisiti
- **ROS 2 Humble** con `rclpy`, `rcl_interfaces` e `rosidl_runtime_py` (inclusi in `ros-humble-desktop`).
- **Python 3.10+** (server HTTP/SSE basato sulla standard library; nessuna build Node/npm richiesta per l'esecuzione).

### Installazione e Compilazione nel Workspace

Clona il repository nella cartella `src/` del tuo workspace ROS 2 e compila con `colcon`:

```bash
cd ~/ros2_ws/src
git clone https://github.com/saishshinde15/MMR_E-Driverless-Node-Visualizer.git mmr-ros-graph
cd ..
colcon build --symlink-install --packages-select mmr_ros_graph
source install/setup.bash
```

*(Consiglio: usa `--symlink-install` così eventuali modifiche ai file statici HTML/CSS/JS o Python sono subito attive.)*

### Come Avviarlo

#### Ubuntu / Linux (oppure Windows con ambiente ROS 2 attivo)
```bash
ros2 run mmr_ros_graph live_graph
```
Oppure direttamente dalla cartella del progetto:
```bash
python3 live_graph        # Linux / macOS
py -3 live_graph          # Windows
```

#### macOS (ROS 2 dentro VM Linux / Colima)
Avvia il comando dal **terminale Mac** (non dentro `ros-shell`), così la pagina si apre nel browser di macOS mentre comunica tramite JSON su `stdin`/`stdout` con un singolo `live_graph worker` persistente nel container:
```bash
cd ~/ROS2/projects/mmr-ros-graph
python3 live_graph
```

Si aprirà `http://127.0.0.1:8765/` (se la porta `8765` è occupata viene scelta automaticamente un'altra porta libera). Premi `Ctrl-C` nel terminale per chiudere in modo pulito sia il server locale che il worker ROS 2 dentro il container, senza lasciare nodi orfani.

#### Opzioni e Sottocomandi CLI
| Opzione / Comando | Descrizione |
| --- | --- |
| `serve` *(predefinito)* | Avvia il server web locale e apre il grafo live nel browser |
| `snapshot` | Stampa uno snapshot JSON su stdout ed esce (`0` se senza errori) |
| `stream` | Stampa snapshot JSON su stdout ogni volta che il grafo cambia |
| `worker` | Nodo ROS 2 persistente con canale RPC bidirezionale su stdio |
| `--port 8765` | Porta HTTP preferita |
| `--period 1.0` | Frequenza di lettura/polling del grafo in secondi |
| `--no-browser` | Stampa l'indirizzo senza aprire automaticamente il browser |

### Funzionalità e Comandi dell'Interfaccia

#### Modalità di Visualizzazione e Filtri (equivalenti a `rqt_graph`)
- **Selettore `View`**:
  - `Nodes & Topics (all)`: Mostra tutti i nodi e tutti i topic, inclusi i *leaf topic* (topic con soli publisher o soli subscriber) mostrati come collegamenti tratteggiati cliccabili.
  - `Nodes & Topics (active)`: Mostra solo i topic attivi (con almeno 1 publisher e 1 subscriber) e i nodi collegati.
  - `Nodes only`: Mostra solo i collegamenti diretti tra nodi con porte singole e conteggio aggregato dei topic multipli.
- **Filtri rapidi**: `system` (`/rosout`, `/parameter_events`), `hide leaf`, `hide dead` (nodi isolati) e `group ns` (riquadri trascinabili per namespace).

#### Canvas Stile Node-RED e Scorciatoie
- **Trascinamento & Griglia**: Trascina liberamente qualsiasi nodo o intestazione di gruppo namespace (le posizioni vengono salvate in `localStorage`). Tieni premuto `Shift` mentre trascini per agganciare alla griglia da 24px, oppure sposta il nodo selezionato con le frecce della tastiera.
- **Pan & Zoom**: Usa la rotellina per lo zoom centrato sul cursore, trascina lo sfondo per spostarti (pan) oppure usa la barra in basso a destra (`−`, `100%`, `+`, `Fit`, `Layout` topologico automatico, `Map`).
- **Sidebar Scorrevole e Ridimensionabile**: Clicca **Panel** (in alto), `»` (sulla sidebar), `◀ Panel` (quando nascosta) oppure premi `Ctrl+Spazio` per aprire/chiudere il pannello laterale. Trascina il bordo sinistro (`280px`–`680px`) per ridimensionarlo.
- **Scorciatoie da Tastiera**:
  - `Ctrl+F` oppure `/`: Cerca nodi, topic, servizi e azioni
  - `Ctrl+Spazio`: Apri/chiudi pannello laterale
  - `F`: Adatta il grafo alla finestra (`Fit`)
  - `M`: Mostra/nascondi minimappa
  - `+` / `-` / `0`: Zoom avanti / indietro / reset 100%
- **Evidenziazione & Esportazione**: Passa il mouse sopra un nodo o un collegamento per evidenziare i percorsi collegati; esporta il grafo con **JSON** o **SVG** dalla barra superiore.

#### Schede del Pannello Laterale
1. **Selection**:
   - **Dettaglio Nodo**: Topic pubblicati/sottoscritti con badge QoS (`reliability`, `durability`, `liveliness`, `deadline`), servizi, client, azioni ricostruite, avvisi per nodi duplicati ed editor live dei **Parametri** (`rcl_interfaces/srv/*`).
   - **Dettaglio Wire / Topic**: Tabella comparativa QoS tra publisher e subscriber, stato `ok` / `warning` / `error` e pulsante **Monitor Live (Hz / Echo)** per vedere frequenza in Hz, banda in KB/s e anteprima JSON dei messaggi.
2. **Console**: Flusso live di `/rosout` con filtro di gravità (`ALL`, `INFO+`, `WARN+`, `ERROR+`), filtro per nodo selezionato, pausa e pulizia.
3. **Health**: Riepilogo cliccabile di incompatibilità QoS, warning QoS e nodi con nomi duplicati.

### Note Operative Importanti
Consulta [docs/cases.md](docs/cases.md) per i dettagli sui comportamenti più comuni di ROS 2:
- **I parametri modificati restano in RAM, non su disco**: Le modifiche dalla UI aggiornano subito il nodo in memoria tramite `SetParameters` (non riscrivono file `.yaml` o `.launch.py`). Per salvare i parametri tarati su file:
  ```bash
  ros2 param dump /nome_nodo > params.yaml
  ```
- **Campi `history: UNKNOWN / depth: 0`**: Comportamento normale della discovery DDS in ROS 2 Humble (Fast-DDS propaga `reliability`, `durability`, `liveliness`, `deadline` e `lifespan`).
- **Dominio ROS e cache CLI**: Verifica che `ROS_DOMAIN_ID` e `ROS_LOCALHOST_ONLY` corrispondano al veicolo/simulazione (`ROS_LOCALHOST_ONLY=1` di default nella VM Mac). Se `ros2 node list` mostra ancora nodi già chiusi, esegui `ros2 daemon stop`.

### Esecuzione dei Test
```bash
python3 -m unittest -v
```
In un workspace ROS 2 configurato:
```bash
colcon build --symlink-install --packages-select mmr_ros_graph
colcon test --packages-select mmr_ros_graph --event-handlers console_direct+
```
