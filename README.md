# Portabler Shahed Detection Benchmark

Diese Kopie laeuft unabhaengig vom Cluster-Projekt. ROS 2 Jazzy, Gazebo, das
YOLO-Modell, das 3D-Modell und eine getestete CPU-Python-Laufzeit liegen bereits
im Ordner. Es werden weder Conda noch ROS, Gazebo oder Python-Pakete auf dem
Host benoetigt.

## Voraussetzungen

- Linux auf x86-64/amd64; unter Windows geht auch WSL2 mit Ubuntu
- Apptainer 1.5 oder neuer
- rund 3 GB freier Speicher fuer Paket, Cache und einen kurzen Testlauf
- fuer den kompletten 10-km-Lauf deutlich mehr Speicher fuer die Videos
- CPU-Betrieb: mindestens 16 GB RAM empfohlen
- optional: NVIDIA-GPU mit aktuellem Treiber und zusaetzlichem Speicherplatz

Ubuntu-Nutzer installieren Apptainer aus dem offiziellen PPA:

```bash
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository -y ppa:apptainer/ppa
sudo apt update
sudo apt install -y apptainer
```

Offizielle Anleitung (auch fuer Fedora, Debian, WSL2 und Installationen ohne
Root-Rechte): <https://apptainer.org/docs/admin/main/installation.html>

## Kopieren und erster Test

Den **gesamten Ordner** auf den anderen PC kopieren, dann im Terminal:

```bash
cd shahed_detection_benchmark_portable
bash scripts/verify.sh
```

Die Pruefung benoetigt kein Internet. Sie testet Python-Pakete, Modell,
Checksummen, ROS/Gazebo-Helfer und die Konfiguration, startet aber noch keinen
langen Benchmark.

## Einfach starten

Zuerst den kurzen 300-m-Test auf der CPU ausfuehren:

```bash
bash run.sh --smoke
```

Der normale Benchmark startet mit:

```bash
bash run.sh
```

Eine andere mitgelieferte Konfiguration kann als erstes Argument stehen:

```bash
bash run.sh configs/long_range_tele.yaml --smoke
```

Weitere Beispiele:

```bash
bash run.sh --start-distance-m 1000 --speeds-kmh 600 --fovs-deg 6 8 12 18 20
SHAHED_DEVICE=cpu SIM_CPU_THREADS=8 bash run.sh --smoke
```

Jeder Lauf schreibt in einen neuen Ordner unter `recordings/`. Es gibt keine
Slurm-Abhaengigkeit und keine fest eingebaute Pfadangabe zum urspruenglichen PC.

## NVIDIA-GPU optional einrichten

Die portable Grundversion nutzt bewusst die CPU und funktioniert offline. Fuer
einen schnellen 4K-Benchmark ist eine NVIDIA-GPU sehr empfehlenswert. Einmalig
mit Internetzugang:

```bash
bash scripts/setup_gpu.sh
```

Dabei werden die offiziellen PyTorch-Pakete fuer CUDA 12.8 in
`runtime/python-gpu/` installiert. Der Download ist mehrere GB gross. Danach:

```bash
SHAHED_USE_GPU=1 bash run.sh --smoke
SHAHED_USE_GPU=1 bash run.sh
```

Unter WSL2 werden zusaetzlich ein Windows-NVIDIA-Treiber mit WSL-Unterstuetzung
und `libnvidia-container-tools` benoetigt. Das Startskript erkennt WSL2 und
verwendet dort Apptainers `--nvccli`-Modus.

## Ausgabe

Jeder Geschwindigkeitsfall enthaelt unter anderem:

- `result.json`, `frames.jsonl` und `camera.json`
- RGB- und Pseudo-Thermal-Video mit YOLO-Markierungen
- Top-down-Positionsvideo und finales Radarbild
- Rohbild und markiertes Bild der ersten Detektion
- die erste bestaetigte Folge aus drei Detektionen

Pseudo-Thermal ist eine deterministische Bildtransformation und keine
radiometrisch korrekte IR-Simulation. Ein in der Simulation gemessener Bereich
ist keine unabhaengige Aussage ueber reale Erkennungsreichweite.

## Fehlerdiagnose

- `Apptainer fehlt`: Apptainer wie oben installieren.
- `GPU-Pakete fehlen`: `bash scripts/setup_gpu.sh` ausfuehren oder CPU nutzen.
- `nvidia-smi fehlt`: NVIDIA-Treiber auf dem Host installieren/aktualisieren.
- `Gazebo startup failed`: die Datei `gazebo.log` im betreffenden
  `recordings/run_.../speed_.../`-Ordner ansehen.
- Ein abgebrochener Lauf ueberschreibt keine alten Ergebnisse; jeder Start
  bekommt einen neuen Zeitstempel.

Die Quellen und Einschraenkungen der Daten stehen in `DATA_SOURCES.md`.
