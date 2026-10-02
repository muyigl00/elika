# Data and model selection

## Selected detector

The benchmark uses `models/detector/shahed_full_best.pt`, a YOLO26n model with
a P2/4 detection head. Only class `shahed` is enabled during inference. The
checkpoint was trained deterministically at image size 960 on Roboflow Universe
`shahed v5`, version 5, and copied into this project with a recorded SHA-256.

Public dataset page:
https://universe.roboflow.com/e-yjnj4/shahed-y4fsd/dataset/5

The page reports 18,301 generated images, a CC BY 4.0 license, and train/valid/test
splits of 16,029/1,521/751. Local training logs report best aggregate validation
metrics of mAP50 0.91087 and mAP50-95 0.75553.

## Small-object audit

The local v5 labels contain 6,245 boxes of class `shahed`. Measured at the
dataset's 1920x1080 export size, only six boxes have area at or below 1,024 px².
This means the public dataset is Shahed-specific but does not by itself establish
performance on very small, distant targets. Detection distances produced by this
benchmark are therefore simulation results for this camera, renderer, mesh,
orientation, threshold and model. They are not claims about real-world range.

## Other public checkpoint found

An MIT-licensed YOLO12m Shahed-136 checkpoint is published at:
https://huggingface.co/shng2025/EDTH-Warsaw-shahed136-detector

Its model card reports roughly 8,000 synthetic and real images at 640x640, but
does not provide an object-size distribution. It remains a useful independent
comparison model, but it was not selected as the benchmark default because the
local P2/4 checkpoint is explicitly structured for smaller features and has a
fully available local training record.

Another CC BY 4.0 single-class Roboflow project with 922 images exists at:
https://universe.roboflow.com/drone-jt2ap/shahed-136-drone_new

Its published page does not document a sufficient distant/small-object subset.

## Visual asset

The simulation mesh comes from Wikimedia Commons and is licensed CC BY 4.0:
https://commons.wikimedia.org/wiki/File:Shahed-136.stl

Attribution and checksum are stored in `models/shahed_136/SOURCE.md`.

## Camera profiles

The RGB tele profile uses 3840x2160 at 30 fps, a video mode published for the
DJI Zenmuse H30 zoom camera. That camera's zoom optics cover a much wider range
than this benchmark's configurable 6-20 degree sweep:
https://enterprise.dji.com/zenmuse-h30-series/specs

The thermal tele profile uses 640x512, 12 um detector pitch and 30 fps. These
are published operating values for Teledyne FLIR Boson+ modules. Boson+ lens
options include 6-30 degree continuous zoom and fixed 8, 12 and 18 degree HFOV
variants:
https://www.oem.flir.com/products/boson-plus/

Only geometry, resolution and cadence are represented. The generated thermal
colors are a deterministic transform of a Gazebo camera and do not model LWIR
radiometry, atmospheric attenuation, NETD, target temperature, lens MTF or
sensor noise. They must not be treated as a physical Boson+ performance model.
