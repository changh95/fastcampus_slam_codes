# ConceptFusion

Open-set 3D mapping: every map point carries a 1024-D OpenCLIP embedding, so the map can
be queried with free text ("sofa", "table") after it is built. Per frame, SAM proposes
masks, OpenCLIP embeds each mask crop and the whole image, and the two are blended into a
**pixel-aligned feature map**; gradslam's PointFusion then fuses those features into a 3D
point cloud using known camera poses. This demo runs it on **ICL-NUIM living room 2**.

- **Repo**: [concept-fusion/concept-fusion](https://github.com/concept-fusion/concept-fusion) (`4457c1f`) + [gradslam](https://github.com/gradslam/gradslam) branch `conceptfusion` (`59ca872`)
- **Paper**: [ConceptFusion: Open-set Multimodal 3D Mapping](https://arxiv.org/abs/2302.07241) — Jatavallabhula et al., RSS 2023
- Models: [Segment Anything](https://github.com/facebookresearch/segment-anything) ViT-H, [OpenCLIP](https://github.com/mlfoundations/open_clip) ViT-H-14 `laion2b_s32b_b79k`
- **Dataset**: [ICL-NUIM](https://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html) — Handa, Whelan, McDonald and Davison, ICRA 2014

![ConceptFusion on ICL-NUIM living room 2: RGB map, "sofa" and "table" queries](results/overview.png)

Left: the fused RGB point cloud with the ground-truth camera path (black). Middle and
right: the same map coloured by similarity to the text query (jet, blended 50/50 with RGB,
points below 0.6 relative similarity left blue). "sofa" lights up both sofas; "table"
lights up the coffee table.

## Build

```bash
podman build -t localhost/slam_zero_to_hero:concept_fusion .
```

CUDA 12.8.1 + PyTorch 2.7.1 cu128 (runs on RTX 50xx / sm_120). ConceptFusion and gradslam
are pure Python; no extension is compiled.

## Download the dataset and the weights

```bash
mkdir -p ~/data/icl_nuim/living_room_traj2_frei_png && cd ~/data/icl_nuim/living_room_traj2_frei_png
curl -fL http://www.doc.ic.ac.uk/~ahanda/living_room_traj2_frei_png.tar.gz | tar -xz
curl -fLO https://www.doc.ic.ac.uk/~ahanda/VaFRIC/livingRoom2n.gt.sim
cd -
./scripts/download_weights.sh          # SAM ViT-H 2.4 GB + OpenCLIP ViT-H-14 3.9 GB -> ~/data/concept_fusion
```

The sequence folder must hold `rgb/`, `depth/` (881 frames, 463 MB) and the
`livingRoom2n.gt.sim` pose file, which gradslam's ICL loader requires and the archive does
not include. `python3 ../download_icl_nuim.py` (default `living_room_traj2_frei_png`) extracts
each sequence into its own folder. Do **not** use the flat `~/data/icl_nuim/rgb` folder an older
version of that script left behind: it is a mix of trajectories (see [NOTES.md](NOTES.md)).

## Run

```bash
podman run --rm --network=host \
  --runtime=/usr/bin/nvidia-container-runtime \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
  -v ~/data/icl_nuim:/data:ro -v ~/data/concept_fusion:/weights \
  -v "$PWD/results":/out \
  localhost/slam_zero_to_hero:concept_fusion
```

`scripts/run_demo.sh` (the image's default command) runs the three upstream stages on
every 20th frame of frames 0–879 (44 frames):

1. `extract_conceptfusion_features.py` — SAM masks + OpenCLIP pixel-aligned features, 160×120
2. `run_feature_fusion_and_save_map.py` — gradslam PointFusion with ground-truth poses
3. `scripts/text_query.py` — headless version of upstream `demo_text_query.py`: scores
   `sofa` and `table`, renders the maps with Open3D under Xvfb, writes a rerun recording
   and streams it to a rerun viewer on the host if one is listening on port 9876

PNGs and `query_summary.json` land in `results/`; the features (~2 GB), the map (~1 GB),
the coloured PLYs and `concept_fusion.rrd` go to `~/data/concept_fusion/run/`.
Change the queries with `-e QUERIES="lamp chair window"`, the subsampling with
`-e STRIDE=10`, or open the recording later with `rerun ~/data/concept_fusion/run/concept_fusion.rrd`.

Query an existing map without re-extracting features (the query stage took 12 s in the full run):

```bash
podman run --rm --network=host \
  --runtime=/usr/bin/nvidia-container-runtime \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
  -v ~/data/icl_nuim:/data:ro -v ~/data/concept_fusion:/weights \
  -v "$PWD/results":/out \
  localhost/slam_zero_to_hero:concept_fusion \
  /opt/cf_tools/query.sh \
    --load-path /weights/run/saved-map --out-dir /out --work-dir /weights/run \
    --feat-dir /weights/run/saved-feat --data-dir /data --queries sofa table
```

## View in a GUI

**Open the saved recording.** After a run, open the recording on the host. It shows one 3D
view per layer: the RGB map with the GT path, then one heatmap per query.

```bash
rerun ~/data/concept_fusion/run/concept_fusion.rrd
```

**Stream live.** Start the viewer on the host first. It listens on port 9876. Then run the
full demo or the query-only command above. `text_query.py` detects the viewer and streams to
it ("Streaming to rerun viewer at ..."). If no viewer is listening, it only writes the `.rrd`.

```bash
rerun &
# then the query-only podman run above, e.g. with --queries lamp chair window
```

**Watch the scan and the reconstruction (end to end).** The two commands above only show
the finished map. `run_demo.sh` runs the stages in sequence: all SAM masks, then all OpenCLIP
features, then all fusion. Nothing exists to show until the end. `scripts/live_demo.py`
runs the same three steps one frame at a time and logs each frame to rerun:

```bash
rerun &
podman run --rm --network=host \
  --runtime=/usr/bin/nvidia-container-runtime \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
  -v ~/data/icl_nuim:/data:ro -v ~/data/concept_fusion:/weights \
  localhost/slam_zero_to_hero:concept_fusion \
  python /opt/cf_tools/live_demo.py --queries sofa table
```

![rerun, live_demo.py at frame 420: growing map, fused CLIP features, SAM masks, depth, per-frame "sofa" query](results/rerun_live.png)

| View | What it shows |
|---|---|
| Reconstruction (RGB) | The fused point cloud growing frame by frame, the camera frustum with the current image, and the GT path so far |
| Fused CLIP features (PCA) / Map: "&lt;query&gt;" | The same map coloured by its 1024-D embeddings (3-component PCA) or by similarity to each query |
| RGB + SAM masks | The input frame with SAM's automatic masks overlaid |
| Depth | The input depth |
| Pixel-aligned CLIP (PCA) | The frame's feature map at 160×120. Each SAM region gets a single feature, so the regions show as flat colours |
| Frame: "&lt;query&gt;" | That feature map scored against each query. Similarity is min-max normalised per frame, so a frame without a sofa still shows its "most sofa-like" region |

The PCA basis is fitted on the first frame and then kept fixed, so colours mean the same
thing on every frame. The run is not real-time. Each frame takes about 1.9 s for SAM and
1.7 s for OpenCLIP, which is 159 s for the 44 frames. To review a run, scrub the `frame`
timeline afterwards, or open `~/data/concept_fusion/run_live/concept_fusion_live.rrd` (390 MB:
the map is logged every frame). The final map, `run_live/saved-map`, is the same
258,122-point map as the staged run. To query it, pass `--load-path /weights/run_live/saved-map`
to `query.sh`. Other options: `--stride`, `--frame-end`, `--sequence`, `--work-dir`,
`--no-stream`.

**Interactive upstream demo (not tested).** In upstream `demo_text_query.py`, you type a
query or click a point in an Open3D window. It needs X11 forwarding:

```bash
xhost +local:
podman run --rm -it --network=host \
  --runtime=/usr/bin/nvidia-container-runtime \
  -e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/data/icl_nuim:/data:ro -v ~/data/concept_fusion:/weights \
  localhost/slam_zero_to_hero:concept_fusion \
  python demo_text_query.py --load-path /weights/run/saved-map
```

The headless `query.sh` path above is the one that was verified.

## Results

ICL-NUIM `living_room_traj2_frei_png`, 44 frames, RTX 5090 (runtimes vary ±50 % with other GPU jobs on the machine):

| | |
|---|---|
| Fused map | **258,122 points**, 1024-D embeddings, 5.1 × 2.7 × 6.3 m |
| GT camera path | 880 poses, 8.42 m |
| Runtime | features 175 s (SAM masks 1.8 s/frame, OpenCLIP per-mask features 1.6 s/frame, plus model loading), fusion 9 s, query + render 12 s — **196 s** total |
| "sofa" | raw cosine −0.01…0.36; 23.3 % of points ≥ 0.6 relative similarity |
| "table" | raw cosine 0.00…0.37; 17.9 % of points ≥ 0.6 relative similarity |

Poses are ground truth: ConceptFusion is a mapping method, so there is no trajectory to
score. The same query on a single frame's pixel-aligned features, before any fusion:

![Frame 0, cosine similarity to "sofa"](results/query_2d_sofa.png)

Rerun view of the recording (one 3D view per layer):

![rerun: RGB map, sofa, table](results/rerun_view.png)

## Supported datasets

| Dataset | Command | Status |
|---|---|---|
| **ICL-NUIM living room 2** (`living_room_traj2_frei_png`) | default (above) | ✅ verified: 258k-point map, "sofa" / "table" queries localise the right furniture |
| Other ICL-NUIM living rooms (`living_room_traj{0,1,3}_frei_png`) | `-e SEQ=living_room_traj1_frei_png -e END=<poses in its .gt.sim>` + that sequence's `livingRoom<k>n.gt.sim` in its folder | not run |
| ScanNet, Replica | upstream `dataconfigs/scannet/*.yaml`, `dataconfigs/replica/*.yaml` via `--dataconfig-path` | not run (ScanNet needs institutional access) |
