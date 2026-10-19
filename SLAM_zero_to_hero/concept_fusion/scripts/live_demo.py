#!/usr/bin/env python3
"""ConceptFusion end to end in one pass, streamed to rerun frame by frame.

run_demo.sh runs the upstream stages one after another (all SAM masks, then all
OpenCLIP features, then all fusion), so nothing can be watched until the map is done.
This script runs the same three steps per frame instead:

  1. SAM automatic masks              (same generator settings as extract_conceptfusion_features.py)
  2. OpenCLIP pixel-aligned features  (same global/per-mask blending as extract_conceptfusion_features.py)
  3. gradslam PointFusion step        (same as run_feature_fusion_and_save_map.py, GT poses)

and logs to rerun on a "frame" timeline:

  world/camera/image/{rgb,sam,depth}   input frame, SAM masks, depth under the camera frustum
  frame/clip_pca                       pixel-aligned feature map, 1024-D -> RGB by PCA
  frame/query/<text>                   cosine similarity of that feature map to each text query
  world/map/{rgb,clip_pca}             the growing fused map, coloured by RGB / by feature PCA
  world/map/query/<text>               the growing map coloured by similarity to each text query
  world/trajectory                     GT camera path up to the current frame

The recording goes to <work>/concept_fusion_live.rrd and, if a viewer listens on
--rerun-url, live to the host. The final map is saved like run_feature_fusion_and_save_map.py
so text_query.py / query.sh can be run on it afterwards.
"""
import argparse
import os
import sys
import time
from pathlib import Path

import cv2
import matplotlib
import numpy as np
import open_clip
import rerun as rr
import rerun.blueprint as rrb
import torch
from gradslam.datasets import ICLDataset, load_dataset_config
from gradslam.slam.pointfusion import PointFusion
from gradslam.structures.pointclouds import Pointclouds
from gradslam.structures.rgbdimages import RGBDImages
from PIL import Image
from segment_anything import SamAutomaticMaskGenerator, sam_model_registry

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from text_query import load_gt_trajectory, rerun_reachable, slug  # noqa: E402

DATACONFIG = "/opt/concept-fusion/examples/dataconfigs/icl.yaml"
H, W = 120, 160  # feature map and fusion resolution, as in run_demo.sh


def pixel_aligned_features(img, imgfile, masks, model, preprocess):
    """extract_conceptfusion_features.py's per-frame feature computation, unchanged."""
    with torch.autocast("cuda"):
        global_feat = model.encode_image(preprocess(Image.open(imgfile)).unsqueeze(0).cuda())
        global_feat /= global_feat.norm(dim=-1, keepdim=True)
    global_feat = torch.nn.functional.normalize(global_feat.half(), dim=-1)  # 1, 1024
    feat_dim = global_feat.shape[-1]

    feat_per_roi, roi_nonzero_inds, similarity_scores = [], [], []
    for m in masks:
        _x, _y, _w, _h = tuple(m["bbox"])
        img_roi = preprocess(Image.fromarray(img[_y:_y + _h, _x:_x + _w, :])).unsqueeze(0).cuda()
        roifeat = torch.nn.functional.normalize(model.encode_image(img_roi), dim=-1)
        feat_per_roi.append(roifeat)
        roi_nonzero_inds.append(torch.argwhere(torch.from_numpy(m["segmentation"])))
        similarity_scores.append(torch.nn.functional.cosine_similarity(global_feat, roifeat, dim=-1))

    softmax_scores = torch.nn.functional.softmax(torch.cat(similarity_scores), dim=0)
    outfeat = torch.zeros(img.shape[0], img.shape[1], feat_dim, dtype=torch.half)
    for i in range(len(masks)):
        w = softmax_scores[i] * global_feat + (1 - softmax_scores[i]) * feat_per_roi[i]
        w = torch.nn.functional.normalize(w, dim=-1)
        r, c = roi_nonzero_inds[i][:, 0], roi_nonzero_inds[i][:, 1]
        outfeat[r, c] += w[0].detach().cpu().half()
        outfeat[r, c] = torch.nn.functional.normalize(outfeat[r, c].float(), dim=-1).half()

    outfeat = outfeat.unsqueeze(0).float().permute(0, 3, 1, 2)
    outfeat = torch.nn.functional.interpolate(outfeat, [H, W], mode="nearest")
    outfeat = torch.nn.functional.normalize(outfeat.permute(0, 2, 3, 1), dim=-1)
    return outfeat[0].half()  # H, W, 1024


class FeaturePCA:
    """1024-D -> RGB with a basis fitted once (on the first frame), so colours stay stable."""

    def __init__(self, feats):
        x = feats[feats.norm(dim=-1) > 0].float()
        self.mean = x.mean(0)
        _, _, v = torch.linalg.svd(x - self.mean, full_matrices=False)
        self.basis = v[:3].T
        p = (x - self.mean) @ self.basis
        self.lo, self.hi = torch.quantile(p, 0.01, dim=0), torch.quantile(p, 0.99, dim=0)

    def __call__(self, feats):
        valid = feats.norm(dim=-1, keepdim=True) > 0
        p = (feats.float() - self.mean) @ self.basis
        rgb = ((p - self.lo) / (self.hi - self.lo + 1e-12)).clamp(0, 1) * valid
        return (rgb.cpu().numpy() * 255).astype(np.uint8)


def heat_colors(sim, rgb, cmap, thresh, valid=None):
    """Same colouring as text_query.py: min-max relative similarity, jet above thresh, 50/50 with RGB.
    `valid` excludes pixels no SAM mask covered (zero feature) from the min-max range."""
    s = sim[valid] if valid is not None else sim
    rel = (sim - s.min()) / (s.max() - s.min() + 1e-12)
    rel = rel.cpu().numpy()
    rel[rel < thresh] = 0.0
    if valid is not None:
        rel[~valid.cpu().numpy()] = 0.0
    return (0.5 * rgb + 0.5 * cmap(rel)[..., :3]).clip(0, 1)


def blueprint(queries, traj):
    c = traj.mean(0)
    dist = max(6.0, 2.0 * float(np.linalg.norm(traj.max(0) - traj.min(0))))
    eye = rrb.EyeControls3D(kind=rrb.Eye3DKind.Orbital, position=c + np.array([0.0, 0.75, -0.66]) * dist,
                            look_target=c, eye_up=[0.0, 1.0, 0.0])
    cam = ["/world/trajectory", "/world/camera/image"]

    def view3d(name, layer, extra=()):
        return rrb.Spatial3DView(name=name, origin="/world", contents=[layer, *cam, *extra], eye_controls=eye)

    return rrb.Blueprint(rrb.Horizontal(
        view3d("Reconstruction (RGB)", "/world/map/rgb", ["/world/camera/image/rgb"]),
        rrb.Tabs(view3d("Fused CLIP features (PCA)", "/world/map/clip_pca"),
                 *[view3d(f'Map: "{q}"', f"/world/map/query/{slug(q)}") for q in queries]),
        rrb.Vertical(
            rrb.Spatial2DView(name="RGB + SAM masks", origin="/world/camera/image",
                              contents=["/world/camera/image/rgb", "/world/camera/image/sam"]),
            rrb.Spatial2DView(name="Depth", origin="/world/camera/image", contents=["/world/camera/image/depth"]),
            rrb.Spatial2DView(name="Pixel-aligned CLIP (PCA)", origin="/frame", contents=["/frame/clip_pca"]),
            rrb.Tabs(*[rrb.Spatial2DView(name=f'Frame: "{q}"', origin="/frame", contents=[f"/frame/query/{slug(q)}"])
                       for q in queries]),
        ),
        column_shares=[2, 2, 1],
    ), collapse_panels=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default="/data")
    ap.add_argument("--sequence", default="living_room_traj2_frei_png")
    ap.add_argument("--frame-end", type=int, default=880)
    ap.add_argument("--stride", type=int, default=20)
    ap.add_argument("--sam-ckpt", default=os.environ.get("SAM_CKPT", "/weights/sam_vit_h_4b8939.pth"))
    ap.add_argument("--queries", nargs="+", default=["sofa", "table"])
    ap.add_argument("--similarity-thresh", type=float, default=0.6)
    ap.add_argument("--work-dir", default="/weights/run_live", help=".rrd and saved-map")
    ap.add_argument("--rerun-url", default="rerun+http://127.0.0.1:9876/proxy")
    ap.add_argument("--no-stream", action="store_true")
    args = ap.parse_args()

    out = Path(args.work_dir)
    out.mkdir(parents=True, exist_ok=True)
    torch.autograd.set_grad_enabled(False)
    cmap = matplotlib.colormaps["jet"]

    cfg = load_dataset_config(DATACONFIG)
    cam = cfg["camera_params"]
    dataset = ICLDataset(cfg, args.data_dir, args.sequence, start=0, end=args.frame_end, stride=args.stride,
                         desired_height=H, desired_width=W)
    traj_all, _ = load_gt_trajectory(args.data_dir, args.sequence, args.frame_end, args.stride)
    print(f"{args.sequence}: {len(dataset)} frames (stride {args.stride}), GT path {len(traj_all)} poses")

    rr.init("concept_fusion_live", recording_id="concept_fusion_live")
    sinks = [rr.FileSink(str(out / "concept_fusion_live.rrd"))]
    if not args.no_stream and rerun_reachable(args.rerun_url):
        sinks.append(rr.GrpcSink(args.rerun_url))
        print(f"Streaming to rerun viewer at {args.rerun_url}")
    rr.set_sinks(*sinks)
    rr.send_blueprint(blueprint(args.queries, traj_all))
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Y_UP, static=True)
    # icl.yaml has fy < 0: gradslam back-projects pixel rows upwards, i.e. camera axes are
    # x right, y up, z forward. A pinhole with |fy| and RUF axes projects the same way.
    rr.log("world/camera/image", rr.Pinhole(
        focal_length=[cam["fx"], abs(cam["fy"])], principal_point=[cam["cx"], cam["cy"]],
        resolution=[cam["image_width"], cam["image_height"]],
        camera_xyz=rr.ViewCoordinates.RUF, image_plane_distance=0.4), static=True)

    t0 = time.time()
    sam = sam_model_registry["vit_h"](checkpoint=args.sam_ckpt).cuda()
    mask_generator = SamAutomaticMaskGenerator(model=sam, points_per_side=8, pred_iou_thresh=0.92,
                                               crop_n_layers=1, crop_n_points_downscale_factor=2)
    model, _, preprocess = open_clip.create_model_and_transforms("ViT-H-14", "laion2b_s32b_b79k")
    model = model.cuda().eval()
    tokenizer = open_clip.get_tokenizer("ViT-H-14")
    tfeats = torch.nn.functional.normalize(model.encode_text(tokenizer(args.queries).cuda()).float(), dim=-1)
    print(f"SAM ViT-H + OpenCLIP ViT-H-14 loaded in {time.time() - t0:.1f} s")

    slam = PointFusion(odom="gt", dsratio=1, device="cuda", use_embeddings=True)
    pointclouds = Pointclouds(device="cuda")
    pca = None
    t_sam = t_clip = t_fuse = 0.0
    t_start = time.time()

    for idx in range(len(dataset)):
        imgfile = dataset.color_paths[idx]
        fid = int(Path(imgfile).stem)
        rr.set_time("frame", sequence=fid)

        img = cv2.cvtColor(cv2.imread(imgfile), cv2.COLOR_BGR2RGB)
        t = time.time()
        masks = mask_generator.generate(img)
        t_sam += time.time() - t
        t = time.time()
        feat = pixel_aligned_features(img, imgfile, masks, model, preprocess).cuda()  # H, W, 1024
        torch.cuda.synchronize()
        t_clip += time.time() - t

        _color, _depth, intrinsics, _pose, *_ = dataset[idx]
        t = time.time()
        frame = RGBDImages(_color[None, None], _depth[None, None], intrinsics[None, None], _pose[None, None],
                           embeddings=feat[None, None])
        pointclouds, _ = slam.step(pointclouds, frame, None)
        torch.cuda.synchronize()
        t_fuse += time.time() - t

        # Camera, input frame, SAM masks (largest painted first so small parts stay visible)
        pose = _pose.cpu().numpy()
        rr.log("world/camera", rr.Transform3D(translation=pose[:3, 3], mat3x3=pose[:3, :3]))
        rr.log("world/trajectory", rr.LineStrips3D([traj_all[:fid + 1]], colors=[[0, 0, 0]], radii=0.01))
        rr.log("world/camera/image/rgb", rr.Image(img))
        seg = np.zeros(img.shape[:2], np.uint16)
        for k, m in enumerate(sorted(masks, key=lambda m: -m["area"])):
            seg[m["segmentation"]] = k + 1
        rr.log("world/camera/image/sam", rr.SegmentationImage(seg, opacity=0.5))
        depth_raw = cv2.imread(dataset.depth_paths[idx], cv2.IMREAD_UNCHANGED)
        rr.log("world/camera/image/depth", rr.DepthImage(depth_raw, meter=cfg["camera_params"]["png_depth_scale"]))

        # Pixel-aligned features: PCA colours and per-query similarity
        if pca is None:
            pca = FeaturePCA(feat)
        rr.log("frame/clip_pca", rr.Image(pca(feat)))
        small = cv2.resize(img, (W, H), interpolation=cv2.INTER_AREA) / 255.0
        fn = torch.nn.functional.normalize(feat.float(), dim=-1)
        covered = feat.norm(dim=-1) > 0
        for q, tf in zip(args.queries, tfeats):
            heat = heat_colors(fn @ tf, small, cmap, args.similarity_thresh, covered)
            rr.log(f"frame/query/{slug(q)}", rr.Image((heat * 255).astype(np.uint8)))

        # The fused map so far
        pts = pointclouds.points_padded[0].float().cpu().numpy()
        rgb = pointclouds.colors_padded[0].float().cpu().numpy()
        rgb = rgb / 255.0 if rgb.max() > 1.0 else rgb
        emb = torch.nn.functional.normalize(pointclouds.embeddings_padded[0].float(), dim=-1)
        rr.log("world/map/rgb", rr.Points3D(pts, colors=rgb, radii=0.01))
        rr.log("world/map/clip_pca", rr.Points3D(pts, colors=pca(emb), radii=0.01))
        for q, tf in zip(args.queries, tfeats):
            rr.log(f"world/map/query/{slug(q)}",
                   rr.Points3D(pts, colors=heat_colors(emb @ tf, rgb, cmap, args.similarity_thresh), radii=0.01))

        print(f"frame {fid:4d} ({idx + 1}/{len(dataset)}): {len(masks):3d} SAM masks, map {pts.shape[0]:7d} pts, "
              f"{time.time() - t_start:6.1f} s")
        torch.cuda.empty_cache()

    os.makedirs(out / "saved-map", exist_ok=True)
    pointclouds.save_to_h5(str(out / "saved-map"))
    n = len(dataset)
    print(f"Map: {pointclouds.points_padded.shape[1]} points -> {out / 'saved-map'}")
    print(f"Timing per frame: SAM {t_sam / n:.2f} s, OpenCLIP {t_clip / n:.2f} s, fusion {t_fuse / n:.2f} s; "
          f"total {time.time() - t_start:.0f} s for {n} frames")
    print(f"Recording: {out / 'concept_fusion_live.rrd'}")


if __name__ == "__main__":
    main()
