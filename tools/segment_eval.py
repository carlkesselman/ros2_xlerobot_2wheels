#!/usr/bin/env python3
"""Experiment 2/3 of ADR-0005: the cube pose from a learned detector and mask instead of the yellow threshold.

Scores saved frames of tools/cube_pose_eval.py (laptop, GPU, no ROS):

  ~/venvs/xlerobot-perception/bin/python tools/segment_eval.py ~/pose_eval/cube_<time> [--step 4] [--prompt "a cube"]

Only the mask source changes. The sheet pose, the camera model and the silhouette fit are the ones of
cube_pose_eval, so the numbers compare directly with its table. Backends: `yellow` (the baseline) and
`owlv2+sam2` (open-vocabulary boxes on the whole frame, then a SAM 2 mask from a crop around the best box).
"""
import argparse
import glob
import json
import math
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import cube_pose_eval as cpe  # noqa: E402

OWL = "google/owlv2-base-patch16-ensemble"
SAM = "facebook/sam2.1-hiera-small"
# The Gemini 2 colour lens as /cameras/head/camera_info reports it; sessions recorded before meta.json carried D used it.
FACTORY_D = [53.997039794921875, -110.29363250732422, -0.000273539568297565, 0.00021029447088949382,
             123.43366241455078, 53.815738677978516, -109.88721466064453, 123.76702117919922]


class Detector:
    """Open-vocabulary boxes (OWLv2). Returns [(score, x0, y0, x1, y1)] in image pixels, best first."""

    def __init__(self, prompt):
        import torch
        from transformers import Owlv2ForObjectDetection, Owlv2Processor
        self.torch, self.prompt = torch, prompt
        self.proc = Owlv2Processor.from_pretrained(OWL)
        self.model = Owlv2ForObjectDetection.from_pretrained(OWL).cuda().half().eval()

    def __call__(self, img, region=None, thr=0.1):
        torch = self.torch
        x0, y0, x1, y1 = region if region else (0, 0, img.shape[1], img.shape[0])
        crop = cv2.cvtColor(img[y0:y1, x0:x1], cv2.COLOR_BGR2RGB)
        side = max(crop.shape[:2])                                   # OWLv2 pads to a square, so post-process on that size
        inp = self.proc(text=[[self.prompt]], images=crop, return_tensors="pt").to("cuda")
        inp["pixel_values"] = inp["pixel_values"].half()
        with torch.no_grad():
            out = self.model(**inp)
        res = self.proc.post_process_grounded_object_detection(out, threshold=thr, target_sizes=[(side, side)])[0]
        boxes = [(float(s), float(b[0]) + x0, float(b[1]) + y0, float(b[2]) + x0, float(b[3]) + y0)
                 for s, b in zip(res["scores"].cpu(), res["boxes"].cpu())]
        return sorted(boxes, reverse=True)


class YoloWorld:
    """YOLO-World (CNN, open vocabulary), the lightweight candidate."""

    def __init__(self, prompt, weights="yolov8s-worldv2.pt"):
        from ultralytics import YOLOWorld
        self.model = YOLOWorld(os.path.expanduser(f"~/.cache/xlerobot/{weights}"))
        self.model.set_classes([prompt])

    def __call__(self, img, region=None, thr=0.05):
        x0, y0, x1, y1 = region if region else (0, 0, img.shape[1], img.shape[0])
        r = self.model.predict(img[y0:y1, x0:x1], imgsz=1280, conf=thr, verbose=False)[0]
        return sorted(((float(c), b[0] + x0, b[1] + y0, b[2] + x0, b[3] + y0)
                       for c, b in zip(r.boxes.conf.cpu().numpy(), r.boxes.xyxy.cpu().numpy())), reverse=True)


class GroundingDino:
    """Grounding DINO tiny, the offline reference."""

    def __init__(self, prompt, model="IDEA-Research/grounding-dino-tiny"):
        import torch
        from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor
        self.torch, self.prompt = torch, prompt.rstrip(".") + "."
        self.proc = AutoProcessor.from_pretrained(model)
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(model).cuda().eval()

    def __call__(self, img, region=None, thr=0.1):
        x0, y0, x1, y1 = region if region else (0, 0, img.shape[1], img.shape[0])
        crop = cv2.cvtColor(img[y0:y1, x0:x1], cv2.COLOR_BGR2RGB)
        inp = self.proc(images=crop, text=self.prompt, return_tensors="pt").to("cuda")
        with self.torch.no_grad():
            out = self.model(**inp)
        res = self.proc.post_process_grounded_object_detection(out, inp.input_ids, threshold=thr, text_threshold=thr,
                                                               target_sizes=[crop.shape[:2]])[0]
        return sorted(((float(s), float(b[0]) + x0, float(b[1]) + y0, float(b[2]) + x0, float(b[3]) + y0)
                       for s, b in zip(res["scores"].cpu(), res["boxes"].cpu())), reverse=True)


DETECTORS = {"owlv2": Detector, "yoloworld": YoloWorld, "gdino": GroundingDino}


class Segmenter:
    """SAM 2 mask prompted by a box on an upscaled crop (the model's masks are low resolution)."""

    def __init__(self):
        import torch
        from transformers import Sam2Model, Sam2Processor
        self.torch = torch
        self.proc = Sam2Processor.from_pretrained(SAM)
        self.model = Sam2Model.from_pretrained(SAM).cuda().eval()

    def __call__(self, img, box, ctx=3.0, up=512):
        torch = self.torch
        _, bx0, by0, bx1, by1 = box
        cx, cy, side = (bx0 + bx1) / 2, (by0 + by1) / 2, max(bx1 - bx0, by1 - by0, 24) * ctx
        x0, y0 = int(round(cx - side / 2)), int(round(cy - side / 2))
        n = int(round(side))
        pad = cv2.copyMakeBorder(img, n, n, n, n, cv2.BORDER_REPLICATE)
        crop = pad[y0 + n:y0 + n + n, x0 + n:x0 + n + n]
        big = cv2.resize(crop, (up, up), interpolation=cv2.INTER_CUBIC)
        k = up / n
        b = [[[(bx0 - x0) * k, (by0 - y0) * k, (bx1 - x0) * k, (by1 - y0) * k]]]
        inp = self.proc(images=cv2.cvtColor(big, cv2.COLOR_BGR2RGB), input_boxes=b, return_tensors="pt").to("cuda")
        with torch.no_grad():
            out = self.model(**inp, multimask_output=False)
        m = self.proc.post_process_masks(out.pred_masks.cpu(), inp["original_sizes"])[0][0, 0].numpy().astype(np.float32)
        small = cv2.resize(m, (n, n), interpolation=cv2.INTER_AREA)       # area average = coverage per native pixel
        full = np.zeros(img.shape[:2], np.float32)
        ys, xs = max(y0, 0), max(x0, 0)
        ye, xe = min(y0 + n, img.shape[0]), min(x0 + n, img.shape[1])
        full[ys:ye, xs:xe] = small[ys - y0:ye - y0, xs - x0:xe - x0]
        return full > 0.5


def session_frames(sess, step):
    for d in sorted(glob.glob(os.path.join(sess, "*/"))):
        meta_p = os.path.join(d, "meta.json")
        if not os.path.exists(meta_p):
            continue
        meta = json.load(open(meta_p))
        for k, fr in enumerate(meta["frames"]):
            if k % step == 0:
                yield meta, fr, cv2.imread(os.path.join(d, f"color_{fr['index']:02d}.png"))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("--step", type=int, default=4, help="score every Nth saved frame")
    ap.add_argument("--prompt", default="a cube")
    ap.add_argument("--detector", choices=sorted(DETECTORS), default="owlv2")
    ap.add_argument("--table-crop", action="store_true", help="run the detector on the sheet's region instead of the whole frame")
    a = ap.parse_args()
    det, seg = DETECTORS[a.detector](a.prompt), Segmenter()
    name = f"{a.detector}+sam2"
    rows = {}
    for meta, fr, img in session_frames(a.session, a.step):
        fp, K, T = meta["footprint"], np.array(meta["K"]), np.array(fr["T_camera_from_sheet"])
        D = np.array(meta["D"] if "D" in meta else FACTORY_D)
        truth = np.array(fp["center_mm"]) * meta["scale"]
        z = cpe.project(T, K, [(truth[0], truth[1], cpe.CUBE_MM / 2)], D)[0]
        region = None
        if a.table_crop:
            sh = json.load(open(os.path.expanduser(meta["sheet_file"]))) if os.path.exists(os.path.expanduser(meta["sheet_file"])) else None
            w, h = (sh["size_mm"] if sh else (262, 197))
            c = cpe.project(T, K, [(sx * w / 2 * meta["scale"], sy * h / 2 * meta["scale"], 0) for sx in (-1, 1) for sy in (-1, 1)], D)
            region = (int(max(c[:, 0].min(), 0)), int(max(c[:, 1].min(), 0)), int(min(c[:, 0].max(), img.shape[1])), int(min(c[:, 1].max(), img.shape[0])))
        t0 = time.time()
        boxes = det(img, region)
        t_det = time.time() - t0
        row = rows.setdefault(fp["id"], {"frames": 0, "found": 0, "extra": 0, "est": {"yellow": [], name: []}, "t": []})
        row["frames"] += 1
        # baseline on the same frame
        ym = cpe.yellow_mask(img)
        if ym is not None:
            near = np.zeros_like(ym)
            near[int(max(z[1] - 90, 0)):int(z[1] + 90), int(max(z[0] - 90, 0)):int(z[0] + 90)] = True
            r = cpe.fit_cube(ym & near, T, K, truth, D)
            if r:
                row["est"]["yellow"].append(r)
        hit = [b for b in boxes if b[1] <= z[0] <= b[3] and b[2] <= z[1] <= b[4]]
        row["extra"] += len([b for b in boxes if b not in hit and b[0] > 0.2])
        if not hit:
            continue
        row["found"] += 1
        t0 = time.time()
        m = seg(img, hit[0])
        row["t"].append((t_det, time.time() - t0))
        r = cpe.fit_cube(m, T, K, truth, D)
        if r:
            row["est"][name].append(r)
    print(f"{'footprint':9s} {'recall':>7s} {'extra':>5s} | " + " | ".join(f"{k:^30s}" for k in ("yellow", name)))
    allr = {"yellow": [], name: []}
    for fid, row in sorted(rows.items()):
        mt = json.load(open(os.path.join(a.session, fid, "meta.json")))
        truth, scale, yaw0 = np.array(mt["footprint"]["center_mm"]), mt["scale"], mt["footprint"]["yaw_deg"]
        cells = []
        for k in ("yellow", name):
            e = np.array(row["est"][k])
            if len(e) == 0:
                cells.append(f"{'-':^30s}")
                continue
            dx, dy = e[:, 0].mean() - truth[0] * scale, e[:, 1].mean() - truth[1] * scale
            dyaw = np.degrees((e[:, 2] - math.radians(yaw0) + math.pi / 4) % (math.pi / 2) - math.pi / 4).mean()
            allr[k].append((dx, dy, dyaw))
            cells.append(f"dx {dx:+5.2f} dy {dy:+5.2f} yaw {dyaw:+5.1f} n{len(e):2d}")
        print(f"{fid:9s} {row['found']:3d}/{row['frames']:<3d} {row['extra']:5d} | " + " | ".join(f"{c:30s}" for c in cells))
    for k, v in allr.items():
        if v:
            v = np.array(v)
            print(f"{k:11s} mean dx {v[:, 0].mean():+.2f} dy {v[:, 1].mean():+.2f} mm, horizontal spread {np.hypot(v[:, 0] - v[:, 0].mean(), v[:, 1] - v[:, 1].mean()).max():.2f} mm, |yaw| max {np.abs(v[:, 2]).max():.1f} deg")
    ts = [t for r in rows.values() for t in r["t"]]
    if ts:
        ts = np.array(ts)
        print(f"latency on this GPU: detector {ts[:, 0].mean() * 1000:.0f} ms, mask {ts[:, 1].mean() * 1000:.0f} ms per frame")


if __name__ == "__main__":
    main()
