"""Detector and mask model on TensorRT for the Jetson (numpy + cv2 + tensorrt + cuda-python only).

TrtOwlv2: boxes for a text prompt from the OWLv2 engine built by build_engines.sh.
TrtEvitSam: EfficientViT-SAM L0/L1 box-prompted masks.
Both follow the interfaces of tools/segment_eval.py, so that script scores them on saved frames, on the Jetson or the laptop.
"""
import os

import cv2
import numpy as np

from .trt_runner import TrtEngine

ENGINES = os.path.expanduser("~/.cache/xlerobot/engines")
PROMPTS = os.path.expanduser("~/onnx/prompts.npz")
_CLIP_MEAN = np.array([0.48145466, 0.4578275, 0.40821073], np.float32)
_CLIP_STD = np.array([0.26862954, 0.26130258, 0.27577711], np.float32)


def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


class TrtOwlv2:
    SIZE = 960

    def __init__(self, prompt, engine=None, prompts=None):
        self.eng = TrtEngine(engine or os.path.join(ENGINES, "owlv2_det.engine"))
        emb = np.load(prompts or PROMPTS)
        if prompt not in emb.files:
            raise KeyError(f"no text embedding for {prompt!r}; export it on the laptop with export_models.py --prompts (have {emb.files})")
        self.query = emb[prompt].astype(np.float32)

    def __call__(self, img, region=None, thr=0.1):
        x0, y0, x1, y1 = region if region else (0, 0, img.shape[1], img.shape[0])
        crop = cv2.cvtColor(img[y0:y1, x0:x1], cv2.COLOR_BGR2RGB)
        h, w = crop.shape[:2]
        side = max(h, w)
        sq = np.full((side, side, 3), 0.5, np.float32)                 # the processor pads bottom and right with grey
        sq[:h, :w] = crop.astype(np.float32) / 255.0
        x = cv2.resize(sq, (self.SIZE, self.SIZE), interpolation=cv2.INTER_AREA if side > self.SIZE else cv2.INTER_LINEAR)
        x = ((x - _CLIP_MEAN) / _CLIP_STD).transpose(2, 0, 1)[None]
        out = self.eng(pixel_values=x, query_embeds=self.query)
        score = _sigmoid(out["logits"][0, :, 0])
        b = out["boxes"][0]
        keep = np.nonzero(score > thr)[0]
        res = []
        for k in keep:
            cx, cy, bw, bh = b[k] * side
            res.append((float(score[k]), cx - bw / 2 + x0, cy - bh / 2 + y0, cx + bw / 2 + x0, cy + bh / 2 + y0))
        return sorted(res, reverse=True)


class TrtEvitSam:
    SIZE = 512                                                          # L0/L1/L2 encoder input; the decoder works in a 1024 frame

    def __init__(self, variant="l0"):
        self.enc = TrtEngine(os.path.join(ENGINES, f"evit_{variant}_encoder.engine"))
        self.dec = TrtEngine(os.path.join(ENGINES, f"evit_{variant}_decoder.engine"))

    def mask(self, big_rgb, box):
        """Boolean mask for a box on a SIZE x SIZE RGB crop."""
        x = (big_rgb.astype(np.float32) - [123.675, 116.28, 103.53]) / [58.395, 57.12, 57.375]
        emb = self.enc(input_image=x.transpose(2, 0, 1)[None])["image_embeddings"].reshape(1, 256, 64, 64)
        k = 1024.0 / big_rgb.shape[0]
        pts = (np.array(box, np.float32).reshape(1, 2, 2) * k).astype(np.float32)
        out = self.dec(image_embeddings=emb, point_coords=pts, point_labels=np.array([[2, 3]], np.float32))
        low = out["masks"].reshape(256, 256)
        up = cv2.resize(low, (1024, 1024), interpolation=cv2.INTER_LINEAR)
        return cv2.resize(up, (big_rgb.shape[1], big_rgb.shape[0]), interpolation=cv2.INTER_LINEAR) > 0
