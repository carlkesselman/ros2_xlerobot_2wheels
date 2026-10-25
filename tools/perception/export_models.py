#!/usr/bin/env python3
"""Export the perception models to ONNX on the laptop (needs torch; ~/venvs/xlerobot-perception) and copy them to the Jetson,
where `build_engines.sh` turns them into TensorRT engines.

  ~/venvs/xlerobot-perception/bin/python tools/perception/export_models.py [--out ~/.cache/xlerobot/onnx] [--prompts "a cube" "a toy block"]

owlv2_det.onnx: pixel_values [1,3,960,960] + query_embeds [1,1,512] -> logits [1,3600,1], boxes [1,3600,4] (cx,cy,w,h normalised
to the padded square). The text side runs here, once per prompt: the embeddings go to prompts.npz, so changing the prompt on the
robot needs no text encoder there. The EfficientViT-SAM encoders and decoders come from the authors' ONNX on Hugging Face.
"""
import argparse
import os
import shutil

import numpy as np
import torch


def export_owlv2(out, prompts):
    from transformers import Owlv2ForObjectDetection, Owlv2Processor
    name = "google/owlv2-base-patch16-ensemble"
    proc, model = Owlv2Processor.from_pretrained(name), Owlv2ForObjectDetection.from_pretrained(name).eval()

    class Det(torch.nn.Module):
        def __init__(s, m):
            super().__init__()
            s.m = m

        def forward(s, pixel_values, query_embeds):
            fm = s.m.image_embedder(pixel_values=pixel_values)[0]
            feats = fm.reshape(fm.shape[0], -1, fm.shape[-1])
            logits, _ = s.m.class_predictor(feats, query_embeds, None)
            return logits, s.m.box_predictor(feats, fm)

    emb = {}
    with torch.no_grad():
        for p in prompts:
            t = proc.tokenizer([p], padding="max_length", max_length=16, return_tensors="pt")
            te = model.owlv2.get_text_features(input_ids=t["input_ids"], attention_mask=t["attention_mask"])
            if not torch.is_tensor(te):
                te = te.pooler_output
            emb[p] = (te / te.norm(dim=-1, keepdim=True)).numpy().reshape(1, 1, -1)
    np.savez(os.path.join(out, "prompts.npz"), **{k: v for k, v in emb.items()})
    det = Det(model)
    x, q = torch.randn(1, 3, 960, 960), torch.from_numpy(emb[prompts[0]])
    torch.onnx.export(det, (x, q), os.path.join(out, "owlv2_det.onnx"), input_names=["pixel_values", "query_embeds"],
                      output_names=["logits", "boxes"], opset_version=17, dynamo=False)
    print("owlv2_det.onnx and prompts.npz written")


def fetch_evit(out):
    from huggingface_hub import hf_hub_download
    for n in ("l0", "l1", "l2"):
        for k in ("encoder", "decoder"):
            dst = os.path.join(out, f"evit_{n}_{k}.onnx")
            if not os.path.exists(dst):
                shutil.copyfile(hf_hub_download("mit-han-lab/efficientvit-sam", f"onnx/{n}_{k}.onnx"), dst)
    print("efficientvit-sam l0/l1/l2 onnx copied")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default=os.path.expanduser("~/.cache/xlerobot/onnx"))
    ap.add_argument("--prompts", nargs="+", default=["a cube", "a toy block", "a lego brick"])
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    export_owlv2(a.out, a.prompts)
    fetch_evit(a.out)
