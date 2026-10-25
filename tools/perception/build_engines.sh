#!/bin/sh
# Build the TensorRT engines on the Jetson from the ONNX files that export_models.py made (fp16).
#   tools/perception/build_engines.sh [onnx_dir] [engine_dir]
ONNX=${1:-$HOME/onnx}
OUT=${2:-$HOME/.cache/xlerobot/engines}
mkdir -p "$OUT"
for f in owlv2_det evit_l0_encoder evit_l0_decoder evit_l1_encoder evit_l1_decoder; do
    [ -f "$ONNX/$f.onnx" ] || continue
    [ -f "$OUT/$f.engine" ] && continue
    echo "building $f"
    case $f in
        *_decoder) SH="--minShapes=point_coords:1x1x2,point_labels:1x1 --optShapes=point_coords:1x2x2,point_labels:1x2 --maxShapes=point_coords:1x8x2,point_labels:1x8" ;;
        *) SH="" ;;
    esac
    trtexec --onnx="$ONNX/$f.onnx" --fp16 $SH --saveEngine="$OUT/$f.engine" > "$OUT/$f.log" 2>&1 || echo "  failed, see $OUT/$f.log"
done
