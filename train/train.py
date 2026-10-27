"""Train the board segmenter on train_data/ (make_dataset.py), export ONNX for the ROS node.

    pixi run -e train python train/train.py            # ~30 min on an RTX 5060 laptop
    # with reviewed real labels (train/add_real.py), fine-tune and pick the epoch on real frames only:
    START=runs/boards/weights/best.pt DATA=data_real.yaml EPOCHS=30 NAME=boards_real pixi run -e train python train/train.py
Result: models/boards-seg.onnx (+ runs/<NAME>/ curves, val images).
"""
import os
import shutil

from ultralytics import YOLO

HERE = os.path.dirname(os.path.abspath(__file__))
IMGSZ = 960  # ROI crop is 797 x 885 px: native resolution, boards ~200 px

START = os.environ.get("START", "yolo26s-seg.pt")
DATA = os.environ.get("DATA", "data.yaml")
EPOCHS = int(os.environ.get("EPOCHS", 40))
NAME = os.environ.get("NAME", "boards")

m = YOLO(START)
m.train(data=os.path.join(HERE, "..", "train_data", DATA), imgsz=IMGSZ, epochs=EPOCHS, batch=8, patience=15,
        flipud=0.5, degrees=10, mosaic=0.5, close_mosaic=10, project=os.path.join(HERE, "..", "runs"),
        name=NAME, exist_ok=True, workers=8, plots=True)
best = YOLO(os.path.join(HERE, "..", "runs", NAME, "weights", "best.pt"))
onnx = best.export(format="onnx", imgsz=IMGSZ, simplify=True)
os.makedirs(os.path.join(HERE, "..", "models"), exist_ok=True)
shutil.copy(onnx, os.path.join(HERE, "..", "models", "boards-seg.onnx"))
print("exported models/boards-seg.onnx")
