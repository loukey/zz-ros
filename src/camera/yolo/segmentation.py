from ultralytics import YOLO
import glob


img_paths = glob.glob("./data/images/*.jpg")
model = YOLO("./models/yolo11x-seg.pt")
for img_path in img_paths:
    results = model(img_path)
    results[0].show()

