from ultralytics import YOLO

model = YOLO("size_t.pt")

results = model.train(data="drone.yaml", epochs = 100, imgsz=640, device=0, cos_lr=True, dropout=0.2)