from ultralytics import YOLO

f = "engine"
# model1 = YOLO("size_x.pt")
model2 = YOLO("size_t.pt")
model3 = YOLO("size_s.pt")
# model4 = YOLO("size_m.pt")
# model5 = YOLO("size_l.pt")


out = model3.export(format=f, imgsz=640, dynamic=True, verbose=False, batch=8, workspace=2)
#models = [model3, model2]

#for mod in models:
#    mod.export(format=f, half=True, dynamic=True, data="drone.yaml", batch=8, workspace=8.0, verbose=False,) 
