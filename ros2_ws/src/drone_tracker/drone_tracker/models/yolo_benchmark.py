#### 
from ultralytics.utils.benchmarks import benchmark
import time

#Benchmark on GPU
model_t = 'size_t.pt'
benchmark(model=model_t, data='drone.yaml', device="cuda:0", imgsz=640)
#model_s = 'size_s.pt'
#benchmark(model=model_s, data='drone.yaml', device="cuda:0", imgsz=640)
#model_m = 'size_m.pt'
#benchmark(model=model_m, data='drone.yaml', device="cuda:0", imgsz=640)
#model_l = 'size_l.pt'
#benchmark(model=model_l, data='drone.yaml', device="cuda:0", imgsz=640)
#model_x = 'size_x.pt'
#benchmark(model=model_x, data='drone.yaml', device="cuda:0", imgsz=640)

# models = [model_t, model_s, model_m, model_l, model_x]
# for mod in models:
#    benchmark(model=mod, data='drone.yaml', device="cuda:0")
#    time.sleep(300)
