import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

main_path = "/home/usrl1234/ros2_ws/src/drone_tracker/metrics_gathering"
tracker_path = os.path.join(main_path, "tracker_bbox")
gt_path = os.path.join(main_path, "Anti-UAV-Tracking-V0GT")
methods = ["na", "mosse", "kcf", "csrt", "sort", "deepsort"]

# Create a new folder for saving results
results_folder = os.path.join(main_path, "results")
os.makedirs(results_folder, exist_ok=True)

def convert_gt_bbox(gt_bbox):
    x, y, w, h = gt_bbox
    x1 = x
    y1 = y
    x2 = x + w
    y2 = y + h
    return [x1, y1, x2, y2]

def calculate_iou(boxA, boxB):
    x1, y1, x2, y2 = boxA
    x3, y3, x4, y4 = boxB
    x_inter1 = max(x1, x3)
    y_inter1 = max(y1, y3)
    x_inter2 = min(x2, x4)
    y_inter2 = min(y2, y4)

    width_inter = max(0, x_inter2 - x_inter1)
    height_inter = max(0, y_inter2 - y_inter1)
    area_inter = width_inter * height_inter

    width_box1 = abs(x2 - x1)
    height_box1 = abs(y2 - y1)
    width_box2 = abs(x4 - x3)
    height_box2 = abs(y4 - y3)

    area_box1 = width_box1 * height_box1
    area_box2 = width_box2 * height_box2

    area_union = area_box1 + area_box2 - area_inter
    iou = area_inter / float(area_union) if area_union > 0 else 0
    return iou

def process_files(gt_file, tracker_file):
    gt_data = pd.read_csv(gt_file, delim_whitespace=True, header=None).values
    tracker_data = pd.read_csv(tracker_file, header=None).values

    min_frames = min(len(gt_data), len(tracker_data))

    successes = 0
    iou_list = []
    valid_iou_list = []  # List for non-failed detections (valid tracking for accuracy)

    for frame_idx in range(min_frames):
        gt_bbox = convert_gt_bbox(gt_data[frame_idx])
        tracker_bbox = tracker_data[frame_idx]

        # If the tracker missed detection (0 0 0 0), append IOU as 0 but skip for accuracy calculation
        if np.all(tracker_bbox == 0):
            iou_list.append(0)  # Append 0 IOU for missed detection
            continue

        successes += 1
        iou = calculate_iou(gt_bbox, tracker_bbox)
        iou_list.append(iou)  # Append IOU for plotting
        valid_iou_list.append(iou)  # Append IOU for accuracy calculation

    success_rate = successes / min_frames if min_frames > 0 else 0
    tracking_accuracy = np.mean(valid_iou_list) if valid_iou_list else 0  # Use valid IOUs only for accuracy
    return success_rate, tracking_accuracy, iou_list

results = {}

combined_fig, combined_axs = plt.subplots(nrows=5, ncols=1, figsize=(12, 30))
plt.subplots_adjust(hspace=0.5)

for i in range(5):
    video_gt_file = os.path.join(gt_path, f"video{i+1:02d}_gt.txt")

    plt.figure(figsize=(48, 12))

    for method in methods:
        tracker_file = os.path.join(tracker_path, f"{i}_{method}.csv")

        if not os.path.exists(tracker_file):
            print(f"Tracker file not found: {tracker_file}")
            continue

        success_rate, tracking_accuracy, iou_list = process_files(video_gt_file, tracker_file)

        results[f"Video {i+1:02d} - {method}"] = {
            "success_rate": success_rate,
            "tracking_accuracy": tracking_accuracy
        }

        plt.plot(range(1, len(iou_list) + 1), iou_list, label=method)
        combined_axs[i].plot(range(1, len(iou_list) + 1), iou_list, label=method)
    
    plt.xticks(fontsize=34)
    plt.xlabel("Frame", fontsize=40)
    plt.xlim(left=0)
    plt.ylabel("IOU", fontsize=40)
    plt.ylim(0, 1.05)
    plt.yticks(np.arange(0, 1.1, 0.2), fontsize=34)
    plt.title(f"Scenario {i+1}", fontsize=50)
    plt.legend(fontsize=36)
    plt.grid(True)
    plt.savefig(os.path.join(results_folder, f"Scenario {i+1}_iou_comparison_plot.png"))
    plt.close()

    print(f"Scenario {i+1} done")

result_df = pd.DataFrame.from_dict(results, orient='index')
result_df.to_csv(os.path.join(results_folder, "tracking_results_summary.csv"))

print(f"Processing complete. Results saved to {results_folder}.")
