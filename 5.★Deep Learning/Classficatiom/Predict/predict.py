import time
import cv2
import os
import numpy as np
try:
    import torch
except Exception:
    torch = None
from ultralytics import YOLO



def main():
    model_path = r"H:\\TechProject\\yolo-train-main\\models\\5\\best.pt"
    stream_url = 'http://192.168.50.1:8080/?action=stream'

    # Prefer CUDA if available
    if torch is not None and torch.cuda.is_available():
        device = 'cuda:0'
    else:
        device = 'cpu'

    print(f"Loading model {model_path} on device {device}...")
    model = YOLO(model_path)
    try:
        model.to(device)
    except Exception as e:
        print(f"Warning: couldn't set device to {device}: {e}\nFalling back to model default device.")

    cap = cv2.VideoCapture(stream_url)
    if not cap.isOpened():
        print(f"Warning: can't open stream {stream_url}, trying local camera 0")
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            raise SystemExit("Cannot open video stream or local camera")

    # 获取视频参数
    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps_in = cap.get(cv2.CAP_PROP_FPS) or 30.0  # 有些摄像头返回0，这里给默认值

    # 保存路径（同目录）
    save_path = os.path.join(os.path.dirname(__file__), "yolo_output3.avi")
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(save_path, fourcc, fps_in, (width, height))

    win_name = 'YOLO Real-time (press q to quit)'
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)

    fps_avg = 0.0
    frame_count = 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                time.sleep(0.1)
                continue

            frame_count += 1
            t_start = time.time()

            # Run inference
            try:
                results = model(frame, device = 'cuda:0', conf=0.25, verbose=False)
            except Exception as e:
                print(f"Inference error: {e}")
                continue

            # Draw results
            if results and len(results) > 0:
                r = results[0]
                if hasattr(r, 'boxes') and r.boxes is not None:
                    try:
                        xyxy = r.boxes.xyxy.cpu().numpy()
                        confs = r.boxes.conf.cpu().numpy()
                        classes = r.boxes.cls.cpu().numpy()
                    except Exception:
                        xyxy = np.array(r.boxes.xyxy)
                        confs = np.array(r.boxes.conf)
                        classes = np.array(r.boxes.cls)

                    for (x1, y1, x2, y2), c, s in zip(xyxy, classes, confs):
                        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                        cls_id = int(c) if not isinstance(c, np.ndarray) else int(c[0])
                        score = float(s)
                        label = f"{model.names.get(cls_id, cls_id)} {score:.2f}" if hasattr(model, 'names') else f"{cls_id} {score:.2f}"

                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
                        cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw, y1), (0, 255, 0), -1)
                        cv2.putText(frame, label, (x1, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)

            # FPS
            t_end = time.time()
            fps = 1.0 / (t_end - t_start) if (t_end - t_start) > 0 else 0.0
            fps_avg = fps_avg * 0.95 + fps * 0.05 if frame_count > 1 else fps
            cv2.putText(frame, f"FPS: {fps_avg:.1f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)

            # 显示
            cv2.imshow(win_name, frame)

            # 保存帧
            out.write(frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        cap.release()
        out.release()  # 释放视频写入器
        cv2.destroyAllWindows()
        print(f"视频已保存到: {save_path}")

if __name__ == '__main__':
    main()


    #  & C:/Users/17278/AppData/Local/Programs/Python/Python310/python.exe h:/TechProject/predict2.py
