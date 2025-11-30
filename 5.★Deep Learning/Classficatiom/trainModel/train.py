from ultralytics import YOLO

def main():
    # Load a model
    model = YOLO("yolov8n.pt")  # load a pretrained model (recommended for training)

    # Train using the single most idle GPU
    results = model.train(pretrained="H:\\TechProject\\yolo-train-main\\models\\4\\best.pt",
        data=r"H:\TechProject\yolo-train-main\datasets\My First Project.v8i.yolov11\data.yaml",
        epochs=400, 
        imgsz=640, 
        device=[-1,-1]  # 使用最空闲的GPU
    )

if __name__ == '__main__':
    main()



