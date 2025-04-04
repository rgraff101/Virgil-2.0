# Using YOLOv8 Retraining Docker 
Do following on the desktop in LSCA 105.

## Copy Dataset to Docker
```bash
cd ~
docker cp ~/buckets_dataset/ hailo_ai_sw_suite_2025-04_container:/local/workspace/datasets/
```

## Launch Docker
```bash
cd ~/Downloads
./hailo_ai_sw_suite_docker_run.sh --resume
```

## Launch the Retraining 
```bash
cd /local/workspace/hailo_model_zoo/training/yolov8
yolo detect train data=/local/workspace/datasets/buckets_dataset/data.yaml model=yolov8s.pt name=bucket_yolov8s epochs=50 batch=16
```

### Validate the new checkpoint 

```bash
yolo predict task=detect source=/local/workspace/datasets/buckets_dataset/valid/images/IMG_4511_jpg.rf.a1ce9a90595b28c828cbe7cd098bca9d.jpg model=./runs/detect/bucket_yolov8s/weights/best.pt
```

## Export the model to ONNX

```bash
yolo export model=./runs/detect/bucket_yolov8s/weights/best.pt imgsz=640 format=onnx opset=11
```

## Copy the ONNX to a directory mapped outside the Docker container

```bash
cd /local/workspace/hailo_model_zoo/training/yolov8/bucket_models
cp ../runs/detect/bucket_yolov8s/weights/best.onnx ./bucket_detector.onnx
```


## Convert the model to Hailo 

Use the Hailo Model Zoo command (this can take up to 30 minutes):

```bash
hailomz compile yolov8s --ckpt=bucket_detector.onnx --hw-arch hailo8l --calib-path /local/workspace/datasets/buckets_dataset/test/images/ --classes 1 --performance
```
Now, we have the yolov8s.hef. This file can be used on the Raspberry Pi 5 AI Kit.
