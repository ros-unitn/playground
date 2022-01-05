x_yolo = $(shell rospack find x_yolo)

train:
	python $(x_yolo)/yolov5/train.py --img 1024 --batch 12 --epochs 80 --data $(x_yolo)/legoblocks/yolo-train.yaml --weights $(x_yolo)/yolov5s.pt --cache

.PHONY: train