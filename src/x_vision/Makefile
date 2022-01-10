x_vision = $(shell rospack find x_vision)

train:
	python $(x_vision)/yolov5/train.py --img 1024 --batch 12 --epochs 80 --data $(x_vision)/legoblocks/yolo-train.yaml --weights $(x_vision)/yolov5s.pt --cache

.PHONY: train