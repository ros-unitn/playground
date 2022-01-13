x_vision = .

train:
	rm legoblocks/images
	ln -s ../legoblocks/color legoblocks/images
	python $(x_vision)/yolov5/train.py --img 1024 --batch 6 --epochs 80 --data $(x_vision)/legoblocks/yolo-train.yaml --weights $(x_vision)/yolov5m.pt --cache

train_depth:
	rm legoblocks/images
	ln -s ../legoblocks/depth legoblocks/images
	python $(x_vision)/yolov5/train.py --img 1024 --batch 6 --epochs 80 --data $(x_vision)/legoblocks/yolo-train-depth.yaml --weights $(x_vision)/yolov5m.pt --cache

.PHONY: train