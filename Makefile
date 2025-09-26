.PHONY: run_robot_view

IMAGE_NAME=robot_description
TAG=latest

run_robot_view:
	xhost +local:docker

	docker build -t $(IMAGE_NAME):$(TAG) .

	docker run -it --rm \
		--net=host \
		-e DISPLAY=$$DISPLAY \
		-e QT_X11_NO_MITSHM=1 \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		$(IMAGE_NAME):$(TAG) \
		ros2 launch robot_description robot_view.launch.py

	xhost -local:docker