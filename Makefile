.PHONY: build_robot_description_view \
		run_robot_description_view \
		stop_robot_description_view \
		build_robot_description_core \
		run_robot_description_core \
		stop_robot_description_core \
		connect_robot_description_view \
		connect_robot_description_core

build_robot_description_view:
	@docker compose build robot_description_view

run_robot_description_view:
	@docker compose up robot_description_view

stop_robot_description_view:
	@docker compose stop robot_description_view

build_robot_description_core:
	@docker compose build robot_description_core

run_robot_description_core:
	@docker compose up robot_description_core

stop_robot_description_core:
	@docker compose stop robot_description_core

connect_robot_description_container:
	@docker exec -it robot_description_view bash

connect_robot_description_core_container:
	@docker exec -it robot_description_core bash