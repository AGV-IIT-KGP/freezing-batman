# Colour coding of the output text
NO_COLOR	  = \033[0m
WHITE_COLOR   = \033[37;01m
OK_COLOR      = \033[32;01m
ERROR_COLOR   = \033[31;01m
WARN_COLOR    = \033[33;01m
MAGENTA_COLOR = \033[35;01m
BLUE_COLOR    = \033[34;01m


all:
	rospack profile
	@echo "$(MAGENTA_COLOR)Creating vectorNav$(NO_COLOR)"
	rosmake vectorNav
	@echo "$(MAGENTA_COLOR)Creating odometry$(NO_COLOR)"
	rosmake odometry
	@echo "$(MAGENTA_COLOR)Creating logitech_camera$(NO_COLOR)"
	rosmake logitech_camera
	@echo "$(MAGENTA_COLOR)Creating hokuyo_node$(NO_COLOR)"
	rosmake hokuyo_node
	@echo "$(MAGENTA_COLOR)Creating bumblebee_stereo$(NO_COLOR)"
	rosmake bumblebee_stereo
	@echo "$(WHITE_COLOR)*** ALL Files successfully created!$(NO_COLOR)"
