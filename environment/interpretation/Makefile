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
	@echo "$(MAGENTA_COLOR)Creating lane_detector$(NO_COLOR)"
	rosmake lane_detector
	@echo "$(MAGENTA_COLOR)Creating obstacle_detector$(NO_COLOR)"
	rosmake obstacle_detector
	@echo "$(MAGENTA_COLOR)Creating position_from_target$(NO_COLOR)"
	rosmake position_from_target
	@echo "$(WHITE_COLOR)*** ALL Files successfully created!$(NO_COLOR)"
