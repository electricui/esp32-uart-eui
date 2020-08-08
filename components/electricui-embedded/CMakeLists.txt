idf_component_register(SRCS "electricui-embedded/src/electricui.c"
							"electricui-embedded/src/utilities/eui_crc.c"
							"electricui-embedded/src/utilities/eui_offset_validation.c"
							"electricui-embedded/src/transports/eui_serial_transport.c"
							 INCLUDE_DIRS "electricui-embedded/src"
							 			   "electricui-embedded/src/transports"
							 			   "electricui-embedded/src/utilities")
