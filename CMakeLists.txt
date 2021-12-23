add_executable(dev_led_device_test
        led_1111_controller.c
	    cmd_parser.c
        gzlib.c
        )

# generate the header file into the source tree as it is included in the RP2040 datasheet
pico_generate_pio_header(dev_led_device_test ${CMAKE_CURRENT_LIST_DIR}/apa104.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)

target_link_libraries(dev_led_device_test PRIVATE pico_stdlib hardware_pio hardware_resets hardware_irq)
pico_add_extra_outputs(dev_led_device_test)

# add url via pico_set_program_url
example_auto_set_url(dev_led_device_test)
