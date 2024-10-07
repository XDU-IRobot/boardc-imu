
# utility openocd targets

set(OPENOCD_EXECUTABLE "openocd")
set(OPENOCD_CONFIG_FILE "${CMAKE_HOME_DIRECTORY}/openocd/stm32f4_stlink.cfg")

add_custom_target(Download
        COMMAND openocd -f ${OPENOCD_CONFIG_FILE} -c "program ${CMAKE_PROJECT_NAME}.elf verify reset exit"
        VERBATIM
        DEPENDS ${CMAKE_PROJECT_NAME}
        COMMENT "Downloading program to target"
)
add_custom_target(Reset
        COMMAND openocd -f ${OPENOCD_CONFIG_FILE} -c "init" -c "reset" -c "exit"
        COMMENT "Resetting target"
)
add_custom_target(Erase
        COMMAND openocd -f ${OPENOCD_CONFIG_FILE} -c "init" -c "halt" -c "stm32f4x mass_erase 0" -c "exit"
        COMMENT "Erasing target"
)