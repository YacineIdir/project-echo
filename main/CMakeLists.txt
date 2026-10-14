idf_component_register(SRCS "main.c"
                    "implementations/joystick/joystick_impl.c"
                    "implementations/push_button/push_button_impl.c"
                    "implementations/potentiometer/potentiometer_impl.c"
                    "implementations/buzzer_passive/buzzer_passive_impl.c"
                    INCLUDE_DIRS "." 
                    REQUIRES joystick led push_button potentiometer buzzer_passive
                )
