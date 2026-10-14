#include <stdio.h>
#include "implementations/joystick/joystick_impl.h"
#include "implementations/push_button/push_button_impl.h"
#include "implementations/potentiometer/potentiometer_impl.h"
#include "implementations/buzzer_passive/buzzer_passive_impl.h"

void app_main(void)
{
    start_potentiometer();
    start_joystick();
    start_buzzer_passive();
}