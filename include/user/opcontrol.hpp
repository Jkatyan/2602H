#ifndef _OPCONTROL_HPP
#define _OPCONTROL_HPP


struct Key_Binding{
    movement_e_t movement_type;
    pros::hydra_control_e_t control0;
    pros::hydra_control_e_t control1;
};


extern struct Key_Binding* KEY_MAP;


void load_keyBindings();


#endif
