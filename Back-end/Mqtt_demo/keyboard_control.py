def function_on_speed(bt_w, bt_s, bt_a, bt_d,forward_speed,steering_speed):

    alpha_forwordspeed = bt_w-bt_s
    alpha_steeringspeed = bt_d-bt_a

    if forward_speed >= 0 and forward_speed < 9:
            forward_speed = forward_speed+alpha_forwordspeed
            if forward_speed  <0:
                    forward_speed  =0
            if steering_speed >9:
                    forward_speed =9
    elif forward_speed < 0:
            forward_speed = 0
    else:
            forward_speed = 9

    if steering_speed >= 0 and steering_speed < 9:
                steering_speed = steering_speed+alpha_steeringspeed
                if steering_speed <0:
                    steering_speed =0
                if steering_speed >9:
                    steering_speed=9
    elif steering_speed < 0:
                steering_speed = 0
    else:
                steering_speed = 9
    return forward_speed,steering_speed 