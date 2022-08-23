def keyboard_forwordspeed (bt_w, bt_s,bt_down,bt_up):
    if(bt_w == 'w'or bt_up == 'up'):
        deta_forwardspeed = 1
    elif(bt_s == 's'or bt_down == 'down'):
        deta_forwardspeed = -1
    else:
        deta_forwardspeed = 0
    return deta_forwardspeed
            
def keyboard_steering_speed (bt_a,bt_d, bt_right,bt_left):
    if(bt_d == 'd'or bt_right == 'right'):
        deta_steering_speed = 1
    elif(bt_a == 'a' or bt_left == 'left'):
        deta_steering_speed = -1
    else:
        deta_steering_speed = 0
    return deta_steering_speed

def function_on_speed(bt_w, bt_s, bt_up, bt_down,bt_a, bt_d, bt_right, bt_left):

    alpha_forwordspeed = keyboard_forwordspeed(bt_w, bt_s, bt_up, bt_down)
    alpha_steeringspeed = keyboard_steering_speed(
                bt_a, bt_d, bt_right, bt_left)

    if forward_speed > 0 and forward_speed < 9:
            forward_speed = forward_speed+alpha_forwordspeed
    elif forward_speed <= 0:
            forward_speed = 0
    else:
            forward_speed = 9

    if steering_speed > 0 and steering_speed < 9:
                steering_speed = steering_speed+alpha_steeringspeed
    elif steering_speed <= 0:
                steering_speed = 0
    else:
                steering_speed = 9
    return forward_speed,steering_speed 