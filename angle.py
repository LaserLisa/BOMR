import math

def move_to_checkpoint(pos_x, pos_y, pos_angle, check_x, check_y):
        #I assume pos_angle to be 0 when facing North and go towards 360 counter-clockwise!
        
        #calculate rotation & distance
        dx = check_x - pos_x
        dy = check_y - pos_y
        
        dir = math.degrees(math.atan2(dy, dx))

        return dir
    
    
print(move_to_checkpoint(0, 0, 0, 1, 0))
print(move_to_checkpoint(0, 0, 0, 1, 1))
print(move_to_checkpoint(0, 0, 0, 0, 1))
print(move_to_checkpoint(0, 0, 0, -1, 1))
print(move_to_checkpoint(0, 0, 0, -1, 0))
print(move_to_checkpoint(0, 0, 0, -1, -1))
print(move_to_checkpoint(0, 0, 0, 0, -1))
print(move_to_checkpoint(0, 0, 0, 1, -1))

