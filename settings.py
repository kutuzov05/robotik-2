import pygame
import sympy


# Initialisierung vom Pygame-Modul
pygame.init()

width, height = 1000, 1000
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("PRR-Manipulator")
FPS = 60
clock = pygame.time.Clock()

# Farben im RGB
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREY = (100, 100, 100)
LIGHT_GREY = (175, 175, 175)
TEXT_GREY = (210, 210, 210)
BLUE = (0, 0, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)

# Variablen
x = sympy.symbols("x")
car_size = (width*0.175, height*0.075+width*0.175/7)
car_x = width*0.25
p_1, p_2, p_3 = 0, 0, 0
t_1, t_2, t_3 = 0, 0, 0
l_1, l_2 = width*0.2, width*0.15
obj_size = width*0.035
obj_pos = (width*0.75, height*0.5)
height_distance = width*0.01
width_distance = width*0.2
car_velocity = width/(5*FPS)
robot_velocity = width/(15*FPS)
is_writing = False
input_num = ""
selected_obj_pos = width*0.75
new_car_x = 0.25*width
obj_is_moving = False
show_details = False
round_angles = False
arm_direction = "right"


# Animation
timer = 0
input_cursor_frame = 0
move_obj_with_car = False
is_moving_to_obj = False
rotate_arm_1 = False
tool_positioning = False
is_picking_up = False
prep_move = False
car_is_moving = False
rotate_arm_2 = False
is_putting_down = False
move_car_to_new_pos = False
