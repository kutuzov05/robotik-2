from settings import *
import random
import numpy


def deg2rad(angle):
    return angle/180*numpy.pi


def rad2deg(angle):
    return angle/numpy.pi*180


# Rotationsmatrix
def R(angle):
    return sympy.Matrix([[numpy.cos(angle), -numpy.sin(angle)], [numpy.sin(angle), numpy.cos(angle)]])


# Direktes kinematisches Problem
def DKP(O_t, angle, P_t):
    return sympy.Matrix(O_t) + R(angle) * sympy.Matrix(P_t)


# Inverses kinematisches problem
def IKP(t):
    if arm_direction == "right":
        if float((t[0] ** 2 + t[1] ** 2 + l_1 ** 2 - l_2 ** 2) / (2 * l_1 * (t[0] ** 2 + t[1] ** 2) ** 0.5)) <= -1:
            a = -1
        elif float((t[0] ** 2 + t[1] ** 2 + l_1 ** 2 - l_2 ** 2) / (2 * l_1 * (t[0] ** 2 + t[1] ** 2) ** 0.5)) >= 1:
            a = 1
        else:
            a = float((t[0] ** 2 + t[1] ** 2 + l_1 ** 2 - l_2 ** 2) / (2 * l_1 * (t[0] ** 2 + t[1] ** 2) ** 0.5))
        t1 = rad2deg(numpy.arctan2(float(t[1]), float(t[0])) - numpy.arccos(
            a))
        if float((l_1 ** 2 + l_2 ** 2 - t[0] ** 2 - t[1] ** 2) / (2 * l_1 * l_2)) <= -1:
            b = -1
        elif float((l_1 ** 2 + l_2 ** 2 - t[0] ** 2 - t[1] ** 2) / (2 * l_1 * l_2)) >= 1:
            b = 1
        else:
            b = float((l_1 ** 2 + l_2 ** 2 - t[0] ** 2 - t[1] ** 2) / (2 * l_1 * l_2))
        t2 = 180 - rad2deg(numpy.arccos(b))
        return [t1, t2]
    else:
        u, v = t[0], t[1]
        a = 2 * l_1 * l_2
        b = l_1 ** 2 + l_2 ** 2 - u ** 2 - v ** 2
        c = l_1 ** 2 - l_2 ** 2 + u ** 2 + v ** 2
        d = (((a + b) * (a - b))**2)**0.5
        c1 = (c * u - v * d ** 0.5) / (2 * l_1 * (u ** 2 + v ** 2))
        s1 = (c * v + u * d ** 0.5) / (2 * l_1 * (u ** 2 + v ** 2))
        t1 = rad2deg(numpy.arctan2(float(s1), float(c1)))
        c2 = (u - l_1 * c1) / l_2
        s2 = (v - l_1 * s1) / l_2
        t2 = rad2deg(numpy.arctan2(float(s2), float(c2))) - t1
        return [t1, t2]


# Darstellung der visuellen Elemente
def visualisation():
    screen.fill(GREY)
    pygame.draw.rect(screen, LIGHT_GREY, (0, height*0.5, width, height*0.5))
    draw_car()
    display_input_box()
    if show_details:
        display_details()
    font = pygame.font.Font("fonts/unicode.ttf", int(height*0.02))
    text_1 = font.render("F1: für mehr Details", True, BLACK)
    text_1_pos = [width*0.451, height*0.001]
    text_2 = font.render("F2: für gerundete Winkel", True, BLACK)
    text_2_pos = [width*0.701, height*0.001]
    screen.blit(text_1, text_1_pos)
    screen.blit(text_2, text_2_pos)


# Fahrzeug wird gezeichnet
def draw_car():
    pygame.draw.rect(screen, WHITE, (car_x-car_size[0]/2, height*0.5-car_size[1], car_size[0], car_size[1]-width*0.175/7))
    pygame.draw.circle(screen, WHITE, (car_x-car_size[0]/2+width*0.175/7*1.8, height*0.5-width*0.175/7), int(width*0.175/7))
    pygame.draw.circle(screen, WHITE, (car_x-car_size[0]/2+width*0.175/7*5.2, height*0.5-width*0.175/7), int(width*0.175/7))
    draw_robot_arm()


# Roboterarm wird gezeichnet
def draw_robot_arm():
    calculate_robot_arm_points()
    draw_obj()
    # Gelenke werden gezeichnet
    pygame.draw.circle(screen, WHITE, p_1, int(height*0.01))
    pygame.draw.circle(screen, WHITE, p_2, int(height*0.01))
    pygame.draw.circle(screen, WHITE, p_3, int(height*0.01))
    # Gelenke werden verbunden
    pygame.draw.line(screen, WHITE, p_1, p_2, int(height*0.0075))
    pygame.draw.line(screen, WHITE, p_2, p_3, int(height*0.0075))
    draw_tool()


# Berechnung der Gelenke des Arms
def calculate_robot_arm_points():
    global p_1, p_2, p_3, t_3
    p_1 = (car_x, height*0.5-car_size[1])
    p_2 = DKP(p_1, deg2rad(t_1), (l_1, 0))
    p_3 = DKP(p_2, deg2rad(t_1+t_2), (l_2, 0))
    t_3 = 180-(t_1+t_2)


# "Greifarm" wird am 3. Gelenk gezeichnet
def draw_tool():
    tool_width, tool_height = obj_size, obj_size
    line_width = int(height*0.0075)
    pygame.draw.line(screen, WHITE, DKP(p_3, deg2rad(t_1+t_2+t_3), (tool_width/2, 0)), DKP(p_3, deg2rad(t_1+t_2+t_3), (tool_width/2, -tool_height)), line_width)
    pygame.draw.line(screen, WHITE, DKP(p_3, deg2rad(t_1+t_2+t_3), (tool_width/2, 0)), DKP(p_3, deg2rad(t_1+t_2+t_3), (-tool_width/2, 0)), line_width)
    pygame.draw.line(screen, WHITE, DKP(p_3, deg2rad(t_1+t_2+t_3), (-tool_width/2, 0)), DKP(p_3, deg2rad(t_1+t_2+t_3), (-tool_width/2, -tool_height)), line_width)


# Das zu bewegende Objekt wird gezeichnet
def draw_obj():
    global obj_pos
    if move_obj_with_car:
        obj_pos = [p_3[0], p_3[1]+obj_size]
    pygame.draw.rect(screen, BLACK, (obj_pos[0]-obj_size/2, obj_pos[1]-obj_size, obj_size, obj_size))
    # pygame.draw.rect(screen, RED, (obj_pos[0]-width_distance, p_1[1]-1, width_distance*2, 2))


# Zur Überprüfung, wo das Fahrzeug nach Ablage des Objekts fahren kann, um nicht durch das Objekt durch zu fahren
def check_area(pos):
    collision = True
    if ((pos-selected_obj_pos)**2)**0.5 > width_distance:
        collision = False
    return collision


# Fahrzeug bewegt sich zum Objekt mit einem bestimmten Abstand
def pick_up_pos():
    point = 0
    if car_x < obj_pos[0]:
        point = obj_pos[0]-width_distance
    elif car_x > obj_pos[0]:
        point = obj_pos[0]+width_distance
    return point


# Fahrzeug fährt zu einem Punkt, welcher den bestimmten Abstand entfernt zur gewünschten Position ist
def place_down_pos():
    point, direction = 0, None
    if pick_up_pos() < selected_obj_pos:
        point = selected_obj_pos - width_distance
        direction = "right"
    elif pick_up_pos() > selected_obj_pos:
        point = selected_obj_pos + width_distance
        direction = "left"
    return [point, direction]


# zufällige Position wird für das Fahrzeug gewählt
def pick_random_pos():
    options = []
    for nums in range(101):
        options.append(nums)
    run = True
    while run:
        option = random.choice(options)
        point = car_size[0]/2+option/100*(width-car_size[0])
        if place_down_pos()[1] == "right":
            if not check_area(point) and point < selected_obj_pos:
                run = False
        elif place_down_pos()[1] == "left":
            if not check_area(point) and point > selected_obj_pos:
                run = False
    return point


# Visualisierung des Schreibfelds, um Position des Objekts festzulegen
def display_input_box():
    pygame.draw.rect(screen, WHITE, (width*0.25, height*0.85, width*0.5, height*0.1))
    font = pygame.font.Font(None, int(height*0.05))
    text = font.render("Eingabe (in %)", True, TEXT_GREY)
    text_pos = text.get_rect()
    text_pos.center = (width*0.5, height*0.9)
    input_text = font.render(str(input_num), True, BLACK)
    input_cursor_pos = input_text.get_width()+width*0.255
    input_text_pos = (width*0.255, height*(0.85+1/30))
    if input_cursor_frame == 1 and is_writing:
        pygame.draw.line(screen, BLACK, (input_cursor_pos, height*0.855), (input_cursor_pos, height*0.945), int(height*0.003))
    elif not is_writing:
        screen.blit(text, text_pos)
    screen.blit(input_text, input_text_pos)


def animations():
    global move_obj_with_car, rotate_arm_1, rotate_arm_2, is_putting_down, tool_positioning
    if is_moving_to_obj:
        car_pick_up_animation()
    elif tool_positioning:
        prep_tool_pickup()
    elif rotate_arm_1:
        rotate_robot_arm(check_direction(car_x, obj_pos[0]))
        rotate_arm_1 = False
    elif is_picking_up:
        pick_up_obj()
    elif prep_move:
        move_obj_with_car = True
        move_tool_back()
    elif car_is_moving:
        move_car_to_new_obj_pos()
    elif rotate_arm_2:
        rotate_robot_arm(check_direction(car_x, selected_obj_pos))
        rotate_arm_2 = False
    elif is_putting_down:
        pick_up_obj()
        move_obj_with_car = False
        move_tool_back()
        is_putting_down = False
    elif move_car_to_new_pos:
        move_car_away_from_obj()


# Bewegung des Fahrzeugs in x-Richtung
def car_pick_up_animation():
    global car_x, is_moving_to_obj, new_t1, new_t2
    if car_x + car_velocity < pick_up_pos():
        car_x += car_velocity
    elif car_x - car_velocity > pick_up_pos():
        car_x -= car_velocity
    else:
        car_x = pick_up_pos()
        is_moving_to_obj = False


# Roboterarm wird so rotiert, dass der Greifer das Objekt aufheben kann
def prep_tool_pickup():
    global t_1, t_2, tool_positioning
    if car_x < obj_pos[0]:
        direction = "left"
    else:
        direction = "right"
    if direction == "left" and p_3[0]-robot_velocity > pick_up_tool_pos()[0]:
        t_1, t_2 = IKP([p_3[0]-robot_velocity-p_1[0], 0])
    elif direction == "left":
        t_1, t_2 = IKP([pick_up_tool_pos()[0]-p_1[0], pick_up_tool_pos()[1]-p_1[1]])
        tool_positioning = False
    if direction == "right" and p_3[0]+robot_velocity < pick_up_tool_pos()[0]:
        t_1, t_2 = IKP([p_3[0]+robot_velocity-p_1[0], 0])
        pygame.display.update()
    elif direction == "right":
        t_1, t_2 = IKP([pick_up_tool_pos()[0]-p_1[0], pick_up_tool_pos()[1]-p_1[1]])
        tool_positioning = False


def pick_up_obj():
    global t_1, t_2, is_picking_up
    elements = 100
    y = numpy.linspace(float(p_3[1]), height*0.5-obj_size, elements)
    for nums in range(len(y)):
        t_1, t_2 = IKP([p_3[0]-p_1[0], y[nums]-p_1[1]])
        visualisation()
        pygame.display.update()
    is_picking_up = False


def move_tool_back():
    global t_1, t_2, prep_move, obj_pos
    elements = 100
    y = numpy.linspace(float(p_3[1]), p_1[1], elements)
    for nums in range(len(y)):
        t_1, t_2 = IKP([p_3[0] - p_1[0], y[nums] - p_1[1]])
        visualisation()
        pygame.display.update()
    prep_move = False


def move_car_to_new_obj_pos():
    global car_x, car_is_moving
    point = calc_closer_xcoord(car_x, selected_obj_pos-width_distance, selected_obj_pos+width_distance)
    if car_x > point:
        direction = "left"
    else:
        direction = "right"
    process = True
    while process:
        if direction == "left" and car_x-car_velocity/2 > point:
            car_x -= car_velocity/2
            visualisation()
            pygame.display.update()
        elif direction == "right" and car_x+car_velocity/2 < point:
            car_x += car_velocity/2
            visualisation()
            pygame.display.update()
        else:
            process = False
            car_x = calc_closer_xcoord(car_x, selected_obj_pos-width_distance, selected_obj_pos+width_distance)
            car_is_moving = False


def move_car_away_from_obj():
    global car_x, move_car_to_new_pos
    if car_x > new_car_x:
        direction = "left"
    else:
        direction = "right"
    if direction == "left" and car_x-car_velocity > new_car_x:
        car_x -= car_velocity
    elif direction == "right" and car_x+car_velocity < new_car_x:
        car_x += car_velocity
    else:
        car_x = new_car_x
        move_car_to_new_pos = False


# Entscheidet, ob das Fahrzeug links oder rechts vom Objekt sein muss
def calc_closer_xcoord(system, point_1, point_2):
    if ((point_1-system)**2)**0.5 > ((point_2-system)**2)**0.5:
        result = point_2
    else:
        result = point_1
    return result


# Arm wird rotiert
def rotate_robot_arm(direction):
    global t_1, t_2, tool_positioning
    radius = ((p_3[0]-car_x)**2)**0.5
    if arm_direction == direction:
        start_pos(direction)
        radius = l_1+l_2
    if direction == "left":
        target_pos = [radius, 0]
        process = True
        while process:
            if p_3[0]+robot_velocity-car_x < target_pos[0]:
                t_1, t_2 = IKP([p_3[0]+robot_velocity-car_x, -(radius**2-(p_3[0]+robot_velocity-car_x)**2)**0.5])
                visualisation()
                pygame.display.update()
            else:
                t_1, t_2 = IKP(target_pos)
                process = False
    else:
        target_pos = [-radius, 0]
        process = True
        while process:
            if p_3[0]-robot_velocity-car_x > target_pos[0]:
                t_1, t_2 = IKP([p_3[0]-robot_velocity-car_x, -(radius**2-(p_3[0]-robot_velocity-car_x)**2)**0.5])
                visualisation()
                pygame.display.update()
            else:
                t_1, t_2 = IKP(target_pos)
                process = False
    if radius == l_1+l_2:
        tool_positioning = True


# Überprüfung der Fahrtrichtung des Fahrzeugs
def check_direction(center, point):
    result = point - center
    if result < 0:
        direction = "right"
    else:
        direction = "left"
    return direction


# "Homeposition" mit 0 als Startwinkel aller Gelenkwinkel
def start_pos(direction):
    global arm_direction, t_1, t_2
    if direction == "right":
        target_pos = [l_1+l_2, 0]
        process = True
        while process:
            if p_3[0]+robot_velocity-car_x < target_pos[0]:
                t_1, t_2 = IKP([p_3[0]+robot_velocity-car_x, 0])
                visualisation()
                pygame.display.update()
            else:
                t_1, t_2 = IKP(target_pos)
                arm_direction = "left"
                process = False
    else:
        target_pos = [-l_1-l_2, 0]
        process = True
        while process:
            if p_3[0]-robot_velocity-car_x > target_pos[0]:
                t_1, t_2 = IKP([p_3[0]-robot_velocity-car_x, 0])
                visualisation()
                pygame.display.update()
            else:
                t_1, t_2 = IKP(target_pos)
                arm_direction = "right"
                process = False


def pick_up_tool_pos():
    if car_x > obj_pos[0]:
        return [car_x-width_distance, p_1[1]]
    else:
        return [car_x+width_distance, p_1[1]]


def display_details():
    if round_angles:
        t1, t2, t3 = (round(t_1*100))/100, (round(t_2*100))/100, (round(t_3*100))/100
    else:
        t1, t2, t3 = t_1, t_2, t_3
    font = pygame.font.Font("fonts/unicode.ttf", int(height*0.05))
    t_1_text = font.render("\u03B8\u2081 = "+str(t1), True, RED)
    t_1_text_pos = (width*0.001, height*0.501)
    screen.blit(t_1_text, t_1_text_pos)
    t_2_text = font.render("\u03B8\u2082 = "+str(t2), True, RED)
    t_2_text_pos = (width*0.001, height*0.551)
    screen.blit(t_2_text, t_2_text_pos)
    t_3_text = font.render("\u03B8\u2083 = "+str(t3), True, RED)
    t_3_text_pos = (width*0.001, height*0.601)
    screen.blit(t_3_text, t_3_text_pos)
    l_1_text = font.render("l\u2081 = "+str(int(l_1)), True, RED)
    l_1_text_pos = (width*0.781, height*0.501)
    screen.blit(l_1_text, l_1_text_pos)
    l_2_text = font.render("l\u2082 = "+str(int(l_2)), True, RED)
    l_2_text_pos = (width*0.781, height*0.551)
    screen.blit(l_2_text, l_2_text_pos)
    car_x_text = font.render("car\u2093 = "+str(int(car_x)), True, RED)
    car_x_text_pos = (width*0.001, height*0.651)
    screen.blit(car_x_text, car_x_text_pos)
    obj_x_text = font.render("obj\u2093 = "+str(int(obj_pos[0])), True, RED)
    obj_x_text_pos = (width*0.001, height*0.701)
    screen.blit(obj_x_text, obj_x_text_pos)
    font = pygame.font.Font("fonts/unicode.ttf", int(height*0.02))
    fps_text = font.render("FPS = "+str(int(FPS)), True, GREEN)
    fps_text_pos = (width*0.001, height*0.001)
    screen.blit(fps_text, fps_text_pos)


# Main-Loop
running = True
while running:
    timer += 1
    mouse_x, mouse_y = pygame.mouse.get_pos()
    if not timer % (FPS/3):
        input_cursor_frame += 1
    if input_cursor_frame == 2:
        input_cursor_frame = 0
    clock.tick(FPS)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                if width*0.25 <= mouse_x <= width*0.75 and height*0.85 <= mouse_y <= height*0.95 and not obj_is_moving:
                    is_writing = True
                else:
                    is_writing = False
        if event.type == pygame.KEYDOWN:
            if is_writing:
                if event.key == pygame.K_BACKSPACE:
                    input_num = input_num[0:len(input_num)-1]
                if len(input_num) < 3 and not obj_is_moving:
                    if event.key == pygame.K_0:
                        input_num += "0"
                    if event.key == pygame.K_1:
                        input_num += "1"
                    if event.key == pygame.K_2:
                        input_num += "2"
                    if event.key == pygame.K_3:
                        input_num += "3"
                    if event.key == pygame.K_4:
                        input_num += "4"
                    if event.key == pygame.K_5:
                        input_num += "5"
                    if event.key == pygame.K_6:
                        input_num += "6"
                    if event.key == pygame.K_7:
                        input_num += "7"
                    if event.key == pygame.K_8:
                        input_num += "8"
                    if event.key == pygame.K_9:
                        input_num += "9"
                    if input_num != "":
                        if int(input_num) > 100:
                            input_num = "100"
                if event.key == pygame.K_RETURN:
                    is_writing = False
                    is_moving_to_obj = True
                    rotate_arm_1 = True
                    tool_positioning = True
                    is_picking_up = True
                    prep_move = True
                    car_is_moving = True
                    rotate_arm_2 = True
                    is_putting_down = True
                    move_car_to_new_pos = True
                    selected_obj_pos = obj_size/2+int(input_num)/100*(width-obj_size)
                    # obj_pos = [selected_obj_pos, obj_pos[1]]
                    input_num = ""
                    # obj_is_moving = True
                    new_car_x = pick_random_pos()
                    # car_x = new_car_x
            if event.key == pygame.K_F1:
                if show_details:
                    show_details = False
                else:
                    show_details = True
            if event.key == pygame.K_F2:
                if round_angles:
                    round_angles = False
                else:
                    round_angles = True
    visualisation()
    animations()
    pygame.display.update()
pygame.quit()
