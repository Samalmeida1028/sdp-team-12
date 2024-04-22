import pygame

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No joystick detected.")

joystick = pygame.joystick.Joystick(0)
joystick.init()

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            quit()

    yaxis = joystick.get_axis(0)
    xaxis = joystick.get_axis(1)

    print(xaxis, yaxis)