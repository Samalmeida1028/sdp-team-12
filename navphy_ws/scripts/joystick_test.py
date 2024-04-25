import pygame
import sys

# Initialize Pygame
pygame.init()

# Initialize the joystick module
pygame.joystick.init()

# Check if there are any joysticks
if pygame.joystick.get_count() == 0:
    print("No joysticks detected.")
    sys.exit()

# Initialize the joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Main loop
running = True
while running:
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.JOYBUTTONDOWN:
            print("Button {} pressed".format(event.button))
        elif event.type == pygame.JOYBUTTONUP:
            print("Button {} released".format(event.button))
        elif event.type == pygame.JOYAXISMOTION:
            print("Axis {} moved to {}".format(event.axis, joystick.get_axis(event.axis)))

# Quit Pygame
pygame.quit()
