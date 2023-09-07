import pygame

#You need to install pygame.
# See this video on installing pygame: https://youtu.be/dGA9GPi5cfg

#Pygame API is here: https://www.pygame.org/docs/

# pygame setup
pygame.init()

#origin is at the top left corner
# right side is positive x
# bottom is positive y
x_min = 0; y_min = 0;

#bottom right corner is x_max,y_max
x_max= 1280; y_max = 720;

#time increment
dt = 0.02

#initialize the screen size
screen = pygame.display.set_mode((x_max, y_max)) #16:9

#some constants
vx = -10; #speed of ball in the x-direction
vy = 10; #speed of ball in the y-direction
dx = 10;

#set the default font for all text
my_font = pygame.font.SysFont('Arial', 20)

#title of the game
pygame.display.set_caption('Pong Game by a ME410 student')

#initial ball position and size
ball_size = 10;
ball_pos = pygame.Vector2(x_max-100,y_min+100)
paddle_size = pygame.Vector2(100,20)
paddle_pos = pygame.Vector2(100,600)


#initialize the continuous loop
running = True

while running:
    # poll for events
    # pygame.QUIT event means the user clicked X to close your window
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # fill the screen with a color to wipe away anything from last frame
    screen.fill("white")

    #draw the ball and the paddle
    pygame.draw.circle(screen, "red", ball_pos, ball_size)
    pygame.draw.rect(screen,"blue",[paddle_pos.x, paddle_pos.y, paddle_size.x, paddle_size.y])

    #update the ball position.
    ball_pos.x += vx * dt
    ball_pos.y += vy * dt

    #Paddle movement is programmed here
    keys = pygame.key.get_pressed()
    if keys[pygame.K_LEFT]:
        paddle_pos.x -= dx
    if keys[pygame.K_RIGHT]:
        paddle_pos.x += dx

    #1) You need to write code to detect the walls and the paddle
    #2) You also need to write code that will ensure the proper physics after the ball bounces
    # of the walls and paddles

    #Signal the game is over if the ball goes down
    if (ball_pos.y>=y_max):
        text_game_over = my_font.render("Game Over", False, (0, 0, 0))
        screen.blit(text_game_over, (x_max/2,y_max/2))
        running = False


    # flip() the display to put your work on screen
    pygame.display.flip()

    #pause for 2000 ms before exiting
    if (running==False):
        pygame.time.wait(2000)


pygame.quit()
