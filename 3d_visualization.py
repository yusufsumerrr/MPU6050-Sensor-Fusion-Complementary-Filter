import pygame
import serial
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *

# ==========================================
# ⚙️ CONFIGURATION
# ==========================================
SERIAL_PORT = 'COM12'
BAUD_RATE   = 115200

# Axis direction multipliers (for alignment correction)
ROLL_MULTIPLIER  = -1.0
PITCH_MULTIPLIER =  1.0

# Camera and cube layout settings
CAMERA_DISTANCE = -14.0
CUBE_SPACING    = 4.5
# ==========================================

# Global font objects (created once for performance)
font_title = None
font_data  = None

def main():
    global font_title, font_data

    # Initialize Pygame with OpenGL
    video_flags = OPENGL | DOUBLEBUF
    pygame.init()
    screen = pygame.display.set_mode((1920, 1080), video_flags)
    pygame.display.set_caption("STM32 - Realtime Sensor Fusion")

    resizewin(1920, 1080)
    init()

    # Create fonts only once (significant performance improvement)
    font_title = pygame.font.SysFont("Arial", 18, True)
    font_data  = pygame.font.SysFont("Arial", 16, True)

    # Serial port initialization
    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"SUCCESS: Connected to {SERIAL_PORT}.")
    except Exception as e:
        print(f"ERROR: Could not open {SERIAL_PORT}! ({e})")

    # Angle variables (Pitch, Roll) for:
    # 1) Accelerometer
    # 2) Gyroscope
    # 3) Complementary Filter
    p1, r1 = 0.0, 0.0
    p2, r2 = 0.0, 0.0
    p3, r3 = 0.0, 0.0

    # Clock for FPS control
    clock = pygame.time.Clock()

    while True:
        # Handle window events
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            break

        # --- SERIAL COMMUNICATION (OPTIMIZED) ---
        if ser and ser.is_open:
            try:
                # Read all available lines and keep only the most recent one
                # This prevents latency caused by buffered old data
                last_line = None
                while ser.in_waiting > 0:
                    try:
                        line = ser.readline()
                        if len(line) > 5:  # Ignore empty or invalid lines
                            last_line = line
                    except:
                        pass

                # Process the latest valid data packet
                if last_line:
                    decoded_line = last_line.decode('UTF-8', errors='ignore').strip()
                    data = decoded_line.split()

                    if len(data) >= 6:
                        # Group 1: Accelerometer-based angles
                        r1 = float(data[0]) * ROLL_MULTIPLIER
                        p1 = float(data[1]) * PITCH_MULTIPLIER

                        # Group 2: Gyroscope-based angles
                        r2 = float(data[2]) * ROLL_MULTIPLIER
                        p2 = float(data[3]) * PITCH_MULTIPLIER

                        # Group 3: Complementary filter output
                        r3 = float(data[4]) * ROLL_MULTIPLIER
                        p3 = float(data[5]) * PITCH_MULTIPLIER

            except ValueError:
                pass
            except Exception as e:
                print(f"Data Error: {e}")

        # --- RENDERING ---
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # Draw three cubes side-by-side
        draw_cube(-CUBE_SPACING, p1, r1, "ACCELEROMETER", font_title, font_data)
        draw_cube(0.0,          p2, r2, "GYROSCOPE",     font_title, font_data)
        draw_cube(CUBE_SPACING, p3, r3, "COMPLEMENTARY FILTER", font_title, font_data)

        pygame.display.flip()

        # Limit frame rate to 60 FPS to reduce CPU usage
        clock.tick(60)

    # Close serial port on exit
    if ser:
        ser.close()

def draw_cube(x_pos, pitch, roll, label, f_title, f_data):
    glLoadIdentity()
    glTranslatef(x_pos, 0.0, CAMERA_DISTANCE)

    # White color for text
    glColor3f(1.0, 1.0, 1.0)

    # Render labels and angle values
    drawText((-1.5,  2.0, 0), label, f_title)
    drawText((-1.2, -2.2, 0), f"P:{pitch:.1f}  R:{roll:.1f}", f_data)

    # Apply rotations (Roll around Z, Pitch around X)
    glRotatef(roll,  0.0, 0.0, 1.0)
    glRotatef(pitch, 1.0, 0.0, 0.0)

    # Draw cube using quads
    glBegin(GL_QUADS)

    glColor3f(0.0, 1.0, 0.0)
    glVertex3f( 1.0,  0.2, -1.0); glVertex3f(-1.0,  0.2, -1.0)
    glVertex3f(-1.0,  0.2,  1.0); glVertex3f( 1.0,  0.2,  1.0)

    glColor3f(1.0, 0.5, 0.0)
    glVertex3f( 1.0, -0.2,  1.0); glVertex3f(-1.0, -0.2,  1.0)
    glVertex3f(-1.0, -0.2, -1.0); glVertex3f( 1.0, -0.2, -1.0)

    glColor3f(1.0, 0.0, 0.0)
    glVertex3f( 1.0,  0.2,  1.0); glVertex3f(-1.0,  0.2,  1.0)
    glVertex3f(-1.0, -0.2,  1.0); glVertex3f( 1.0, -0.2,  1.0)

    glColor3f(1.0, 1.0, 0.0)
    glVertex3f( 1.0, -0.2, -1.0); glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0,  0.2, -1.0); glVertex3f( 1.0,  0.2, -1.0)

    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(-1.0,  0.2,  1.0); glVertex3f(-1.0,  0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0); glVertex3f(-1.0, -0.2,  1.0)

    glColor3f(1.0, 0.0, 1.0)
    glVertex3f( 1.0,  0.2, -1.0); glVertex3f( 1.0,  0.2,  1.0)
    glVertex3f( 1.0, -0.2,  1.0); glVertex3f( 1.0, -0.2, -1.0)

    glEnd()

def resizewin(width, height):
    if height == 0:
        height = 1

    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, width / height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.05, 0.05, 0.05, 1.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

def drawText(position, textString, fontObj):
    # Render text using pre-created font (avoids runtime overhead)
    textSurface = fontObj.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 0))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(
        textSurface.get_width(),
        textSurface.get_height(),
        GL_RGBA,
        GL_UNSIGNED_BYTE,
        textData
    )

if __name__ == '__main__':
    main()
