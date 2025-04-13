import os, sys, pygame, math, time
import numpy as np

from pygame.locals import (
    QUIT, MOUSEBUTTONDOWN, KEYDOWN, K_BACKSPACE, K_RETURN,
    K_0, K_1, K_2, K_3, K_4, K_5, K_6, K_7, K_8, K_9, K_PERIOD
)
from pygame.math import Vector2

# --- Constants ---
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
FPS = 144
PIXELS_PER_METER = 100 # Visual scaling
GROUND_Y = SCREEN_HEIGHT - 50
WALL_X = 10.0 # Use float for position calculations
INITIAL_SPEED_MPS = 1.0 # Initial speed of block 2 in meters per second
SIM_SPEED_FACTOR = 1.0 # Adjust to speed up/slow down simulation time visually
EPSILON = 1e-9 # Small number for float comparisons

# --- Physics Constants ---
COR = 1.0 # Coefficient of Restitution (MUST be 1.0 for Pi calculation)

# --- Dark Mode Colors ---
class Color:
    BACKGROUND = (30, 30, 30)
    GROUND_WALL = (80, 80, 80)
    TEXT = (230, 230, 230)
    TEXT_INPUT_ACTIVE = (255, 255, 255)
    BLOCK_RED = (210, 60, 60)
    BLOCK_YELLOW = (240, 190, 70)
    BUTTON_RUN = (0, 180, 0)
    BUTTON_RESET = (200, 100, 0)
    BUTTON_X100 = (60, 120, 180) # Bluish color for x100 button
    BUTTON_DISABLED = (100, 100, 100)
    INPUT_BOX_BG = (50, 50, 50)
    INPUT_BOX_BORDER = (150, 150, 150)
    INPUT_BOX_BORDER_ACTIVE = (255, 255, 255)

# --- Pygame and Mixer Setup ---
pygame.init()
try:
    pygame.mixer.init(frequency=22050, size=-16, channels=2, buffer=512)
    sound_enabled = True
except pygame.error:
    print("Warning: Pygame mixer could not be initialized. Sound disabled.")
    sound_enabled = False
pygame.font.init()

# --- Font Setup ---
# Increased font size for input boxes/labels
input_font = pygame.font.SysFont("Consolas", 20)
main_font = pygame.font.SysFont("Consolas", 18) # Keep for block mass text
button_font = pygame.font.SysFont("Arial", 20, bold=True)
info_font = pygame.font.SysFont("Arial", 20)
small_button_font = pygame.font.SysFont("Arial", 14, bold=True) # For x100 button

# --- Sound Generation & Playback ---
# (Sound functions remain the same)
click_sound = None
if sound_enabled:
    try:
        def generate_click_sound(frequency=1500, duration=0.03, sample_rate=22050):
            """Generates a simple click sound."""
            samples = int(sample_rate * duration)
            t = np.linspace(0., duration, samples, endpoint=False)
            wave = 0.3 * np.sin(2. * np.pi * frequency * t)
            decay = np.exp(-t / (duration / 4)) # Exponential decay
            wave *= decay
            wave_int16 = np.int16(wave * 32767)
            stereo_wave = np.repeat(wave_int16[:, np.newaxis], 2, axis=1) # Make it stereo
            sound = pygame.sndarray.make_sound(stereo_wave)
            return sound
        click_sound = generate_click_sound()
    except Exception as e:
        print(f"Warning: Could not generate click sound - {e}")
        sound_enabled = False

sound_play_times = {} # Dictionary to track last play time per object pair/wall
SOUND_COOLDOWN = 0.01 # Minimum time between sounds for the same collision (shorter for faster events)

def play_click_sound(obj1_id, obj2_id="wall"):
    """Plays the click sound if available and cooldown has passed."""
    if not (sound_enabled and click_sound):
        return
    # Use a simpler key for event-based sound triggering
    key = f"{obj1_id}-{obj2_id}"
    current_time = time.time() # Use real time for cooldown to prevent sound spam
    last_play = sound_play_times.get(key, 0)
    if current_time - last_play > SOUND_COOLDOWN:
        click_sound.play()
        sound_play_times[key] = current_time


# --- Input Box Class ---
# (InputBox class remains the same)
class InputBox:
    """A simple text input box for Pygame."""
    def __init__(self, x, y, w, h, initial_text='', font=input_font): # Use new input_font
        self.rect = pygame.Rect(x, y, w, h)
        self.color_border = Color.INPUT_BOX_BORDER
        self.text = initial_text
        self.font = font
        self.text_surface = self.font.render(self.text, True, Color.TEXT)
        self.active = False
        self.cursor_visible = False
        self.cursor_timer = 0
        self.padding = 8 # Increased padding

    def handle_event(self, event):
        """Handles events for the input box (clicks, key presses)."""
        if event.type == MOUSEBUTTONDOWN:
            if self.rect.collidepoint(event.pos):
                self.active = not self.active
            else:
                self.active = False
            self.color_border = Color.INPUT_BOX_BORDER_ACTIVE if self.active else Color.INPUT_BOX_BORDER
            self.cursor_visible = self.active
            self.cursor_timer = time.time()

        if event.type == KEYDOWN:
            if self.active:
                if event.key == K_BACKSPACE:
                    self.text = self.text[:-1]
                elif event.key == K_RETURN: # Allow Enter to deactivate
                    self.active = False
                    self.color_border = Color.INPUT_BOX_BORDER
                elif event.unicode.isdigit() or (event.key == K_PERIOD and '.' not in self.text):
                     # Allow input only if it fits (check against width minus padding)
                     if self.font.size(self.text + event.unicode)[0] < self.rect.width - (2 * self.padding):
                         self.text += event.unicode

                # Update text surface immediately after text change
                self.text_surface = self.font.render(self.text, True, Color.TEXT_INPUT_ACTIVE if self.active else Color.TEXT)
                self.cursor_visible = True
                self.cursor_timer = time.time()

    def update(self):
        """Updates the cursor blink."""
        if self.active:
            if time.time() - self.cursor_timer > 0.5:
                self.cursor_visible = not self.cursor_visible
                self.cursor_timer = time.time()
        else:
             self.cursor_visible = False

    def draw(self, screen):
        """Draws the input box on the screen."""
        pygame.draw.rect(screen, Color.INPUT_BOX_BG, self.rect)
        # Render text color based on active state and update surface
        self.text_surface = self.font.render(self.text, True, Color.TEXT_INPUT_ACTIVE if self.active else Color.TEXT)
        # Center text vertically, apply horizontal padding
        text_y = self.rect.y + (self.rect.height - self.text_surface.get_height()) // 2
        screen.blit(self.text_surface, (self.rect.x + self.padding, text_y))
        pygame.draw.rect(screen, self.color_border, self.rect, 2, border_radius=3)

        # Draw cursor if active
        if self.active and self.cursor_visible:
            cursor_x = self.rect.x + self.padding + self.text_surface.get_width()
            # Ensure cursor stays within bounds (consider padding)
            if cursor_x < self.rect.right - self.padding:
                # Adjust cursor vertical position to match text
                cursor_y = text_y
                cursor_height = self.font.get_height()
                pygame.draw.line(screen, Color.TEXT_INPUT_ACTIVE, (cursor_x, cursor_y), (cursor_x, cursor_y + cursor_height), 2)

    def get_value(self):
        """Returns the numeric value or 1.0 if invalid/empty."""
        try:
            # Handle potential large numbers from x100
            val = float(self.text)
            return val if val > 0 and math.isfinite(val) else 1.0 # Ensure mass is positive and finite
        except ValueError:
            return 1.0


# --- Block Class Definition (Simplified for Event-Based) ---
# (Block class remains the same)
class Block:
    """Represents a rectangular block with physical properties for event-based simulation."""
    _id_counter = 0 # Class variable for unique IDs

    def __init__(self, initial_x_meters, width_pixels, height_pixels, color, initial_mass_kg, font):
        self.id = Block._id_counter # Assign unique ID
        Block._id_counter += 1

        self.initial_x_meters = initial_x_meters # Store initial position in meters
        self.width = width_pixels
        self.height = height_pixels
        self.color = color
        self.mass = initial_mass_kg if initial_mass_kg > 0 else 1.0
        self.font = font # Use main_font for block text

        # Use floats for physics calculations
        self.position_x = 0.0 # Position of the left edge in pixels
        self.velocity_x = 0.0 # Velocity in pixels per second

        # Pygame rect for drawing
        self.rect = pygame.Rect(0, 0, self.width, self.height)
        self.text_surface = self.font.render("", True, Color.TEXT)
        self.text_rect = self.text_surface.get_rect()
        self.reset() # Set initial state

    def set_mass(self, mass_kg):
        """Updates the mass and rerenders the text."""
        self.mass = mass_kg if mass_kg > 0 else 1.0
        self._update_text() # Call internal text update

    def reset(self):
        """Resets the block to its initial state."""
        self.position_x = WALL_X + self.initial_x_meters * PIXELS_PER_METER
        self.velocity_x = 0.0
        self._update_rect()
        self._update_text() # Call internal text update

    def advance(self, time_step):
        """Moves the block forward in time by time_step."""
        self.position_x += self.velocity_x * time_step
        self._update_rect() # Update visual rect after moving

    def _update_rect(self):
        """Updates the Pygame rect based on the float position."""
        # Ensure block stays visually on the ground
        pos_y = GROUND_Y - self.height
        self.rect.topleft = (round(self.position_x), pos_y)
        # Update text position to follow rect
        self.text_rect.center = self.rect.center

    def _update_text(self):
        """Updates the mass text surface and rect."""
        # Format large masses potentially with scientific notation for display on block
        if self.mass > 1e4:
             mass_str = f"{self.mass:.1e} kg"
        else:
             mass_str = f"{self.mass:.1f} kg"
        self.text_surface = self.font.render(mass_str, True, Color.TEXT)
        self.text_rect = self.text_surface.get_rect(center=self.rect.center)
        # Ensure rect is updated if text changes size
        self._update_rect()


    def draw(self, surface):
        """Draws the block and its mass text."""
        self._update_rect() # Ensure rect is current before drawing
        pygame.draw.rect(surface, self.color, self.rect, border_radius=5)
        surface.blit(self.text_surface, self.text_rect)


# --- Collision Calculation Functions ---
# (Collision functions remain the same)
def time_to_wall_collision(block):
    """Calculates time until the block hits the left wall (WALL_X)."""
    if block.velocity_x >= 0: # Moving right or stationary
        return float('inf')
    # Wall is at WALL_X, block left edge is at position_x
    distance_to_wall = block.position_x - WALL_X
    if distance_to_wall <= EPSILON: # Already touching or past
        return 0.0
    # Avoid division by zero if velocity is extremely small negative
    if abs(block.velocity_x) < EPSILON:
        return float('inf')
    return distance_to_wall / abs(block.velocity_x)

def time_to_block_collision(block1, block2):
    """Calculates time until block1 (left) and block2 (right) collide."""
    # Ensure block1 is the one on the left for calculation consistency
    # This function assumes block1 and block2 are passed in a fixed order
    # Let's re-evaluate based on current positions each time
    b1, b2 = (block1, block2) if block1.position_x < block2.position_x else (block2, block1)

    relative_velocity = b2.velocity_x - b1.velocity_x
    # If moving apart or parallel (or same velocity), no collision
    if relative_velocity >= -EPSILON:
        return float('inf')

    # Collision happens when right edge of b1 meets left edge of b2
    gap = b2.position_x - (b1.position_x + b1.width)
    if gap <= EPSILON: # Already touching or overlapping
        return 0.0

    # Avoid division by zero
    if abs(relative_velocity) < EPSILON:
        return float('inf')

    return gap / abs(relative_velocity)

def resolve_wall_collision(block):
    """Applies elastic collision with the wall."""
    global collision_count
    # Perfectly elastic collision: velocity reverses
    block.velocity_x = -block.velocity_x * COR # COR should be 1.0
    collision_count += 1
    play_click_sound(block.id, "wall")
    # Small position correction to ensure it's off the wall
    block.position_x = max(block.position_x, WALL_X + EPSILON)
    block._update_rect() # Update rect immediately after position change


def resolve_block_collision(block1, block2):
    """Applies 1D elastic collision between two blocks."""
    global collision_count
    m1, m2 = block1.mass, block2.mass
    v1_initial, v2_initial = block1.velocity_x, block2.velocity_x

    # Check for NaN or Inf masses (shouldn't happen with get_value check, but safety)
    if not (math.isfinite(m1) and math.isfinite(m2) and m1 > 0 and m2 > 0):
        print(f"Warning: Invalid mass detected during collision ({m1}, {m2}). Skipping resolution.")
        return

    # Prevent division by zero if total mass is zero (shouldn't happen)
    if m1 + m2 < EPSILON:
         print(f"Warning: Zero total mass detected during collision. Skipping resolution.")
         return

    # Formulas for 1D elastic collision (COR = 1.0)
    v1_final = ((m1 - m2) * v1_initial + (2 * m2) * v2_initial) / (m1 + m2)
    v2_final = ((2 * m1) * v1_initial + (m2 - m1) * v2_initial) / (m1 + m2)

    block1.velocity_x = v1_final
    block2.velocity_x = v2_final
    collision_count += 1
    play_click_sound(block1.id, block2.id)

    # Small position correction to avoid sticking/overlap after velocity change
    # Determine which block is left and right *now*
    b1, b2 = (block1, block2) if block1.position_x < block2.position_x else (block2, block1)
    overlap = (b1.position_x + b1.width) - b2.position_x
    if overlap > -EPSILON: # If overlapping or touching
        # Separate them slightly based on overlap
        correction = (overlap / 2.0) + EPSILON # Add epsilon to ensure separation
        b1.position_x -= correction
        b2.position_x += correction
        b1._update_rect() # Update rects immediately
        b2._update_rect()


# --- Simulation State & UI Setup ---
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
clock = pygame.time.Clock()
pygame.display.set_caption("Pi Calculation via Colliding Blocks")

# Create Blocks, Input Boxes, Buttons
block1 = Block(initial_x_meters=2.0, width_pixels=50, height_pixels=50, color=Color.BLOCK_RED, initial_mass_kg=1.0, font=main_font)
block2 = Block(initial_x_meters=4.0, width_pixels=80, height_pixels=80, color=Color.BLOCK_YELLOW, initial_mass_kg=100.0, font=main_font)
blocks = [block1, block2]

# Input Boxes Layout
input_box_y = 35
input_box_h = 35
input_box1 = InputBox(50, input_box_y, 110, input_box_h, initial_text=f"{block1.mass:.1f}", font=input_font)
input_box2 = InputBox(200, input_box_y, 110, input_box_h, initial_text=f"{block2.mass:.1f}", font=input_font)
input_boxes = [input_box1, input_box2]

# x100 Button Layout
x100_button_w = 45
x100_button_h = input_box_h
x100_button_x = input_box2.rect.right + 10
x100_button_rect = pygame.Rect(x100_button_x, input_box_y, x100_button_w, x100_button_h)

# Run/Reset Buttons Layout - Positioned below input boxes
button_width = 80
button_height = 40
# Calculate new Y position below input boxes + margin
button_y = input_box_y + input_box_h + 20
run_button_rect = pygame.Rect(SCREEN_WIDTH // 2 - button_width - 260, button_y, button_width, button_height)
reset_button_rect = pygame.Rect(SCREEN_WIDTH // 2 - 240, button_y, button_width, button_height)


# Game State
simulation_running = False
simulation_finished = False
collision_count = 0

# --- Reset Function ---
# (Reset function remains the same)
def reset_simulation():
    """Resets the simulation to the initial state based on input boxes."""
    global simulation_running, simulation_finished, collision_count, sound_play_times, block1, block2
    simulation_running = False
    simulation_finished = False # Reset this flag too
    collision_count = 0
    Block._id_counter = 0 # Reset block IDs
    sound_play_times = {} # Reset sound cooldown tracking

    # Get masses from input boxes
    mass1 = input_box1.get_value()
    mass2 = input_box2.get_value()

    # Re-create blocks to ensure clean state and correct IDs
    # Pass main_font explicitly for blocks
    block1 = Block(initial_x_meters=2.0, width_pixels=50, height_pixels=50, color=Color.BLOCK_RED, initial_mass_kg=mass1, font=main_font)
    block2 = Block(initial_x_meters=4.0, width_pixels=80, height_pixels=80, color=Color.BLOCK_YELLOW, initial_mass_kg=mass2, font=main_font)
    blocks = [block1, block2]

    # Update input box text to reflect actual masses used (handles invalid input)
    # Format potentially large masses for input box display
    if mass1 > 1e6: mass1_str = f"{mass1:.1e}"
    else: mass1_str = f"{mass1:.1f}"
    if mass2 > 1e6: mass2_str = f"{mass2:.1e}"
    else: mass2_str = f"{mass2:.1f}"
    input_box1.text = mass1_str
    input_box2.text = mass2_str
    # No need to call _update_text here, draw method handles it.

    # Deactivate input boxes
    for box in input_boxes:
        box.active = False
        box.color_border = Color.INPUT_BOX_BORDER

# --- Initial Reset ---
reset_simulation()

# --- Game Loop ---
runtime = True
while runtime:
    # Time elapsed since last frame in seconds
    frame_dt = clock.tick(FPS) / 1000.0 * SIM_SPEED_FACTOR
    frame_dt = min(frame_dt, 0.1) # Cap frame time


    # --- Event Handling ---
    # (Event handling logic remains the same, including x100 button)
    for event in pygame.event.get():
        if event.type == QUIT:
            runtime = False

        # Allow input box interaction only if simulation is NOT running
        if not simulation_running and not simulation_finished:
            for box in input_boxes:
                box.handle_event(event)

        if event.type == MOUSEBUTTONDOWN:
            # Check buttons only if sim not running/finished
            if not simulation_running and not simulation_finished:
                # RUN Button
                if run_button_rect.collidepoint(event.pos):
                    reset_simulation() # Reset with current masses from input boxes
                    simulation_running = True
                    # Initial conditions: block1 stationary, block2 moving left
                    block1.velocity_x = 0.0
                    block2.velocity_x = -INITIAL_SPEED_MPS * PIXELS_PER_METER
                    # Deactivate input boxes visually
                    for box in input_boxes:
                        box.active = False
                        box.color_border = Color.INPUT_BOX_BORDER
                # x100 Button
                elif x100_button_rect.collidepoint(event.pos):
                    current_val = input_box2.get_value()
                    new_val = current_val * 100.0
                    # Format for display, potentially use scientific notation
                    if new_val > 1e6: # Threshold for scientific notation
                        input_box2.text = f"{new_val:.1e}"
                    else:
                        input_box2.text = f"{new_val:.1f}"
                    # Deactivate box after button press
                    input_box2.active = False
                    input_box2.color_border = Color.INPUT_BOX_BORDER

            # RESET Button: Always active
            if reset_button_rect.collidepoint(event.pos):
                reset_simulation()


    # --- Input Box Updates (Cursor blink) ---
    # (Input box update logic remains the same)
    for box in input_boxes:
        box.update()

    # --- Physics Update (Event-Based) ---
    # (Physics update logic remains the same)
    if simulation_running:
        time_remaining_in_frame = frame_dt
        collision_this_step = False # Flag to detect if a collision happened

        # Process multiple collisions within a single frame if necessary
        max_steps_per_frame = 1000000 # Safety break limit
        steps_processed = 0

        while time_remaining_in_frame > EPSILON and steps_processed < max_steps_per_frame:
            steps_processed += 1
            collision_this_step = False # Reset flag for this sub-step

            # Check stopping condition (condition met -> set finished flag but don't stop)
            if not simulation_finished: # Only check if not already finished
                if block1.velocity_x >= -EPSILON and block2.velocity_x >= block1.velocity_x - EPSILON:
                     if block2.position_x > block1.position_x + block1.width - EPSILON:
                        print(f"Condition for last collision met. Collision Count: {collision_count}. Simulation continues visually.")
                        simulation_finished = True # Set flag to prevent further collision checks/resolutions

            # Calculate time to next potential collision ONLY IF NOT FINISHED
            min_time_to_collision = float('inf')
            if not simulation_finished:
                t_wall = time_to_wall_collision(block1)
                t_blocks = time_to_block_collision(block1, block2)
                min_time_to_collision = min(t_wall, t_blocks)

            # If no collision will happen or collision time is beyond remaining frame time
            if min_time_to_collision == float('inf') or min_time_to_collision > time_remaining_in_frame + EPSILON :
                if time_remaining_in_frame > EPSILON:
                    block1.advance(time_remaining_in_frame)
                    block2.advance(time_remaining_in_frame)
                time_remaining_in_frame = 0 # Exit while loop
            else:
                # Advance blocks to the point of collision
                advance_time = max(0, min_time_to_collision)
                if advance_time > EPSILON:
                    block1.advance(advance_time)
                    block2.advance(advance_time)

                # Resolve the collision that occurred (only if not finished)
                if not simulation_finished:
                    if abs(min_time_to_collision - t_wall) < EPSILON:
                        resolve_wall_collision(block1)
                        collision_this_step = True
                    elif abs(min_time_to_collision - t_blocks) < EPSILON :
                         gap = block2.position_x - (block1.position_x + block1.width)
                         if gap < PIXELS_PER_METER * 0.02 :
                            resolve_block_collision(block1, block2)
                            collision_this_step = True

                # Decrease the time remaining in the frame
                time_remaining_in_frame -= advance_time

                # Anti-stuck logic if no collision resolved
                if not collision_this_step and time_remaining_in_frame > EPSILON and not simulation_finished:
                     tiny_advance = min(time_remaining_in_frame, EPSILON * 10)
                     block1.advance(tiny_advance)
                     block2.advance(tiny_advance)
                     time_remaining_in_frame -= tiny_advance


        # End of inner while loop
        if steps_processed >= max_steps_per_frame:
             print(f"Warning: Max physics steps ({max_steps_per_frame}) per frame reached.")

        # Ensure final positions are drawn correctly
        block1._update_rect()
        block2._update_rect()


    # --- Drawing ---
    screen.fill(Color.BACKGROUND)

    # Draw Ground and Wall
    pygame.draw.line(screen, Color.GROUND_WALL, (0, GROUND_Y), (SCREEN_WIDTH, GROUND_Y), 5)
    pygame.draw.line(screen, Color.GROUND_WALL, (WALL_X, 0), (WALL_X, GROUND_Y), 5)

    # Draw Blocks
    block2.draw(screen)
    block1.draw(screen)

    # Draw UI Elements
    # Labels
    label_y = input_box_y - 20
    screen.blit(input_font.render("Mass 1 (kg):", True, Color.TEXT), (input_box1.rect.x, label_y))
    input_box1.draw(screen)
    screen.blit(input_font.render("Mass 2 (kg):", True, Color.TEXT), (input_box2.rect.x, label_y))
    input_box2.draw(screen)

    # Draw x100 Button
    x100_button_color = Color.BUTTON_X100 if not simulation_running and not simulation_finished else Color.BUTTON_DISABLED
    pygame.draw.rect(screen, x100_button_color, x100_button_rect, border_radius=5)
    x100_text = small_button_font.render("x100", True, Color.BACKGROUND if x100_button_color != Color.BUTTON_DISABLED else Color.GROUND_WALL)
    x100_text_rect = x100_text.get_rect(center=x100_button_rect.center)
    screen.blit(x100_text, x100_text_rect)


    # Draw Run/Reset Buttons (Now below inputs)
    run_button_color = Color.BUTTON_RUN if not simulation_running and not simulation_finished else Color.BUTTON_DISABLED
    pygame.draw.rect(screen, run_button_color, run_button_rect, border_radius=5)
    run_text = button_font.render("RUN", True, Color.BACKGROUND if run_button_color != Color.BUTTON_DISABLED else Color.GROUND_WALL)
    run_text_rect = run_text.get_rect(center=run_button_rect.center)
    screen.blit(run_text, run_text_rect)

    reset_button_color = Color.BUTTON_RESET
    pygame.draw.rect(screen, reset_button_color, reset_button_rect, border_radius=5)
    reset_text = button_font.render("RESET", True, Color.BACKGROUND)
    reset_text_rect = reset_text.get_rect(center=reset_button_rect.center)
    screen.blit(reset_text, reset_text_rect)

    # Draw Collision Count (Aligned with new button position)
    collision_text = info_font.render(f"Collisions: {collision_count}", True, Color.TEXT)
    # Align topright with the space to the right of the buttons
    collision_text_rect = collision_text.get_rect(topright=(SCREEN_WIDTH - 20, button_y + 5))
    screen.blit(collision_text, collision_text_rect)

    # Update the display
    pygame.display.flip()

# --- Quit Pygame ---
if sound_enabled:
    pygame.mixer.quit()
pygame.font.quit()
pygame.quit()
# sys.exit()
