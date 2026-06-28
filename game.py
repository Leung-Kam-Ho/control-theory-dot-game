"""
PID Chase Game — Learn PID Control by Playing
==============================================
A chaser (blue) pursues a target (red) using a PID controller.
Students adjust Kp, Ki, Kd to see how each parameter affects behavior.

Controls:
  Arrow keys / WASD — move the target
  + / - on Kp, Ki, Kd — adjust parameters
  R — reset to defaults
  Space — pause/resume
  Q — quit

Educational goal: Understand how P, I, and D terms work together
to control a system's response to error.
"""

import pygame
import math
import sys

# ── Constants ──────────────────────────────────────────────────────────
SCREEN_WIDTH = 1200
SCREEN_HEIGHT = 800
FPS = 60

# Colors
BG_COLOR = (20, 22, 30)
GRID_COLOR = (35, 40, 55)
TARGET_COLOR = (220, 50, 50)       # Red
CHASER_COLOR = (50, 150, 255)     # Blue
ERROR_COLOR = (255, 200, 50)      # Yellow
TRAIL_COLOR = (50, 150, 255, 80)
TEXT_COLOR = (220, 220, 230)
ACCENT_COLOR = (100, 255, 180)
P_COLOR = (255, 100, 100)
I_COLOR = (100, 200, 255)
D_COLOR = (100, 255, 100)

# Chaser physics
CHASER_RADIUS = 25
TARGET_RADIUS = 18
CHASE_SPEED_MAX = 500  # pixels per second
TRAIL_LENGTH = 120
ERROR_LINE_ALPHA = 180

# PID defaults
DEFAULT_KP = 0.8
DEFAULT_KI = 0.05
DEFAULT_KD = 0.15


class PIDController:
    """Simple PID controller for tracking."""

    def __init__(self, kp=DEFAULT_KP, ki=DEFAULT_KI, kd=DEFAULT_KD):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_time = None

    def reset(self):
        self.integral_x = 0.0
        self.integral_y = 0.0
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_time = None

    def update(self, error_x, error_y, dt):
        """Compute control output given position error and timestep."""
        # Proportional term
        p_out_x = self.kp * error_x
        p_out_y = self.kp * error_y

        # Integral term (with anti-windup)
        self.integral_x += error_x * dt
        self.integral_y += error_y * dt
        max_integral = 500.0
        self.integral_x = max(-max_integral, min(max_integral, self.integral_x))
        self.integral_y = max(-max_integral, min(max_integral, self.integral_y))
        i_out_x = self.ki * self.integral_x
        i_out_y = self.ki * self.integral_y

        # Derivative term
        if self.prev_time is not None and dt > 0:
            d_out_x = self.kd * (error_x - self.prev_error_x) / dt
            d_out_y = self.kd * (error_y - self.prev_error_y) / dt
        else:
            d_out_x = 0.0
            d_out_y = 0.0

        # Store current error for next iteration
        self.prev_error_x = error_x
        self.prev_error_y = error_y
        self.prev_time = pygame.time.get_ticks() / 1000.0

        # Combine terms
        control_x = p_out_x + i_out_x + d_out_x
        control_y = p_out_y + i_out_y + d_out_y

        # Clamp to max speed
        magnitude = math.sqrt(control_x ** 2 + control_y ** 2)
        if magnitude > CHASE_SPEED_MAX:
            scale = CHASE_SPEED_MAX / magnitude
            control_x *= scale
            control_y *= scale

        return control_x, control_y


class Chaser:
    """The PID-controlled chaser entity."""

    def __init__(self, x, y):
        self.pos = [x, y]
        self.vel = [0.0, 0.0]
        self.pid = PIDController()
        self.trail = []
        self.radius = CHASER_RADIUS

    def update(self, target_pos, dt):
        """Update chaser position using PID control."""
        error_x = target_pos[0] - self.pos[0]
        error_y = target_pos[1] - self.pos[1]

        control = self.pid.update(error_x, error_y, dt)

        self.vel[0] += control[0] * dt
        self.vel[1] += control[1] * dt

        # Clamp velocity
        speed = math.sqrt(self.vel[0] ** 2 + self.vel[1] ** 2)
        if speed > CHASE_SPEED_MAX:
            self.vel[0] = (self.vel[0] / speed) * CHASE_SPEED_MAX
            self.vel[1] = (self.vel[1] / speed) * CHASE_SPEED_MAX

        self.pos[0] += self.vel[0] * dt
        self.pos[1] += self.vel[1] * dt

        # Record trail
        self.trail.append((self.pos[0], self.pos[1]))
        if len(self.trail) > TRAIL_LENGTH:
            self.trail.pop(0)

    def set_pid(self, kp, ki, kd):
        self.pid.kp = kp
        self.pid.ki = ki
        self.pid.kd = kd
        self.pid.reset()

    def get_error(self, target_pos):
        return (target_pos[0] - self.pos[0], target_pos[1] - self.pos[1])

    def get_p_term(self, error_x):
        return self.pid.kp * error_x

    def get_i_term(self):
        return self.pid.ki * self.pid.integral_x

    def get_d_term(self, error_x):
        if self.pid.prev_time is not None and self.pid.prev_error_x is not None:
            dt = (pygame.time.get_ticks() / 1000.0 - self.pid.prev_time)
            if dt > 0:
                return self.pid.kd * (error_x - self.pid.prev_error_x) / dt
        return 0.0

    def reset(self):
        self.pos = [SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2]
        self.vel = [0.0, 0.0]
        self.pid.reset()
        self.trail.clear()


class Target:
    """The target that the chaser pursues."""

    def __init__(self, x, y):
        self.pos = [x, y]
        self.radius = TARGET_RADIUS
        self.manual_mode = True  # Player controls target
        self.docked_to_mouse = True  # Target follows mouse cursor
        self.auto_velocity = [0.3, 0.2]
        self.auto_phase = [0.0, 0.0]
        self.auto_amplitude = [200, 150]

    def set_auto(self, enabled):
        self.manual_mode = not enabled

    def toggle_dock(self):
        self.docked_to_mouse = not self.docked_to_mouse
        return self.docked_to_mouse

    def update(self, dt):
        if self.manual_mode:
            return
        # Auto mode: move in a figure-8 pattern
        self.auto_phase[0] += dt * 0.8
        self.auto_phase[1] += dt * 0.6
        self.pos[0] = SCREEN_WIDTH / 2 + math.sin(self.auto_phase[0]) * self.auto_amplitude[0]
        self.pos[1] = SCREEN_HEIGHT / 2 + math.sin(self.auto_phase[1]) * self.auto_amplitude[1]

        # Bounce off walls
        margin = 60
        if self.pos[0] < margin:
            self.pos[0] = margin
            self.auto_velocity[0] = abs(self.auto_velocity[0])
        if self.pos[0] > SCREEN_WIDTH - margin:
            self.pos[0] = SCREEN_WIDTH - margin
            self.auto_velocity[0] = -abs(self.auto_velocity[0])
        if self.pos[1] < margin:
            self.pos[1] = margin
            self.auto_velocity[1] = abs(self.auto_velocity[1])
        if self.pos[1] > SCREEN_HEIGHT - margin:
            self.pos[1] = SCREEN_HEIGHT - margin
            self.auto_velocity[1] = -abs(self.auto_velocity[1])

    def get_error_vector(self, chaser_pos):
        return (chaser_pos[0] - self.pos[0], chaser_pos[1] - self.pos[1])


class Game:
    """Main game state machine."""

    def __init__(self):
        pygame.init()
        pygame.display.set_caption("PID Chase Game — Learn PID Control")
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("monospace", 18, bold=True)
        self.small_font = pygame.font.SysFont("monospace", 14)
        self.big_font = pygame.font.SysFont("monospace", 36, bold=True)

        self.target = Target(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 3)
        self.chaser = Chaser(SCREEN_WIDTH / 2, SCREEN_HEIGHT * 3 / 4)
        self.paused = False
        self.running = True
        self.captured = False
        self.capture_timer = 0
        self.captured_count = 0

        # PID display bounds
        self.pid_panel_x = SCREEN_WIDTH - 340
        self.pid_panel_y = 30
        self.pid_panel_w = 310
        self.pid_panel_h = 480

        # Slider state
        self._slider_active = None  # 'kp', 'ki', or 'kd'
        self._slider_track_rects = []  # list of (x, y, w, h) for hit testing

        # Error display
        self.error_panel_x = 15
        self.error_panel_y = 30
        self.error_panel_w = 310
        self.error_panel_h = 480

        # Instructions panel
        self.instruction_panel_x = SCREEN_WIDTH // 2 - 300
        self.instruction_panel_y = SCREEN_HEIGHT - 180
        self.instruction_panel_w = 600
        self.instruction_panel_h = 150

        # Graph
        self.graph_x = 15
        self.graph_y = 530
        self.graph_w = 500
        self.graph_h = 250
        self.error_history = []
        self.max_history = 300

    def reset(self):
        self.chaser.reset()
        self.target.auto_phase = [0.0, 0.0]
        self.captured = False
        self.captured_count = 0
        self.error_history.clear()

    def run(self):
        while self.running:
            dt = self.clock.tick(FPS) / 1000.0
            self.handle_events()
            if not self.paused:
                self.update(dt)
            self.draw()
        pygame.quit()
        sys.exit()

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q or event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_SPACE:
                    self.paused = not self.paused
                elif event.key == pygame.K_r:
                    self.reset()
                elif event.key == pygame.K_t:
                    self.target.set_auto(not self.target.manual_mode)
                elif event.key == pygame.K_TAB:
                    docked = self.target.toggle_dock()
                    # Steal focus so the target doesn't jump to cursor on toggle
                    pygame.event.post(pygame.event.Event(pygame.MOUSEMOTION, {'pos': self.target.pos}))
            elif event.type == pygame.MOUSEMOTION:
                if self.target.manual_mode and self.target.docked_to_mouse:
                    self.target.pos[0] = event.pos[0]
                    self.target.pos[1] = event.pos[1]
                # Slider drag
                if self._slider_active is not None:
                    mx, my = event.pos
                    track_rects = self._slider_track_rects
                    if 0 <= self._slider_active < len(track_rects):
                        tx, ty, tw, th, _ = track_rects[self._slider_active]
                        # Clamp to track bounds
                        t = max(0.0, min(1.0, (mx - tx) / tw))
                        # Map to parameter range
                        if self._slider_active == 0:  # Kp
                            self.chaser.pid.kp = 0.01 + t * (5.0 - 0.01)
                        elif self._slider_active == 1:  # Ki
                            self.chaser.pid.ki = t * 2.0
                        elif self._slider_active == 2:  # Kd
                            self.chaser.pid.kd = t * 2.0
                        self.chaser.pid.reset()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 1:  # Left click
                    mx, my = event.pos
                    for i, (tx, ty, tw, th, key) in enumerate(self._slider_track_rects):
                        # Check if click is near the track (with some padding)
                        if tx - 10 <= mx <= tx + tw + 10 and ty - 15 <= my <= ty + th + 15:
                            self._slider_active = i
                            break
            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    self._slider_active = None

    def update(self, dt):
        # Target follows mouse cursor (set in handle_events via MOUSEMOTION)
        # Clamp target to screen
        self.target.pos[0] = max(self.target.radius, min(SCREEN_WIDTH - self.target.radius, self.target.pos[0]))
        self.target.pos[1] = max(self.target.radius, min(SCREEN_HEIGHT - self.target.radius, self.target.pos[1]))

        self.target.update(dt)
        self.chaser.update(self.target.pos, dt)

        # Check capture
        dist = math.sqrt((self.chaser.pos[0] - self.target.pos[0]) ** 2 +
                         (self.chaser.pos[1] - self.target.pos[1]) ** 2)
        if dist < (self.chaser.radius + self.target.radius) * 0.5:
            if not self.captured:
                self.captured = True
                self.capture_timer = pygame.time.get_ticks()
                self.captured_count += 1

        # Record error history
        error = self.chaser.get_error(self.target.pos)
        error_mag = math.sqrt(error[0] ** 2 + error[1] ** 2)
        self.error_history.append(error_mag)
        if len(self.error_history) > self.max_history:
            self.error_history.pop(0)

    def draw(self):
        self.screen.fill(BG_COLOR)
        self.draw_grid()
        self.draw_trail()
        self.draw_error_line()
        self.draw_chaser()
        self.draw_target()
        self.draw_pid_panel()
        self.draw_error_panel()
        self.draw_graph()
        self.draw_instructions()
        self.draw_capture_indicator()
        pygame.display.flip()

    def draw_grid(self):
        for x in range(0, SCREEN_WIDTH, 50):
            pygame.draw.line(self.screen, GRID_COLOR, (x, 0), (x, SCREEN_HEIGHT), 1)
        for y in range(0, SCREEN_HEIGHT, 50):
            pygame.draw.line(self.screen, GRID_COLOR, (0, y), (SCREEN_WIDTH, y), 1)

    def draw_trail(self):
        if len(self.chaser.trail) < 2:
            return
        for i in range(1, len(self.chaser.trail)):
            alpha = int(255 * i / len(self.chaser.trail))
            width = max(1, int(4 * i / len(self.chaser.trail)))
            color = (*TRAIL_COLOR[:3], alpha)
            # Convert to solid color for pygame
            solid_color = tuple(int(c * alpha / 255) for c in TRAIL_COLOR[:3])
            pygame.draw.line(self.screen, solid_color, self.chaser.trail[i - 1], self.chaser.trail[i], width)

    def draw_error_line(self):
        error = self.chaser.get_error(self.target.pos)
        error_mag = math.sqrt(error[0] ** 2 + error[1] ** 2)
        if error_mag > 5:
            alpha_surf = pygame.Surface((int(error_mag), 3), pygame.SRCALPHA)
            pygame.draw.line(alpha_surf, (*ERROR_COLOR, ERROR_LINE_ALPHA), (0, 0), (int(error_mag), 0), 2)
            angle = math.atan2(error[1], error[0])
            rotated = pygame.transform.rotate(alpha_surf, -math.degrees(angle))
            rect = rotated.get_rect(center=tuple((self.chaser.pos[i] + self.target.pos[i]) / 2 for i in range(2)))
            self.screen.blit(rotated, rect.topleft)

    def draw_chaser(self):
        # Glow
        glow_surf = pygame.Surface((CHASER_RADIUS * 4, CHASER_RADIUS * 4), pygame.SRCALPHA)
        pygame.draw.circle(glow_surf, (*CHASER_COLOR, 40), (CHASER_RADIUS * 2, CHASER_RADIUS * 2), CHASER_RADIUS * 2)
        self.screen.blit(glow_surf, (self.chaser.pos[0] - CHASER_RADIUS * 2,
                                       self.chaser.pos[1] - CHASER_RADIUS * 2))
        # Body
        pygame.draw.circle(self.screen, CHASER_COLOR, (int(self.chaser.pos[0]), int(self.chaser.pos[1])),
                           self.chaser.radius)
        pygame.draw.circle(self.screen, (255, 255, 255), (int(self.chaser.pos[0]), int(self.chaser.pos[1])),
                           self.chaser.radius, 2)
        # Label
        label = self.font.render("CHASER", True, CHASER_COLOR)
        self.screen.blit(label, (self.chaser.pos[0] - 30, self.chaser.pos[1] + 35))

    def draw_target(self):
        # Glow
        glow_surf = pygame.Surface((TARGET_RADIUS * 4, TARGET_RADIUS * 4), pygame.SRCALPHA)
        pygame.draw.circle(glow_surf, (*TARGET_COLOR, 40), (TARGET_RADIUS * 2, TARGET_RADIUS * 2), TARGET_RADIUS * 2)
        self.screen.blit(glow_surf, (self.target.pos[0] - TARGET_RADIUS * 2,
                                       self.target.pos[1] - TARGET_RADIUS * 2))
        # Body
        pygame.draw.circle(self.screen, TARGET_COLOR, (int(self.target.pos[0]), int(self.target.pos[1])),
                           self.target.radius)
        pygame.draw.circle(self.screen, (255, 255, 255), (int(self.target.pos[0]), int(self.target.pos[1])),
                           self.target.radius, 2)
        # Label
        label = self.font.render("TARGET", True, TARGET_COLOR)
        self.screen.blit(label, (self.target.pos[0] - 30, self.target.pos[1] + 35))

    def draw_pid_panel(self):
        # Background
        panel_surf = pygame.Surface((self.pid_panel_w, self.pid_panel_h), pygame.SRCALPHA)
        pygame.draw.rect(panel_surf, (30, 35, 50, 220), (0, 0, self.pid_panel_w, self.pid_panel_h), border_radius=10)
        self.screen.blit(panel_surf, (self.pid_panel_x, self.pid_panel_y))

        # Title
        title = self.big_font.render("PID CONTROLLER", True, TEXT_COLOR)
        self.screen.blit(title, (self.pid_panel_x + 20, self.pid_panel_y + 15))

        # Slider config
        slider_x = self.pid_panel_x + 20
        slider_w = self.pid_panel_w - 40
        slider_h = 20
        track_h = 8
        knob_r = 12
        params = [
            ("Kp", self.chaser.pid.kp, 0.01, 5.0, P_COLOR, "kp"),
            ("Ki", self.chaser.pid.ki, 0.0, 2.0, I_COLOR, "ki"),
            ("Kd", self.chaser.pid.kd, 0.0, 2.0, D_COLOR, "kd"),
        ]
        self._slider_track_rects = []

        y = self.pid_panel_y + 60
        for label, value, vmin, vmax, color, key in params:
            # Label + value
            lbl = self.font.render(f"{label}:", True, color)
            val = self.small_font.render(f"{value:.3f}", True, TEXT_COLOR)
            self.screen.blit(lbl, (slider_x, y))
            self.screen.blit(val, (slider_x + slider_w - 70, y))

            # Track (background bar)
            track_y = y + 14
            track_rect = pygame.Rect(slider_x, track_y, slider_w, track_h)
            pygame.draw.rect(self.screen, (60, 65, 80), track_rect, border_radius=4)
            self._slider_track_rects.append((slider_x, track_y, slider_w, track_h, key))

            # Fill (colored portion)
            fill_ratio = (value - vmin) / (vmax - vmin) if vmax != vmin else 0
            fill_w = max(0, int(fill_ratio * slider_w))
            if fill_w > 0:
                fill_rect = pygame.Rect(slider_x, track_y, fill_w, track_h)
                pygame.draw.rect(self.screen, (*color, 180), fill_rect, border_radius=4)

            # Knob
            knob_x = slider_x + fill_ratio * slider_w
            knob_color = color if self._slider_active == key else tuple(min(c + 40, 255) for c in color)
            pygame.draw.circle(self.screen, knob_color, (int(knob_x), int(track_y + track_h / 2)), knob_r)
            pygame.draw.circle(self.screen, (255, 255, 255), (int(knob_x), int(track_y + track_h / 2)), knob_r, 2)

            y += 55

        # Legend
        y += 10
        auto_label = self.small_font.render("T — Toggle auto target", True, ACCENT_COLOR)
        self.screen.blit(auto_label, (self.pid_panel_x + 15, y))
        y += 25
        reset_label = self.small_font.render("R — Reset chaser", True, ACCENT_COLOR)
        self.screen.blit(reset_label, (self.pid_panel_x + 15, y))
        y += 25
        pause_label = self.small_font.render("Space — Pause", True, ACCENT_COLOR)
        self.screen.blit(pause_label, (self.pid_panel_x + 15, y))

    def draw_error_panel(self):
        # Background
        panel_surf = pygame.Surface((self.error_panel_w, self.error_panel_h), pygame.SRCALPHA)
        pygame.draw.rect(panel_surf, (30, 35, 50, 220), (0, 0, self.error_panel_w, self.error_panel_h), border_radius=10)
        self.screen.blit(panel_surf, (self.error_panel_x, self.error_panel_y))

        title = self.big_font.render("ERROR ANALYSIS", True, TEXT_COLOR)
        self.screen.blit(title, (self.error_panel_x + 10, self.error_panel_y + 15))

        error = self.chaser.get_error(self.target.pos)
        error_mag = math.sqrt(error[0] ** 2 + error[1] ** 2)

        y = self.error_panel_y + 60
        line_h = 30

        errors = [
            ("Distance:", f"{error_mag:.1f} px", TEXT_COLOR),
            ("ΔX:", f"{error[0]:+.1f} px", TEXT_COLOR),
            ("ΔY:", f"{error[1]:+.1f} px", TEXT_COLOR),
            ("P Term:", f"{self.chaser.get_p_term(error[0]):+.1f}", P_COLOR),
            ("I Term:", f"{self.chaser.get_i_term():+.1f}", I_COLOR),
            ("D Term:", f"{self.chaser.get_d_term(error[0]):+.1f}", D_COLOR),
        ]

        for label, value, color in errors:
            lbl = self.small_font.render(label, True, color)
            val = self.small_font.render(value, True, TEXT_COLOR)
            self.screen.blit(lbl, (self.error_panel_x + 15, y))
            self.screen.blit(val, (self.error_panel_x + 180, y))
            y += line_h

        # Capture counter
        y += 10
        cap_text = self.small_font.render(f"Captures: {self.captured_count}", True, ACCENT_COLOR)
        self.screen.blit(cap_text, (self.error_panel_x + 15, y))

    def draw_graph(self):
        # Background
        panel_surf = pygame.Surface((self.graph_w, self.graph_h), pygame.SRCALPHA)
        pygame.draw.rect(panel_surf, (30, 35, 50, 220), (0, 0, self.graph_w, self.graph_h), border_radius=10)
        self.screen.blit(panel_surf, (self.graph_x, self.graph_y))

        # Title
        title = self.small_font.render("ERROR OVER TIME", True, TEXT_COLOR)
        self.screen.blit(title, (self.graph_x + 15, self.graph_y + 10))

        # Zero line
        zero_y = self.graph_y + 30 + self.graph_h - 30
        pygame.draw.line(self.screen, (80, 80, 100),
                         (self.graph_x + 10, zero_y),
                         (self.graph_x + self.graph_w - 10, zero_y), 1)

        if len(self.error_history) < 2:
            return

        max_error = max(self.error_history) if self.error_history else 1
        if max_error < 1:
            max_error = 1

        # Draw error line
        points = []
        for i, val in enumerate(self.error_history):
            x = self.graph_x + 10 + (i / self.max_history) * (self.graph_w - 20)
            y = zero_y - (val / max_error) * (self.graph_h - 60)
            points.append((int(x), int(y)))

        if len(points) > 1:
            pygame.draw.lines(self.screen, ERROR_COLOR, False, points, 2)

        # Axis labels
        max_label = self.small_font.render(f"max: {max_error:.0f}", True, (150, 150, 170))
        self.screen.blit(max_label, (self.graph_x + 15, self.graph_y + 15))

    def draw_instructions(self):
        panel_surf = pygame.Surface((self.instruction_panel_w, self.instruction_panel_h), pygame.SRCALPHA)
        pygame.draw.rect(panel_surf, (25, 30, 45, 200), (0, 0, self.instruction_panel_w, self.instruction_panel_h), border_radius=10)
        self.screen.blit(panel_surf, (self.instruction_panel_x, self.instruction_panel_y))

        lines = [
            "🎯  HOW TO PLAY",
            "",
            "  • TARGET (red) follows your mouse by default",
            "  • Press TAB to detach it — move it freely to tune sliders",
            "  • Press TAB again to re-dock it to the cursor",
            "  • Watch the CHASER (blue) pursue using PID control",
            "  • Drag the slider knobs to adjust parameters:",
            "      Kp (Proportional) — reacts to current error",
            "      Ki (Integral)      — accumulates past error (remembers)",
            "      Kd (Derivative)    — predicts future error (dampens)",
            "",
            "  💡 Try: high Kp = oscillation  |  add Ki = eliminate gap  |  add Kd = smooth it out",
        ]

        y = self.instruction_panel_y + 10
        for line in lines:
            if "💡" in line or "🎯" in line:
                color = ACCENT_COLOR
            else:
                color = TEXT_COLOR
            txt = self.small_font.render(line, True, color)
            self.screen.blit(txt, (self.instruction_panel_x + 15, y))
            y += 22

    def draw_capture_indicator(self):
        if self.captured:
            elapsed = (pygame.time.get_ticks() - self.capture_timer) / 1000.0
            if elapsed < 2.0:
                # Flashing capture text
                alpha = int(128 + 127 * math.sin(elapsed * 6))
                text = self.big_font.render("✦ CAPTURED! ✦", True, (255, 255, 100))
                rect = text.get_rect(center=(SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2 - 50))
                self.screen.blit(text, rect)


if __name__ == "__main__":
    game = Game()
    game.run()
