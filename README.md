# PID Chase Game 🎪

A hands-on educational game for learning **PID control** — Proportional, Integral, and Derivative controllers.

## What is this?

A blue **Chaser** pursues a red **Target** using a PID controller. You control the target and adjust the PID parameters in real-time to see how each one affects the chaser's behavior.

### Educational Goals

| Parameter | What it does | What happens when you crank it up |
|-----------|-------------|-----------------------------------|
| **Kp** (Proportional) | Reacts to *current* error | Higher = faster response, but too much causes oscillation |
| **Ki** (Integral) | Accumulates *past* error | Eliminates steady-state gap, but can cause overshoot |
| **Kd** (Derivative) | Predicts *future* error | Dampens oscillation, prevents overshoot |

## Quick Start

### macOS / Linux / Windows

```bash
uv run game.py
```

That's it. `uv` installs `pygame` automatically from `pyproject.toml`.

## Controls

| Action | Effect |
|--------|--------|
| **Move mouse** | Move the Target (red circle follows cursor) |
| **Drag slider knobs** | Adjust Kp (red), Ki (blue), Kd (green) |
| **T** | Toggle auto-moving target |
| **R** | Reset chaser to center |
| **Space** | Pause / Resume |
| **Q / Esc** | Quit |

## PID Tuning Challenges

Try these exercises:

1. **Kp only** — Set Ki=0, Kd=0. Increase Kp slowly. At what point does it start oscillating?
2. **Add Kd** — With a high Kp that oscillates, add Kd to dampen it. How much Kd does it take?
3. **Add Ki** — Set Kp high, Kd=0, Ki=0. Watch the chaser fall behind. Slowly add Ki to eliminate the gap.
4. **Full tuning** — Can you get the chaser to track the target smoothly without overshoot?
5. **Auto mode** — Press T to let the target move in a figure-8 pattern. Can your PID keep up?

## How PID Works

```
Error = Target Position − Chaser Position

P term = Kp × Error                    ← "Right now, how far off am I?"
I term = Ki × Σ(Error × dt)           ← "On average, how far off have I been?"
D term = Kd × d(Error)/dt             ← "Which way is the error changing?"

Control Output = P + I + D
```

## Project Structure

```
control-theory-dot-game/
├── game.py          # Main game (pygame)
├── pyproject.toml   # uv dependencies
├── README.md        # This file
└── LICENSE          # MIT
```

## License

MIT
