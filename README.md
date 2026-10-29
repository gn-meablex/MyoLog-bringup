# Motor bring-up: SimpleFOC 

Standalone sketch for testing the hand's winch motor on its own, controlled
over serial.

Hall sensors handle commutation through SimpleFOC. The MT6701 is read
separately to track spool position, and moves stop on reaching the target
or on a stall (under 0.5° of movement in 100 ms).

## Pins

- Hall: GPIO 3, 12, 46
- PWM: GPIO 21, 18, 14
- MT6701 I2C: SDA 10, SCL 9

## Usage

Open serial at 115200. Jog the hand fully open with `F` / `B`(which moves 30 deg at a time), then send
`H` to set home. Home isn't saved, so redo this after every reset.

| Cmd | Action |
|---|---|
| `H` | Set home (fully open) |
| `C` | Close |
| `O` | Open |
| `R` | Return home |
| `F` / `B` | Jog forward / reverse |
| `S` | Stop |
