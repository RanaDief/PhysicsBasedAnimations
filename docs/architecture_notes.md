# Architecture Notes

## Project Overview

This project is a small 2D physics animation/game prototype built with Python and
Pygame. The codebase is organized around a reusable physics engine package and a
Pygame application entry point that visualizes the implemented systems.

At the current stage, `main.py` is the runnable demo. It displays multiple
physics modules in one window:

- forward kinematics for a two-link arm
- inverse kinematics using cyclic coordinate descent
- circular rigid bodies with gravity, restitution, friction, and collisions
- a pressure-based soft body built from particles and springs
- a visual particle emitter used as a rain-like effect

The `game/`, `scenes/`, and `levels/` folders exist as planned boundaries for a
larger game structure, but their files are currently placeholders.

## High-Level Structure

```text
PhysicsBasedAnimations/
├── main.py                 # Current runnable Pygame demo
├── engine/                 # Reusable physics engine code
├── game/                   # Planned game-specific systems
├── scenes/                 # Planned scene/state screens
├── levels/                 # Planned level data files
└── docs/                   # Design and formula documentation
```

The intended architecture separates reusable simulation code from game-specific
logic:

- `engine/` owns physics state, simulation updates, collision handling,
  kinematics, particles, and springs.
- `main.py` creates a Pygame window, constructs demo objects, steps each system,
  and draws the result.
- `game/`, `scenes/`, and `levels/` are intended to become the gameplay layer
  once the prototype grows from a physics showcase into a level-based game.

## Runtime Flow

The current runtime flow is controlled by `main.py`.

1. Pygame is initialized and an `800 x 600` window is created.
2. Demo objects are constructed:
   - a forward kinematics chain
   - an inverse kinematics chain and CCD solver
   - several circular rigid bodies
   - one soft body
   - one particle emitter
3. Each frame:
   - Pygame events are processed
   - keyboard and mouse input update the IK target
   - kinematics points are recalculated
   - particles, rigid bodies, and the soft body are advanced
   - the scene is redrawn
4. The frame is presented with `pygame.display.flip()`.

The frame loop uses `clock.tick(FPS) / 1000.0` for the real-time `dt` used by
particles and rigid bodies. The soft body currently uses a fixed demo timestep
constant, `SOFT_BODY_DT`, instead of the frame `dt`.

## Engine Package

The `engine` package exposes its public API through `engine/__init__.py`. This
lets the application import the implemented physics features from one place:

```python
from engine import Body, Bounds, ParticleEmitter, SoftBody, World
```

### World Module

File: `engine/world.py`

`World` owns the active simulation objects and advances them in a consistent
order. It currently manages:

- rigid circular bodies
- soft bodies
- visual particle emitters
- global gravity
- world bounds
- floor friction
- collision iteration count
- positional correction strength

The application layer registers objects with the world, then calls
`world.update(dt)` once per frame. Rendering remains outside the world so the
engine can focus on simulation state.

Soft bodies can override the world's gravity, bounds, and timestep. This keeps
the current demo behavior intact while still moving update orchestration out of
`main.py`.

### Integrator Module

File: `engine/integrator.py`

The integrator module centralizes motion update functions shared by rigid
bodies, physical particles, and visual particles.

The current integration method is semi-implicit Euler:

```text
velocity += acceleration * dt
position += velocity * dt
```

`integrate_forces()` converts accumulated force into acceleration using inverse
mass, adds optional object acceleration and gravity, then applies the shared
semi-implicit Euler step.

### Rigid Body Module

File: `engine/body.py`

`Body` represents a minimal circular rigid body. It stores:

- position
- velocity
- radius
- mass and inverse mass
- restitution
- friction
- acceleration
- accumulated force
- static/dynamic state

Dynamic bodies integrate motion through `engine/integrator.py`:

```text
velocity += acceleration * dt
position += velocity * dt
```

Forces are accumulated with `apply_force()` and cleared after each integration
step. Static bodies use infinite mass and an inverse mass of zero, so they are
ignored by force integration and treated as immovable during collisions.

### Collision Module

File: `engine/collision.py`

The collision system currently supports circular bodies and axis-aligned world
bounds.

Important types and functions:

- `Bounds`: rectangular simulation area.
- `CollisionManifold`: collision normal and penetration depth.
- `detect_circle_collision()`: detects circle-circle overlap.
- `resolve_circle_collision()`: separates overlapping circles and applies
  impulse response.
- `resolve_all_circle_collisions()`: checks every unique body pair.
- `resolve_bounds_collision()`: keeps a circle inside a rectangle.

Circle collision response has two stages:

1. Positional correction separates overlapping bodies using inverse mass.
2. Velocity correction applies a bounce impulse along the collision normal.

Optional friction is applied as a tangential impulse after the bounce impulse.
Bounds collision also applies floor friction when the body touches the bottom
edge.

### Particle System

File: `engine/particle.py`

There are two particle concepts in this file:

- `VisualParticle`: lightweight render particle owned by a `ParticleEmitter`.
- `Particle`: physical point mass used by springs and soft bodies.

`ParticleEmitter` is used for visual effects such as rain, sparks, smoke, or
dust. It owns a list of particles and respawns them when they die or leave the
configured bounds if looping is enabled.

Configurable emitter properties include:

- spawn rectangle
- particle count
- velocity range
- acceleration
- radius range
- lifetime range
- color or random color function
- optional bounds
- loop behavior
- random seed

Physical `Particle` instances support force accumulation, integration, collision
against bounds, and simple circle drawing.

### Spring and Soft Body Modules

Files: `engine/spring.py`, `engine/particle.py`

`Spring` connects two physical particles. It calculates:

- current length
- direction between particles
- perpendicular normal
- spring force based on rest length and stiffness
- damping force based on relative velocity

`SoftBody` builds a closed loop of particles connected by springs. The particles
are placed around a circle, and neighboring particles are connected by springs.
The object also stores its initial polygon area.

During `SoftBody.update()`:

1. The current polygon area is measured.
2. Pressure is scaled by `initial_area / current_area`.
3. Each spring contributes spring force and pressure force to its particles.
4. Each particle integrates its motion and resolves optional bounds collision.

This creates a simple pressure-filled body that can deform and bounce inside the
window.

### Kinematics Module

File: `engine/kinematics.py`

The kinematics system uses 2D homogeneous transformation matrices.

`model_matrix(theta, tx, ty)` creates a 3 x 3 transform containing rotation and
translation. `ForwardKinematicsChain` multiplies transforms together to compute
joint positions for a chain of links.

`CCDInverseKinematicsSolver` implements cyclic coordinate descent. It repeatedly
walks backward through the chain and rotates each joint so the end effector moves
toward the target.

The current demo uses:

- a forward kinematics arm animated by time-based angles
- an inverse kinematics arm controlled by arrow keys or the mouse

## Application Demo Layout

The current window is split into panels:

- top-left: forward kinematics arm
- top-right: inverse kinematics arm
- bottom-left: rigid body collision demo
- center/bottom area: soft body demo
- full window: particle emitter overlay

`main.py` contains rendering helper functions for each visual system:

- `_draw_forward_kinematics()`
- `_draw_inverse_kinematics()`
- `_draw_rigid_bodies()`

Rigid body simulation is stepped through `_update_rigid_bodies()`, which:

1. integrates dynamic bodies under gravity
2. resolves bounds collision
3. runs multiple passes of circle collision resolution
4. resolves bounds collision again after pair collision correction

## Planned Game Layer

The repository includes placeholder folders for a larger game architecture:

- `game/player.py`
- `game/level.py`
- `game/level_loader.py`
- `game/game_manager.py`
- `game/camera.py`
- `game/ui.py`
- `game/effects.py`
- `scenes/menu_scene.py`
- `scenes/play_scene.py`
- `scenes/win_scene.py`
- `levels/level1.json`
- `levels/level2.json`
- `levels/level3.json`

These files are currently empty, but they suggest the intended direction:

- `game_manager` can own scene switching and global state.
- `level_loader` can parse JSON level files.
- `level` can create physics objects from level data.
- `player` can wrap a controllable `Body`.
- `effects` can create configured `ParticleEmitter` instances.
- `ui` can draw score, instructions, timers, or level labels.
- `scenes` can separate menu, gameplay, and win screens.

When implemented, this layer should depend on `engine`, while `engine` should
stay independent of game rules.

## Data and Dependency Direction

The clean dependency direction is:

```text
main.py / scenes / game
        ↓
      engine
        ↓
 pygame.math.Vector2 / numpy
```

The engine should not import from `game` or `scenes`. This keeps the physics
code reusable and easier to test.

The current engine uses:

- `pygame.math.Vector2` for vector math and drawing compatibility
- `pygame.draw` for simple debug rendering methods
- `numpy` for kinematics matrix math

## Extension Points

The following engine files exist but are currently empty:

- `engine/forces.py`
- `engine/constraint.py`
- `engine/vector.py`

Possible responsibilities:

- `forces.py`: define reusable gravity, drag, wind, and spring force helpers.
- `constraint.py`: implement distance constraints, pin constraints, and PBD
  solvers.
- `vector.py`: provide project-specific vector helpers if Pygame vectors become
  insufficient.

These modules should only be filled when repeated logic appears in the current
systems. Until then, keeping the implementation direct is simpler.

## Current Limitations

- Collision shapes are circles only.
- Collision detection is brute force over all body pairs.
- Soft body simulation uses a fixed timestep constant in `main.py`.
- Some modules listed in the project structure are placeholders.
- Level JSON files are empty.
- There is no scene manager or game state manager yet.
- There is no automated test suite in the repository.

## Recommended Next Architecture Step

The most useful next step is to introduce a fixed physics timestep. The current
demo passes frame `dt` into the world, while the soft body still uses its own
fixed demo timestep. A shared fixed-step update would make rigid body collision,
particles, and soft body behavior more stable across machines.

Until gameplay systems are implemented, `main.py` is a reasonable place to keep
input and rendering because it makes the physics modules easy to inspect and
present. Simulation orchestration now belongs to `World`.
