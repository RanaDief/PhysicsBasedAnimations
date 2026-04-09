cat > README.md <<'EOF'
# Power Core Delivery

A 2D physics-based puzzle game built with **Python** and **Pygame**, designed to demonstrate a **modular physics engine** for gaming.

---

## Project Description

This project is a small interactive game built on top of a custom physics engine.  
The player controls an energy orb called the **Power Core** and must deliver it safely to the reactor at the end of each level.

The main purpose of the project is not only to make a playable game, but also to clearly demonstrate the core components of a physics engine used in computer graphics and game development.

The game is intentionally designed as a **simple 2D physics puzzle/platformer** so that the physics systems are easy to implement, debug, and explain during the final presentation.

---

## Game Idea

The game takes place inside a futuristic lab or energy facility.

The player moves an energy orb through a sequence of levels.  
To reach the final target, the orb must interact with objects driven by different physics systems.

Examples of gameplay:
- pushing rigid crates to activate switches
- crossing a soft bridge that deforms under weight
- using a rope or chain to swing over gaps
- avoiding hazards and visual effects made with particles
- activating a robotic arm to open a gate or move an obstacle

This lets the project show multiple physics modules in a single playable environment.

---

## Project Goals

The main goals of this project are:

1. Build a modular and reusable physics engine
2. Implement multiple physics simulation systems
3. Integrate the physics engine into a small game
4. Keep the implementation simple, stable, and easy to explain
5. Demonstrate the engine in a clear visual way during gameplay

---

## Physics Features to Implement

### 1. Core Engine Architecture

The physics engine is organized around the following concepts:

- object/state representation
- simulation update loop
- force accumulation
- numerical integration
- collision handling
- constraint resolution

These systems form the foundation of the engine and are reused across all physics modules.

---

### 2. Particle System

The particle system is used to simulate small visual effects made from many lightweight particles.

#### Planned features
- particle emitter
- particle lifetime
- spawn rate
- velocity and acceleration
- gravity and wind effects
- particle removal after death
- simple rendering with circles or sprites

#### Example uses in the game
- sparks when the player activates a switch
- smoke near the reactor
- dust when landing on the ground
- win effect when completing a level

---

### 3. Mass-Spring System

The mass-spring system is used for deformable objects made from particles connected by springs.

#### Planned features
- particles with mass, position, and velocity
- springs with rest length and stiffness
- damping to reduce oscillation
- external forces such as gravity

#### Example uses in the game
- soft bridge
- trampoline
- jelly barrier
- hanging cable

---

### 4. Position-Based Dynamics (PBD)

PBD is used to enforce constraints directly on particle positions, making the simulation more stable in real time.

#### Planned features
- distance constraints
- pin constraints
- iterative constraint solving
- rope/chain stabilization
- optional soft-body constraints

#### Example uses in the game
- stable hanging rope
- rope bridge
- pendulum
- constrained soft object

---

### 5. Rigid Body Dynamics

Rigid body dynamics is used for non-deformable objects that move and collide.

#### Planned features
- position, velocity, mass
- gravity
- force accumulation
- restitution / bounce
- ground collision
- box-object collision
- simple impulse or velocity correction response

#### Example uses in the game
- crates
- falling blocks
- bouncing balls
- movable obstacles

---

### 6. Kinematics

Kinematics is used for articulated motion, especially for a robotic arm or mechanism in the level.

#### Planned features
- forward kinematics for hierarchical joints
- inverse kinematics for reaching a target
- 2-link or 3-link arm

#### Example uses in the game
- robotic arm presses a button
- mechanical arm opens a door
- animated gate system

---

## Proposed Gameplay Structure

To keep the project manageable, the game can be split into **three small levels**.

### Level 1 - Basic Physics Room
Purpose:
- introduce player movement
- demonstrate rigid body dynamics
- show a simple particle effect

Gameplay:
- move the orb
- push a crate onto a floor switch
- switch opens the gate
- sparks appear when activated

Physics shown:
- rigid body
- collision handling
- particle system

---

### Level 2 - Soft Bridge Room
Purpose:
- demonstrate deformable simulation

Gameplay:
- cross a hanging soft bridge
- bridge bends and reacts to weight
- optional wind affects the bridge

Physics shown:
- mass-spring system
- damping
- optional particles or rope constraints

---

### Level 3 - Reactor Access Room
Purpose:
- combine multiple modules in one final puzzle

Gameplay:
- use a rope or constrained object to cross a gap
- activate a robotic arm
- arm opens the final reactor gate
- deliver the orb to the goal

Physics shown:
- PBD
- kinematics
- rigid body
- particle effects

---

## Controls

Planned controls:

- **Left Arrow / A** → move left
- **Right Arrow / D** → move right
- **Space** → jump
- **R** → restart current level
- **ESC** → quit the game

If the player is implemented as a draggable orb instead of a platform character, controls can be changed later.

---

## Technical Design

The project is divided into two main layers:

### Engine Layer
This contains the reusable physics code:
- vector math
- bodies
- forces
- springs
- particles
- constraints
- collisions
- numerical integration
- kinematics

### Game Layer
This contains game-specific systems:
- player logic
- levels
- UI
- visual effects
- scenes
- game state management

This separation makes the code cleaner and easier to present.

---

## Project Folder Structure

```text
physics_engine_game/
│
├── main.py
├── config.py
├── README.md
├── requirements.txt
│
├── assets/
│   ├── images/
│   ├── sounds/
│   └── fonts/
│
├── engine/
│   ├── __init__.py
│   ├── vector.py
│   ├── body.py
│   ├── particle.py
│   ├── spring.py
│   ├── constraint.py
│   ├── collision.py
│   ├── integrator.py
│   ├── forces.py
│   ├── world.py
│   └── kinematics.py
│
├── game/
│   ├── __init__.py
│   ├── player.py
│   ├── level.py
│   ├── level_loader.py
│   ├── camera.py
│   ├── ui.py
│   ├── effects.py
│   └── game_manager.py
│
├── levels/
│   ├── level1.json
│   ├── level2.json
│   └── level3.json
│
├── scenes/
│   ├── __init__.py
│   ├── menu_scene.py
│   ├── play_scene.py
│   └── win_scene.py
│
└── docs/
    ├── architecture_notes.md
    ├── formulas.md
    └── screenshots/