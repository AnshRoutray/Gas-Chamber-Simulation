import tkinter as tk
import time
import math
import random
import numpy as np
import matplotlib.pyplot as plt

def v_after_collision(mA, mB, vA, vB, posA, posB):
    normal = (posB[0] - posA[0], posB[1] - posA[1])
    mag_normal = math.sqrt(normal[0] ** 2 + normal[1] ** 2)
    if mag_normal == 0:
        raise ValueError("Positions must not be identical.")
    normal = (normal[0] / mag_normal, normal[1] / mag_normal)
    tangent = (-normal[1], normal[0])
    vA_normal = vA[0] * normal[0] + vA[1] * normal[1]
    vB_normal = vB[0] * normal[0] + vB[1] * normal[1]
    vA_tangent = vA[0] * tangent[0] + vA[1] * tangent[1]
    vB_tangent = vB[0] * tangent[0] + vB[1] * tangent[1]
    vA_normal_final = ((mA - mB) * vA_normal + 2 * mB * vB_normal) / (mA + mB)
    vB_normal_final = ((mB - mA) * vB_normal + 2 * mA * vA_normal) / (mA + mB)
    vA_tangent_final = vA_tangent
    vB_tangent_final = vB_tangent
    vA_final = (
        vA_normal_final * normal[0] + vA_tangent_final * tangent[0],
        vA_normal_final * normal[1] + vA_tangent_final * tangent[1]
    )
    vB_final = (
        vB_normal_final * normal[0] + vB_tangent_final * tangent[0],
        vB_normal_final * normal[1] + vB_tangent_final * tangent[1]
    )
    return [vA_final, vB_final]

def correct_errors(x1, y1, vx1, vy1, x2, y2, vx2, vy2, radius1, radius2):
    dist =  math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    if dist <= (radius1 + radius2):
        if dist < (radius1 + radius2):

            # Error Correction for overlap
            p = x1 - x2 
            q = vx2 - vx1
            r = y1 - y2 
            s = vy2 - vy1
            alpha = q ** 2 + s ** 2
            beta = 2 * (p * q + r * s)
            gamma = p ** 2 + r ** 2 - (radius1 + radius2) ** 2
            t1 = (-beta + math.sqrt(beta ** 2 - 4 * alpha * gamma)) / (2 * alpha) 
            t2 = (-beta - math.sqrt(beta ** 2 - 4 * alpha * gamma)) / (2 * alpha)
            t = t1 if t1 > 0 else t2 
            x1 = x1 - vx1 * t
            y1 = y1 - vy1 * t
            x2 = x2 - vx2 * t
            y2 = y2 - vy2 * t
            return ((x1, y1), (x2, y2))

def checkCollisions(masses):
    objectsCalculated = []
    for index in range(0, len(masses)):
        x1, y1 = masses[index][0]
        vx1, vy1 = masses[index][1]
        radius1 = masses[index][3]
        for jndex in range(0, len(masses)):
            if not (((index, jndex) in objectsCalculated or (jndex, index) in objectsCalculated) or index == jndex):
                x2, y2 = masses[jndex][0]
                vx2, vy2 = masses[jndex][1]
                radius2 = masses[jndex][3]
                dist =  math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                if dist <= (radius1 + radius2):
                    if dist < (radius1 + radius2):
                        results = correct_errors(x1, y1, vx1, vy1, x2, y2, vx2, vy2, radius1, radius2)
                        x1, y1 = results[0]
                        x2, y2 = results[1]

                    v_new = v_after_collision(
                        radius1, radius2,
                        [vx1, vy1], [vx2, vy2],
                        (x1, y1), (x2, y2)
                    )
                    vx1, vy1 = v_new[0]
                    vx2, vy2 = v_new[1]
                    masses[index][0] = [x1, y1]
                    masses[index][1] = [vx1, vy1]
                    masses[jndex][0] = [x2, y2]
                    masses[jndex][1] = [vx2, vy2]
            objectsCalculated.append((index, jndex))
        if x1 - radius1 < 0:
            vx1 = -vx1
            x1 = radius1
        if x1 + radius1 > canvas_width / meterLength:
            vx1 = -vx1
            x1 = canvas_width / meterLength - radius1
        if y1 - radius1 < 0:
            vy1 = -vy1
            y1 = radius1
        if y1 + radius1 > canvas_height / meterLength:
            vy1 = -vy1
            y1 = canvas_height / meterLength - radius1
        masses[index][0] = [x1, y1]
        masses[index][1] = [vx1, vy1]

def generate_masses(count, width, height, min_mass, max_mass):
    masses = []
    mass = random.randint(min_mass, max_mass)
    position = [0, 3]
    for i in range(0, count):
        mass = random.randint(min_mass, max_mass)
        x = position[0]
        y = position[1]
        x += mass + 2
        if x > width - 3:
            x = mass + 3
            y += mass + 8
        if y > height - 3:
            break
        position = [x, y]
        velocity = [10, 10]
        masses.append([position, velocity, [0, 0], mass])
    return masses
        
def update_values():
    global lastTime
    global masses
    global massDrawings
    currentTime = time.time()
    deltaTime = currentTime - lastTime
    lastTime = currentTime
    initialPositions = []
    for index in range(0, len(masses)):
        x0, y0 = masses[index][0]
        vx0, vy0 = masses[index][1]
        ax0, ay0 = masses[index][2]

        x_new = x0 + vx0 * deltaTime
        y_new = y0 + vy0 * deltaTime
        vx_new, vy_new = vx0 + ax0 * deltaTime, vy0 + ay0 * deltaTime
        initialPositions.append([x0, y0])
        masses[index][0] = [x_new, y_new]
        masses[index][1] = [vx_new, vy_new]
    checkCollisions(masses)
    for index in range(0, len(masses)):
        x_new, y_new = masses[index][0]
        x0, y0 = initialPositions[index]
        dx, dy = (x_new - x0) * meterLength, (y_new - y0) * meterLength
        canvas.move(massDrawings[index], dx, dy)
    update_histogram(masses)
    root.after(1, update_values)

def update_histogram(masses):
    global global_counts, bars

    # Extract velocities and masses
    velocities = np.array([particle[1] for particle in masses])  # [vx, vy]
    masses_values = np.array([particle[3] for particle in masses])  # mass

    # Calculate speeds and kinetic energies
    speeds = np.linalg.norm(velocities, axis=1)
    kinetic_energies = 0.5 * masses_values * np.square(speeds)

    # Compute the histogram for new data
    counts, _ = np.histogram(kinetic_energies, bins=bin_edges)

    # Update global counts
    global_counts = counts

    # Update the histogram bars
    if bars is None:
        bars = ax.bar(bin_edges[:-1], global_counts, width=np.diff(bin_edges), align='edge', color='blue', edgecolor='black')
    else:
        for bar, count in zip(bars, global_counts):
            bar.set_height(count)

    # Redraw the plot
    ax.set_ylim(0, global_counts.max() + 5)  # Adjust y-axis dynamically
    plt.draw()

# Initialize the histogram
plt.ion()
fig, ax = plt.subplots()
bars = None
bin_edges = np.linspace(0, 150, 20)  # Define fixed bins for histogram (adjust as needed)
ax.set_xlabel('Kinetic Energy')
ax.set_ylabel('Number of Particles')
ax.set_title('Maxwell Boltzmann Distribution')
plt.show(block=False)

# Global storage for particle counts in each bin
global_counts = np.zeros(len(bin_edges) - 1)

#Simulation 

root = tk.Tk()
root.title("Collision Simulator")

canvas_width, canvas_height = 500, 500
canvas = tk.Canvas(root, width=canvas_width, height=canvas_height, bg = "black")
canvas.pack()

meterLength = 10

masses = generate_masses(50, canvas_width / meterLength, canvas_height / meterLength, 1, 1)
massDrawings = []

for mass in masses:
    radius = mass[3]
    massDrawings.append(
        canvas.create_oval(
            (mass[0][0] - radius) * meterLength,
            (mass[0][1] - radius) * meterLength,
            (mass[0][0] + radius) * meterLength,
            (mass[0][1] + radius) * meterLength,
            fill="yellow"
        )
    )

lastTime = time.time()
update_values()
root.mainloop()
