import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
import matplotlib.animation as animation

# Simulation parameters
time_steps = 10000
drone_speeds = np.random.randint(1, 6, time_steps)

# PID parameters
Kp_init = 0.5
Ki_init = 0.1
Kd_init = 0.05

# Initial conditions
drone_position = 0
camera_position = 0
integral = 0
previous_error = 0

# Recording positions for plotting
drone_positions = []
camera_positions = []
errors = []

# Function to simulate PID controller for one step
def simulate_pid_step(Kp, Ki, Kd, step):
    global drone_position, camera_position, integral, previous_error
    drone_position += drone_speeds[step]
    drone_positions.append(drone_position)
    
    error = drone_position - camera_position
    integral += error
    derivative = error - previous_error
    
    control_signal = Kp * error + Ki * integral + Kd * derivative
    
    camera_speed = control_signal
    camera_position += camera_speed
    camera_positions.append(camera_position)
    
    previous_error = error
    errors.append(error)
    
    return drone_position, camera_position, error

# Function to reset the simulation
def reset_simulation():
    global drone_position, camera_position, integral, previous_error, drone_positions, camera_positions, errors, current_step
    drone_position = 0
    camera_position = 0
    integral = 0
    previous_error = 0
    drone_positions.clear()
    camera_positions.clear()
    errors.clear()
    current_step = 0

# Initial simulation for the first time step
simulate_pid_step(Kp_init, Ki_init, Kd_init, 0)

# Plotting setup
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 12))
plt.subplots_adjust(left=0.1, bottom=0.35)
drone_line, = ax1.plot(drone_positions, label='Drone Position')
camera_line, = ax1.plot(camera_positions, label='Camera Position', linestyle='--')
ax1.set_xlabel('Time Steps')
ax1.set_ylabel('Position')
ax1.set_title('PID Controller for Camera Following Drone')
ax1.legend()
ax1.grid(True)
ax1.set_xlim(0, 100)
ax1.set_ylim(-10, 10)

error_line, = ax2.plot(errors, label='Error', color='r')
ax2.axhline(0, color='black', linewidth=0.8)
ax2.set_xlabel('Time Steps')
ax2.set_ylabel('Error')
ax2.set_title('Error Over Time')
ax2.legend()
ax2.grid(True)
ax2.set_xlim(0, 100)
ax2.set_ylim(-10, 10)

# Error display
error_display = ax1.text(0.02, 0.95, '', transform=ax1.transAxes)

# PID parameter sliders
axcolor = 'lightgoldenrodyellow'
ax_Kp = plt.axes([0.1, 0.25, 0.65, 0.03], facecolor=axcolor)
ax_Ki = plt.axes([0.1, 0.2, 0.65, 0.03], facecolor=axcolor)
ax_Kd = plt.axes([0.1, 0.15, 0.65, 0.03], facecolor=axcolor)

s_Kp = Slider(ax_Kp, 'Kp', 0.0, 2.0, valinit=Kp_init)
s_Ki = Slider(ax_Ki, 'Ki', 0.0, 1.0, valinit=Ki_init)
s_Kd = Slider(ax_Kd, 'Kd', 0.0, 1.0, valinit=Kd_init)

# Update function for sliders
def update(val):
    pass

s_Kp.on_changed(update)
s_Ki.on_changed(update)
s_Kd.on_changed(update)

# Global variable to keep track of current step
current_step = 0

# Reset button
resetax = plt.axes([0.8, 0.025, 0.1, 0.04])
button = Button(resetax, 'Restart', color=axcolor, hovercolor='0.975')

def reset(event):
    reset_simulation()
    ani.event_source.stop()
    ani.event_source.start()

button.on_clicked(reset)

# Animation function
def animate(i):
    global current_step
    if current_step < time_steps:
        Kp = s_Kp.val
        Ki = s_Ki.val
        Kd = s_Kd.val
        drone_position, camera_position, error = simulate_pid_step(Kp, Ki, Kd, current_step)
        current_step += 1
        
        # Update lines
        drone_line.set_data(range(len(drone_positions)), drone_positions)
        camera_line.set_data(range(len(camera_positions)), camera_positions)
        error_line.set_data(range(len(errors)), errors)
        
        # Update error display
        error_display.set_text(f'Error: {error:.2f}')
        
        ax1.set_ylim(min(drone_positions + camera_positions) - 10, max(drone_positions + camera_positions) + 10)
        ax2.set_ylim(min(errors) - 10, max(errors) + 10)
        
        fig.canvas.draw_idle()

ani = animation.FuncAnimation(fig, animate, frames=time_steps, repeat=False)
plt.show()
