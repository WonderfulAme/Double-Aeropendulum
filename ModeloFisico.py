import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.integrate import odeint

# Physical parameters
m = 0.5        # Total mass (kg)
l1 = 0.3       # Distance from pivot to rotor 1 (m)
l2 = 0.3       # Distance from pivot to rotor 2 (m)
d = 0.1     # Distance from pivot to center of gravity (m)
g = 9.81       # Gravity (m/s^2)
c = 0.1        # Damping coefficient (N·m·s/rad)
J = m * (l1**2 + l2**2) / 3  # Moment of inertia (simplified)

# Control inputs (thrust forces in Newtons)
def thrust_control(t, theta, theta_dot):
    """
    Define thrust control strategy
    You can modify this function to test different control schemes
    """
    # Example: Constant thrust
    T1 = 0.5
    T2 = 0
    
    # Example: Oscillating thrust
    # T1 = 2.0 + 0.5 * np.sin(2 * t)
    # T2 = 1.5 + 0.5 * np.cos(2 * t)
    
    # # Example: Feedback control to stabilize at horizontal
    # Kp, Kd = 5.0, 2.0
    # T1 = 2.0 - Kp * theta - Kd * theta_dot
    # T2 = 1.5 + Kp * theta + Kd * theta_dot
    
    return T1, T2

# Differential equation
def aeropendulum_dynamics(state, t):
    """
    state = [theta, theta_dot]
    Returns [theta_dot, theta_ddot]
    """
    theta, theta_dot = state
    
    # Get current thrust values
    T1, T2 = thrust_control(t, theta, theta_dot)
    
    # Calculate angular acceleration from the dynamics equation
    # J θ'' = T₁l₁ - T₂l₂ - mgd sin(θ) - c θ'
    theta_ddot = (T1 * l1 - T2 * l2 - m * g * d * np.cos(theta) - c * theta_dot) / J
    
    return [theta_dot, theta_ddot]

# Initial conditions
theta0 = 90 * np.pi / 180      # Initial angle in radians
theta_dot0 = 0.0        # Initial angular velocity

# Time parameters
t_span = np.linspace(0, 10, 500)  # 10 seconds simulation

# Solve ODE
initial_state = [theta0, theta_dot0]
solution = odeint(aeropendulum_dynamics, initial_state, t_span)
theta_history = solution[:, 0] % (2 * np.pi)  # Wrap angle between -π and π
theta_dot_history = solution[:, 1] 

# Animation setup
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

# Left plot: Physical animation
ax1.set_xlim(-0.6, 0.6)
ax1.set_ylim(-0.6, 0.6)
ax1.set_aspect('equal')
ax1.grid(True, alpha=0.3)
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')
ax1.set_title('Double Rotor Aeropendulum')

# Plot elements
pivot_point, = ax1.plot(0, 0, 'ko', markersize=10, label='Pivot')
bar_line, = ax1.plot([], [], 'b-', linewidth=3, label='Bar')
rotor1_point, = ax1.plot([], [], 'ro', markersize=12, label='Rotor 1')
rotor2_point, = ax1.plot([], [], 'go', markersize=12, label='Rotor 2')
cg_point, = ax1.plot([], [], 'mx', markersize=10, label='Center of Gravity')
# Store thrust arrows as list to update them
thrust_arrows = []
ax1.legend(loc='upper right')

# Right plot: Angle vs time
ax2.set_xlim(0, 10)
ax2.set_ylim(0, 2 * np.pi)
ax2.grid(True, alpha=0.3)
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Angle θ (rad)')
ax2.set_title('Angle Evolution')
angle_line, = ax2.plot([], [], 'b-', linewidth=2)
current_point, = ax2.plot([], [], 'ro', markersize=8)

# Add reference lines
ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3, label='Horizontal')
ax2.legend()

def init():
    bar_line.set_data([], [])
    rotor1_point.set_data([], [])
    rotor2_point.set_data([], [])
    cg_point.set_data([], [])
    angle_line.set_data([], [])
    current_point.set_data([], [])
    return bar_line, rotor1_point, rotor2_point, cg_point, angle_line, current_point

def animate(frame):
    t = t_span[frame]
    theta = theta_history[frame]
    theta_dot = theta_dot_history[frame]
    
    # Calculate positions
    # Rotor 1 position (on the positive side)
    x1 = l1 * np.cos(theta)
    y1 = l1 * np.sin(theta)
    
    # Rotor 2 position (on the negative side)
    x2 = -l2 * np.cos(theta)
    y2 = -l2 * np.sin(theta)
    
    # Center of gravity position
    x_cg = d * np.cos(theta)
    y_cg = d * np.sin(theta)
    
    # Update bar line
    bar_line.set_data([x2, x1], [y2, y1])
    
    # Update rotor positions
    rotor1_point.set_data([x1], [y1])
    rotor2_point.set_data([x2], [y2])
    cg_point.set_data([x_cg], [y_cg])
    
    # Remove old thrust arrows
    global thrust_arrows
    for arrow in thrust_arrows:
        arrow.remove()
    thrust_arrows = []
    
    # Update thrust arrows
    T1, T2 = thrust_control(t, theta, theta_dot)
    thrust_scale = 0.05
    
    # Thrust arrow perpendicular to bar at rotor positions
    perp_angle = theta + np.pi/2
    arrow1 = ax1.arrow(x1, y1, thrust_scale * T1 * np.cos(perp_angle), 
              thrust_scale * T1 * np.sin(perp_angle),
              head_width=0.03, head_length=0.02, fc='red', ec='red', alpha=0.7)
    arrow2 = ax1.arrow(x2, y2, thrust_scale * T2 * np.cos(perp_angle), 
              thrust_scale * T2 * np.sin(perp_angle),
              head_width=0.03, head_length=0.02, fc='green', ec='green', alpha=0.7)
    thrust_arrows = [arrow1, arrow2]
    
    # Update angle plot
    angle_line.set_data(t_span[:frame+1], theta_history[:frame+1])
    current_point.set_data([t], [theta])
    
    return bar_line, rotor1_point, rotor2_point, cg_point, angle_line, current_point

# Print simulation info
print(f"Starting simulation...")
print(f"Initial angle: {np.degrees(theta0):.1f} degrees")
print(f"Final angle: {np.degrees(theta_history[-1]):.1f} degrees")
print(f"Max angle: {np.degrees(np.max(theta_history)):.1f} degrees")
print(f"Min angle: {np.degrees(np.min(theta_history)):.1f} degrees")

# Create animation
anim = FuncAnimation(fig, animate, init_func=init, frames=len(t_span), 
                     interval=20, blit=True, repeat=True)

plt.tight_layout()
plt.show(block=True)  # Force the window to stay open

print("Simulation window closed.")