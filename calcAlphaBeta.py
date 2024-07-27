import math

def calculate_alpha_beta(Va, theta, psi):
    """
    Calculate angle of attack (alpha) and sideslip angle (beta).

    Parameters:
    - Va: Airspeed (m/s)
    - phi: Roll angle (radians) -- not needed
    - theta: Pitch angle (radians)
    - psi: Yaw angle (radians)
    - p: Roll rate (rad/s)      -- not needed
    - q: Pitch rate (rad/s)     -- not needed
    - r: Yaw rate (rad/s)       -- not needed

    Returns:
    - alpha: Angle of attack (radians)
    - beta: Sideslip angle (radians)
    """
    # Calculate the body-relative airspeed components
    u = Va * math.cos(theta) * math.cos(psi)
    v = Va * math.cos(theta) * math.sin(psi)
    w = Va * math.sin(theta)

    # Calculate the angle of attack (alpha)
    alpha = math.atan2(w, u)

    # Calculate the sideslip angle (beta)
    beta = math.atan2(v, u) #TODO: correct to reflect page 21 of the uav book https://www.dropbox.com/scl/fi/uwa7xwpb9imxziutfem6z/uavbook.pdf?rlkey=efpq4vy3ynizf427gexu7rs6e&e=1&dl=0

    return alpha, beta

# Example usage
Va = 50.0  # Airspeed in m/s
phi = math.radians(5)  # Roll angle in degrees converted to radians
theta = math.radians(2)  # Pitch angle in degrees converted to radians
psi = math.radians(10)  # Yaw angle in degrees converted to radians
p = 0.1  # Roll rate in rad/s
q = 0.05  # Pitch rate in rad/s
r = 0.2  # Yaw rate in rad/s

alpha, beta = calculate_alpha_beta(Va, theta, psi)

print(f"Angle of Attack (α): {math.degrees(alpha):.2f} degrees")
print(f"Sideslip Angle (β): {math.degrees(beta):.2f} degrees")
