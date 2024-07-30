import numpy as np

def angleOfAttack_and_slipRate(u, v, w, u_w, v_w, w_w):
    """
    Calculate angle of attack (alpha) and sideslip angle (beta).

    Parameters:
    u (float): Forward velocity component.
    v (float): Lateral velocity component.
    w (float): Vertical velocity component.
    u_w (float): Forward velocity component of wind.
    v_w (float): Lateral velocity component of wind.
    w_w (float): Vertical velocity component of wind.

    Returns:
    tuple: (alpha, beta) where alpha is the angle of attack and beta is the sideslip angle.
    """
    # Relative velocities
    u_r = u - u_w
    v_r = v - v_w
    w_r = w - w_w
    
    # Calculate angle of attack (alpha)
    alpha = np.arctan2(w_r, u_r)
    
    # Calculate sideslip angle (beta)
    u_r_squared = u_r**2
    v_r_squared = v_r**2
    w_r_squared = w_r**2
    denominator = np.sqrt(u_r_squared + v_r_squared + w_r_squared)
    
    # Using arctan for beta to avoid potential issues with arcsin range
    beta = np.arctan2(v_r, np.sqrt(u_r_squared + w_r_squared))
    
    return alpha, beta

# Example usage
u = 10.0
v = 2.0
w = 1.0
u_w = 1.0
v_w = 0.5
w_w = 0.2

alpha, beta = angleOfAttack_and_slipRate(u, v, w, u_w, v_w, w_w)
print(f"Angle of Attack (alpha): {np.degrees(alpha)} degrees")
print(f"Sideslip Angle (beta): {np.degrees(beta)} degrees")
