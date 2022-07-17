import sympy

from IPython.display import display

sympy.init_printing(use_latex='mathjax')

vx, vy, vz, ax, ay, az, yaw, yaw_dot, dt = sympy.symbols('vx, vy, vz, ax, ay, az, yaw, yaw_dot, dt')
# vx*math.cos(yaw_dot) - vy*math.sin(yaw_dot)
# vy*math.cos(yaw_dot) - vx*math.sin(yaw_dot)
F = sympy.Matrix([[(vx+ax*dt)*sympy.cos(yaw_dot * dt)-(vy+ay*dt)*sympy.sin(yaw_dot * dt)],
                  [(vy+ay*dt)*sympy.cos(yaw_dot * dt)-(vx+ax*dt)*sympy.sin(yaw_dot * dt)],
                  [vz+az * dt],
                  [ax],
                  [ay],
                  [az],
                  [yaw+yaw_dot * dt],
                  [yaw_dot]])

state = sympy.Matrix([vx, vy, vz, ax, ay, az, yaw, yaw_dot])

JF = F.jacobian(state)

display(state)
display(JF)
