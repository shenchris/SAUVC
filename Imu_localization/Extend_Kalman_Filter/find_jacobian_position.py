import sympy

from IPython.display import display

sympy.init_printing(use_latex='mathjax')

px, py, pz, vx, vy, vz, yaw, yaw_dot, dt = sympy.symbols('px, py, pz, vx, vy, az, yaw, yaw_dot, dt')

F = sympy.Matrix([[px+vx*dt],
                  [py+vy*dt],
                  [pz+vz*dt],
                  [vx],
                  [vy],
                  [vz],
                  [yaw+yaw_dot*dt],
                  [yaw_dot]])

state = sympy.Matrix([px,py,pz,vx,vy,vz,yaw,yaw_dot])

JF = F.jacobian(state)

display(state)
display(JF)
