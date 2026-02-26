# Proj2_ESE4460_Team3
pure awesomeness


<img width="1429" height="888" alt="image" src="https://github.com/user-attachments/assets/1f085272-7cfb-411e-8277-b0dd32c0299a" />

<img width="1336" height="567" alt="image" src="https://github.com/user-attachments/assets/40ff8310-0ec2-45a8-8f2b-ab42ed7214fb" />

<img width="1263" height="883" alt="image" src="https://github.com/user-attachments/assets/d287218c-bf5c-44a4-a1d5-f90063e376b9" />

<img width="1219" height="604" alt="image" src="https://github.com/user-attachments/assets/e88c49fc-3d7a-45d8-bc6b-668fb6836c14" />

# Rotation about the x-axis by angle θ
Rx = [
   [1, 0, 0, 0],
   [0, cos(θ), -sin(θ), 0],
   [0, sin(θ), cos(θ), 0],
   [0, 0, 0, 1]
]
# Rotation about the y-axis by angle θ
Ry = [
   [cos(θ), 0, sin(θ), 0],
   [0, 1, 0, 0],
   [-sin(θ), 0, cos(θ), 0],
   [0, 0, 0, 1]
]
# Rotation about the z-axis by angle θ
Rz = [
   [cos(θ), -sin(θ), 0, 0],
   [sin(θ), cos(θ), 0, 0],
   [0, 0, 1, 0],
   [0, 0, 0, 1]
]
R = Rz(γ) @ Ry(β) @ Rx(α)



