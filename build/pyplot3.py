import pandas as pd
import matplotlib.pyplot as plt 
import numpy as np 

df = pd.read_csv("log.csv")

Time = df["Sim Time"]

X = df["X Pos"]
Y = df["Y Pos"]
Z = df["Z Pos"]

phi = df["Phi"]
theta = df["Theta"]
psi = df ["Psi"]


Z_Setpoint = df["Height Setpoint"]
Theta_Setpoint = df["Theta Setpoint"]
Heading_Setpoint = df["Heading Setpoint"]

Time = Time.to_numpy()

X = X.to_numpy()
Y = Y.to_numpy()
Z = Z.to_numpy()

phi = phi.to_numpy()
theta = theta.to_numpy()
psi = psi.to_numpy()


Z_Setpoint = Z_Setpoint.to_numpy()
Heading_Setpoint = Heading_Setpoint.to_numpy()
Theta_Setpoint = Theta_Setpoint.to_numpy()

line1, = plt.plot(Time, Z)
line2, = plt.plot(Time, Z_Setpoint)
plt.legend([line1,line2],["Altitude (m)", "Altitude_SetPoint"])
plt.xlabel('Time (s)')
plt.ylabel('Altitude (m)')

plt.show()



line3, = plt.plot(Time, psi)
line4, = plt.plot(Time, Heading_Setpoint)
plt.legend([line1,line2],["Heading (deg)", "Heading_SetPoint"])
plt.xlabel('Time (s)')
plt.ylabel('Heading Angle (deg)')

plt.show()



line5, = plt.plot(Time, theta)
line6, = plt.plot(Time, Theta_Setpoint)
plt.legend([line1,line2],["Pitch Angle (deg)", "Pitch_Setpoint"])
plt.xlabel('Time (s)')
plt.ylabel('Pitch Angle (deg)')

plt.show()