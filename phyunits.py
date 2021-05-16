#
# physical units
#
# developed by Soonkyu Jeong
# last modified Jan 31, 2021
#

from math import pi

inf = float('inf')

# basic
meter = 1.0
kg = 1.0
sec = 1.0

# length
cm = 1e-2*meter
mm = 1e-3*meter
km = 1e3*meter

# time
msec = 1e-3*sec
minute = 60*sec
hour = 60*minute

# speed
kph = km/hour

# acceleration
mps2 = meter/(sec*sec)

# angle
rad = 1.0
deg = pi/180

# force
Newton = 1.0

# torque
Nm = Newton*meter

# power
Watt = 1.0
