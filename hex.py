#!/usr/bin/python3

import maestro
import time

control = maestro.Controller(ttyStr='/dev/ttyO1')

servo_info = [
	{'pin':0, 'home':7500, 'dir':-1},
	{'pin':1, 'home':6900, 'dir':-1},
	{'pin':2, 'home':9000, 'dir':1},
	{'pin':3, 'home':4500, 'dir':1},
	{'pin':5, 'home':5100, 'dir':1},
	{'pin':4, 'home':3000, 'dir':-1},

	{'pin':6, 'home':7500, 'dir':-1},
	{'pin':7, 'home':6900, 'dir':-1},
	{'pin':8, 'home':9000, 'dir':1},
	{'pin':9, 'home':4500, 'dir':1},
	{'pin':10, 'home':5100, 'dir':1},
	{'pin':11, 'home':3000, 'dir':-1},

	{'pin':12, 'home':7500, 'dir':-1},
	{'pin':13, 'home':6900, 'dir':-1},
	{'pin':14, 'home':9000, 'dir':1},
	{'pin':15, 'home':4500, 'dir':1},
	{'pin':16, 'home':5100, 'dir':1},
	{'pin':17, 'home':3000, 'dir':-1}
]

coxa = [0,3,6,9,12,15]
tibia = [1,4,7,10,13,16]
patella = [2,5,8,11,14,17]

def move(servo, angle):
	info = servo_info[servo]
	control.setTarget(info['pin'], int(info['home']+angle*3000/90*info['dir']))

def compact():
	for leg in range(6):
		#move(coxa[leg], 0)
		move(tibia[leg], 60)
		move(patella[leg], -140)

compact()


