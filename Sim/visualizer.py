#!/usr/bin/python

import pygame
import math


class Visualizer:
	def __init__(self, track_length, screen_size=(640,480)):
		pygame.init()
		self.rod_length_factor = .25 #how much of the screen should the rod length be
		(self.xsize, self.ysize) = screen_size
		self.track_length = track_length
		self.screen = pygame.display.set_mode(screen_size) #create screen object
		self.screen.fill((255,255,255)) #give screen white background
		
		self.rod_length = self.rod_length_factor * min(self.xsize, self.ysize)
		
	def draw(self, cartx, angle):
		#scale the cart position
		cartx = float(self.xsize)/self.track_length * cartx

		#calculate the mass position
		massx = cartx + self.rod_length*math.cos(angle)
		massy = self.ysize/2 - self.rod_length*math.sin(angle)

		self.screen.fill((255,255,255)) #give screen white background
		#draw the track 
		pygame.draw.line(self.screen, (0,0,0), [0, self.ysize/2], [self.xsize, self.ysize/2], 3)

		#draw the rod
		pygame.draw.line(self.screen, (0,0,255), [cartx, self.ysize/2], [massx, massy], 2)
	
		#draw the cart
		pygame.draw.circle(self.screen, (255,0,0), [int(cartx), self.ysize/2], 8)

		#draw the mass at the end of the rod
		pygame.draw.circle(self.screen, (0,0,0), [int(massx), int(massy)], 5)

		pygame.display.flip()	
		


def tester():
	import time
	screen = Visualizer(track_length=1000)
	angle = 0
	position = 500
	adder = 1
	while(1):
		screen.draw(position, math.pi/180*angle)
		time.sleep(.01)
		angle += 1
		position += adder
		if(position > 800):
			adder = -1
		if(position < 200):
			adder = 1

if __name__ == "__main__":
	tester()
