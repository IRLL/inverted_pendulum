#!/usr/bin/python

import math
import sys

from PIL import Image, ImageDraw


class Visualizer:
    def __init__(self, track_length, screen_size=(640, 480)):
        # How much of the screen should the rod length be
        self.rod_length_factor = 0.2


        (self.xsize, self.ysize) = screen_size
        self.track_length = track_length

        # Create screen object.
        # Give the screen a white background.
        self.screen = Image.new("RGB", screen_size, (255, 255, 255))
        self.draw = ImageDraw.Draw(self.screen)

        self.rod_length = self.rod_length_factor * min(self.xsize, self.ysize)

        #calculate track positions on the screen
        self.track_origin = self.rod_length
        self.track_end = self.xsize - self.rod_length

    def update(self, cartx, angle, dx, dangle, action):
        position = cartx
        text_angle = angle
        angle -= math.pi*3.0/2.0

        cartx += self.track_length/2 

        #scale the cart position
        cartx = float(self.xsize-(self.rod_length*2))/self.track_length * cartx + self.track_origin

        #calculate the mass position
        massx = cartx + self.rod_length*math.cos(angle)
        massy = self.ysize/2 - self.rod_length*math.sin(angle)

        # Give the screen a white background.
        self.draw.rectangle((0, 0, self.xsize, self.ysize),
                            fill=(255, 255, 255))
        # Draw the track
        self.draw.line((self.track_origin, self.ysize/2,
                        self.track_end, self.ysize/2),
                       fill=(0, 0, 0),
                       width=3)

        #draw the rod
        self.draw.line((cartx, self.ysize/2,
                        massx, massy),
                       fill=(0, 0, 255),
                       width=2)

        #draw the cart
        self.draw.ellipse((int(cartx) - 4, self.ysize/2 - 4,
                           int(cartx) + 4, self.ysize/2 + 4),
                          fill=(255, 0, 0))

        #draw the mass at the end of the rod
        self.draw.ellipse((int(massx) - 2, int(massy) - 2,
                           int(massx) + 2, int(massy) + 2),
                          fill=(0, 0, 0))

    def save_screen(self, filename):
        self.screen.save(filename)
