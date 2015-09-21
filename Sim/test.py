from math import *
import pygame

dt = 0.01
g = 9.81
l = 1.0
m = 1.0

global k1, k2, k3
k1 = 5000
k2 = 100
k3 = 1



clock = pygame.time.Clock()
pygame.init()
size = (width, height) = (1200,600)
screen = pygame.display.set_mode(size)

class Pendulum:
	def __init__(self, x0, y0, angle0):
		self.angle0 = angle0
		self.angle = angle0
		self.velocity = 0
		self.x0 = x0
		self.y0 = y0
		self.x0_vel = 0
		self.x = x0 + 250.0 * sin(angle0)
		self.y = y0 + 250.0 * cos(angle0)
	def move(self, control):
		self.angle = atan2(self.x - self.x0, self.y - self.y0)
		d_velocity = -g * sin(self.angle) * dt / l
		self.velocity = self.velocity + d_velocity
		d_angle = dt * self.velocity
		self.angle = self.angle + d_angle
		self.x = self.x0 + 250.0 * sin(self.angle)
		self.y = self.y0 + 250.0 * cos(self.angle)

		d_x0_vel = dt * control
		self.x0_vel = self.x0_vel + d_x0_vel
		dx0 = dt * self.x0_vel
		self.x0 = self.x0 + dx0

		if self.x0 > 1200 or self.x0 < 0:
			self.x0 = self.x0 - dx0

	def draw(self):
		pygame.draw.circle(screen, (0,0,0), [int(self.x0), int(self.y0)], 5)
		pygame.draw.line(screen, (0,0,0), [self.x0, self.y0], [self.x, self.y], 2)
		pygame.draw.circle(screen, (255,0,0), [int(self.x), int(self.y)], 10)
		pygame.draw.line(screen, (0,0,0), [0, self.y0], [1200, self.y0], 3)

class PID:
	def __init__(self, k1, k2, k3, pendulum):
		self.k1 = k1
		self.k2 = k2
		self.k3 = k3
		self.p = pendulum
		self.error = pi - self.p.angle
		self.derivative = 0
		self.integral = 0
	def update(self):
		self.k1 = k1
		self.k2 = k2
		self.k3 = k3

		tmp = self.error
		self.error = copysign(1, p.angle) * (pi - abs(self.p.angle)) + (self.p.x0 - 600) / 10000
		diff = self.error - tmp
		self.derivative = diff / dt
		self.integral = self.integral + tmp
		
		# print self.error
	def output(self):
		return self.k1 * self.error + self.k2 * self.derivative + self.k3 * self.integral

def pid_tune(pid, pend):
	global k1, k2, k3
	# new_pend = Pendulum(pend.x0, pend.y0, pend.angle)
	# new_pid = PID(pid.k1, pid.k2, pid.k3, new_pend)
	# new_pid.derivative = pid.derivative
	# new_pid.integral = pid.integral
	n_params = 3
	init_params = [pid.k1, pid.k2, pid.k3]
	dparams    = [50, 100, 1]
	params     = [0.0 for i in range(n_params)]

	for i in range(n_params):
		params[i] = init_params[i]

	best_error = 0.0;
	for t in range(100):
		screen.fill((255,255,255))
		pid.update()
		pend.move(pid.output())
		best_error = best_error + abs(pend.angle)
		pend.draw()
		clock.tick(60)
		pygame.display.flip()
	best_error = best_error / 100
	best_error = 3 * (pi - best_error)
	print best_error

	n = 1
	while sum(dparams) > 100 and n <= 15:
		# print dparams
		for i in range(len(params)):
			params[i] += dparams[i]
			err = 0

			[k1, k2, k3] = params
			for t in range(100):
				screen.fill((255,255,255))
				pid.update()
				pend.move(pid.output())
				err = err + abs(pend.angle)
				pend.draw()
				clock.tick(60)
				pygame.display.flip()

			err = err / 100
			err = pi - err
			err = (1 + 1/n)*err


			if err < best_error:
				best_error = err
				dparams[i] *= 1.1
			else:
				params[i] -= 2.0 * dparams[i]            
				err = 0
				
				[k1, k2, k3] = params
				for t in range(100):
					screen.fill((255,255,255))
					pid.update()
					pend.move(pid.output())
					err = err + abs(pend.angle)
					pend.draw()
					clock.tick(60)
					pygame.display.flip()

				err = err / 100
				err = pi - err
				err = (1 + 1/n)*err

				if err < best_error:
					best_error = err
					dparams[i] *= 1.1
				else:
					params[i] += dparams[i]
					dparams[i] *= 0.5
		n += 1
		print '#', n, params, ' -> ', best_error
		print dparams
	print ' '

	return params


def draw_text():
	myfont = pygame.font.SysFont("monospace", 15)
	label1 = myfont.render("Proportional gain: %d" % k1, 1, (255,0,0))
	screen.blit(label1, (100, 500))
	label2 = myfont.render("Derivative gain: %d" % k2, 1, (255,0,0))
	screen.blit(label2, (100, 520))
	label3 = myfont.render("Integral gain: %.1f" % k3, 1, (255,0,0))
	screen.blit(label3, (100, 540))

def make_buttons(k1, k2, k3, pid, pend):
	pygame.draw.rect(screen, (0, 0, 255), [320, 500, 90, 15])
	pygame.draw.rect(screen, (0, 0, 255), [420, 500, 90, 15])
	pygame.draw.rect(screen, (0, 0, 255), [320, 520, 90, 15])
	pygame.draw.rect(screen, (0, 0, 255), [420, 520, 90, 15])
	pygame.draw.rect(screen, (0, 0, 255), [320, 540, 90, 15])
	pygame.draw.rect(screen, (0, 0, 255), [420, 540, 90, 15])
	pygame.draw.rect(screen, (0, 0, 255), [600, 500, 120, 55])

	myfont = pygame.font.SysFont("monospace", 15)
	label1 = myfont.render("Increase", 1, (255,255,255))
	label2 = myfont.render("Decrease", 1, (255,255,255))
	label3 = myfont.render("Tune", 1, (255,255,255))
	label4 = myfont.render("automatically", 1, (255,255,255))
	screen.blit(label1, (320, 500))
	screen.blit(label2, (420, 500))
	screen.blit(label1, (320, 520))
	screen.blit(label2, (420, 520))
	screen.blit(label1, (320, 540))
	screen.blit(label2, (420, 540))
	screen.blit(label3, (640, 500))
	screen.blit(label4, (602, 530))

	if(pygame.mouse.get_pressed()[0]):
		pos = (pos1, pos2) = pygame.mouse.get_pos()
		if (320 <= pos1 <= 410 and 500 <= pos2 <= 515):
			k1 = k1 + 10
		elif (420 <= pos1 <= 490 and 500 <= pos2 <= 515):
			k1 = k1 - 10
		elif (320 <= pos1 <= 410 and 520 <= pos2 <= 535):
			k2 = k2 + 1
		elif (420 <= pos1 <= 490 and 520 <= pos2 <= 535):
			k2 = k2 - 1
		elif (320 <= pos1 <= 410 and 540 <= pos2 <= 555):
			k3 = k3 + 0.1
		elif (420 <= pos1 <= 490 and 540 <= pos2 <= 555):
			k3 = k3 - 0.1
		elif (600 <= pos1 <= 720 and 500 <= pos2 <= 555):
			(k1, k2, k3) = pid_tune(pid, pend)

	return (k1, k2, k3)



p = Pendulum(500, 300, pi-pi/10)
pid = PID(k1, k2, k3, p)
while 1:
	screen.fill((255,255,255))
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			pygame.quit()
	pid.update()
	p.move(pid.output())
	p.draw()
	draw_text()
	(k1, k2, k3) = make_buttons(k1, k2, k3, pid, p)


	clock.tick(60)
	pygame.display.flip()
