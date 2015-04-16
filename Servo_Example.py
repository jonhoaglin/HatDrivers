#!/usr/bin/python

from HatDrivers import PWM,stdServo
import time
import pygame
import sys

pwm = PWM(0X40)
pan = stdServo(pwm, 50, 0, 180, 500, 2710)

pygame.init()
pygame.key.set_repeat(100, 100)
pygame.display.set_mode((100, 100))

while True:
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			sys.exit()
		if event.type == pygame.KEYDOWN:
			if event.key == pygame.K_LEFT:
				pan.addDegree(1)
			elif event.key == pygame.K_RIGHT:
				pan.addDegree(-1)
			elif event.key == pygame.K_ESCAPE:
				sys.exit()
		elif event.type == pygame.KEYUP:
			if event.key == pygame.K_LEFT or event.key == pygame.K_RIGHT:
				pan.moveToDegree(0)
