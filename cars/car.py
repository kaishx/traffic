# kaile to keenan: this is for the car itself, it directly interfaces with the car on how it should move
# we keep the "physical movement" logic of the car in a separate file from its logic and also the larger overall brain

from math import cos, sin
from typing import Union


class Car:
	"""Simple car model.

	Attributes:
		x (float): X position.
		y (float): Y position.
		speed (float): Sp   eed in units per second (>=0).
		angle (float): Heading angle in radians, 0 = +X, positive = counter-clockwise.
		name (str): Optional identifier.

	Methods:
		update(dt): advance position by `speed * dt` in the direction of `angle`.
	"""

	def __init__(self, x: float = 0.0, y: float = 0.0, speed: float = 0.0, angle: float = 0.0, name: str = "Car"):
		self.x = float(x)
		self.y = float(y)
		self.speed = float(speed)
		self.angle = float(angle)
		self.name = str(name)

	def update(self, dt: Union[float, int] = 1.0) -> None:
		"""Update position by moving forward for `dt` seconds.

		Args:
			dt: Time step in seconds.
		"""
		dt = float(dt)
		dx = cos(self.angle) * self.speed * dt
		dy = sin(self.angle) * self.speed * dt
		self.x += dx
		self.y += dy

	def set_speed(self, v: Union[float, int]) -> None:
		self.speed = float(v)

	def set_angle(self, angle: Union[float, int]) -> None:
		self.angle = float(angle)

	def move_to(self, x: Union[float, int], y: Union[float, int]) -> None:
		self.x = float(x)
		self.y = float(y)

	def __repr__(self) -> str:
		return f"<Car name={self.name!r} x={self.x:.3f} y={self.y:.3f} vel={self.speed:.3f} ang={self.angle:.3f}>"

