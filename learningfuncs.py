from asnfuncs import *
import time

class Asn():

	def __init__(self, context):
		self.dms = context["sensors"]["dms"]
		self.ir1 = context["sensors"]["ir1"]

		self.top_motor = context["motors"]["top_motor"]
		self.bot_motor = context["motors"]["bot_motor"]

		self.data_file = context["data_file"]

		self.max_n = context["max_n"]

	def collect_data(self):
		pass

	def setup(self, orientation_adc):
		setMotorTargetPositionCommand(this.bot_motor, orientation_adc)

	def sweep(self):
		pass