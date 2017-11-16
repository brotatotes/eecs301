from asnfuncs import *
import time

# 3 inches from base of wall to front base of robot
# use dms for higher resolution

class Asn():

	def __init__(self, context):
		self.dms = context["sensors"]["dms"]
		self.ir1 = context["sensors"]["ir1"]

		self.top_motor = context["motors"]["top_motor"]
		self.bot_motor = context["motors"]["bot_motor"]

		self.data_file = context["data_file"]

		self.max_n = context["max_n"]

		self.onedeg = 3.41

		self.tendeg = 34

	def collect_data(self):
		first = True
		for o_adc in range(512 - int(45 * self.onedeg), 512 + int(45 * self.onedeg) + 1, int(self.onedeg)):
			self.collect_datum(o_adc, first)
			if first:
				first = False

	def collect_datum(self, orientation_adc, first = False):
		self.setup(orientation_adc)
		if first:
			time.sleep(0.5)
		data = self.sweep()
		result = getMotorPositionCommand(self.bot_motor)
		data.append(result)
		self.save_data(data)


	def setup(self, orientation_adc):
		setMotorTargetPositionCommand(self.bot_motor, orientation_adc)

	def sweep(self):
		vals = []
		for adc in range(512 - self.tendeg * 4, 512 + self.tendeg * 4 + 1, self.tendeg):
			setMotorTargetPositionCommand(self.top_motor, adc)
			time.sleep(0.5)
			vals.append(sorted([getSensorValue(self.dms)]*5)[2])
		return vals

	def save_data(self, data):
		print "Saving", data, "...",
		row = ",".join(map(str, data))
		with open("data.csv", "a") as df:
		    df.write(row + "\n")
		print "Saved!"

