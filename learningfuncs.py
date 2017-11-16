# from asnfuncs import *
import time, math

# 3 inches from base of wall to front base of robot
# use dms for higher resolution

class Asn3():

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
		with open(self.data_file, "a") as df:
		    df.write(row + "\n")
		print "Saved!"

class Asn3Learner():
	def __init__(self, context):
		self.data_file = context["data_file"]
		self.max_n = context["max_n"]
		self.read_data()

	def read_data(self):
		with open(self.data_file) as df:
			df.readline()
			data = df.readlines()

		for i, d in enumerate(data):
			parsed = map(int, d.replace("\n","").split(","))
			data[i] = (parsed[-1], parsed[:-1])

		self.data = data
		return data

	def distance(self, x1, x2):
		return math.sqrt(sum([(i1 - i2)*(i1 - i2) for i1,i2 in zip(x1,x2)]))

	def weigh(self, x1, x2):
		d = self.distance(x1, x2)
		return math.exp(-d)

	def compute_result(self, x):
		weights = [self.weigh(datum[1], x) for datum in self.data]
		return sum([w * datum[0] for w,datum in zip(weights, self.data)]) / sum(weights)

m = Asn3Learner({"data_file":"data.csv", "max_n": 9})
m.read_data()
print m.compute_result([0,0,127,537,1279,1545,1753,1914,1990])
