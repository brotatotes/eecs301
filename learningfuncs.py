from asnfuncs import *
import time, math, random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from pprint import pprint
import decimal as d

# 3 inches from base of wall to front base of robot
# use dms for higher resolution

class Asn3():

	def __init__(self, context):
		self.dms = context["sensors"]["dms"]
		self.ir1 = context["sensors"]["ir1"]

		self.top_motor = context["motors"]["top_motor"]
		self.bot_motor = context["motors"]["bot_motor"]

		self.data_files = context["data_files"]

		self.onedeg = 3.41

		self.tendeg = 34

		self.learner = Asn3Learner(context)

	def collect_data(self):
		first = True
		# 359 to 665 (inclusive)
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
		return data

	def setup(self, orientation_adc):
		setMotorTargetPositionCommand(self.bot_motor, orientation_adc)

	def sweep(self):
		vals = []
		for adc in range(512 - self.tendeg * 4, 512 + self.tendeg * 4 + 1, self.tendeg):
			setMotorTargetPositionCommand(self.top_motor, adc)
			time.sleep(0.5)
			vals.append(sorted([getSensorValue(self.dms) for _ in range(5)])[2])
		return vals

	def save_data(self, data):
		print "Saving", data, "...",
		row = ",".join(map(str, data))
		with open(self.data_files["data3"], "a") as df:
		    df.write(row + "\n")
		print "Saved!"

	def calibrate(self):
		data1 = self.collect_datum(512, True)

		data2 = self.collect_datum(512, True)

		data3 = self.collect_datum(512, True)

		data = []
		for i in range(len(data1)):
			data.append(sorted([data1[i], data2[i], data3[i]])[1])

		print data[:-1], data[-1]
		self.learner.calibrate(data[:-1], 510)
		print self.learner.calibration_factor


class Asn3Learner():
	def __init__(self, context):
		self.data_files = context["data_files"]
		self.data_file = self.data_files["data1"]
		# self.max_n = context["max_n"] # do we need this?

		# constraint: train_size + test_size <= 1
		self.train_size = context["train_test_sizes"]["train_size"]
		self.test_size = context["train_test_sizes"]["test_size"]

		self.calibration_factor = 0

		self.read_data(self.data_file)
		self.data = self.clean_data(self.data)
		self.partition_data()

		self.test_set = self.clean_data(self.read_data(self.data_files["data2"]))
		self.train_set = self.clean_data(self.read_data(self.data_files["data1"]))

	def read_data(self, data_file):
		with open(data_file) as df:
			df.readline()
			data = df.readlines()

		for i, d in enumerate(data):
			parsed = map(int, d.replace("\n","").split(","))
			data[i] = [parsed[-1], parsed[:-1]]

		self.data = data
		return data

	def distance(self, x1, x2):
		return math.sqrt(sum([abs(i1 - i2) for i1,i2 in zip(x1,x2)]))

	def weigh(self, x1, x2):
		d = self.distance(x1, x2)
		return math.exp(-d)

	def compute_result(self, x):
		minn = 700
		maxx = 2000
		x = map(lambda xx: xx + self.calibration_factor, x)
		x = map(lambda xx: 0 if xx <= minn else xx, x)
		x = map(lambda xx: maxx if xx > maxx else xx, x)
		weights = [self.weigh(datum[1], x) for datum in self.train_set]
		res = sum([w * datum[0] for w,datum in zip(weights, self.train_set)]) / (sum(weights) + np.finfo(np.float64).tiny)
		if res < 359 or res > 665:
			print x, weights
		return res

	def partition_data(self):
		random.seed(0)

		n = len(self.data)
		testn = int(0.2 * n)
		test_sample_indices = random.sample(range(n), testn)

		test_set = []
		full_train_set = []

		for i in range(n):
			if i in test_sample_indices:
				test_set.append(self.data[i])
			else:
				full_train_set.append(self.data[i])

		self.test_set = test_set
		self.train_set = random.sample(full_train_set, int((self.train_size + self.test_size) * len(full_train_set)))

	def clean_data(self, data, test=False):
		maxx = 2000
		minn = 700
		data = filter(lambda x: 359 <= x[0] <= 665, data)
		if data:
			data = map(lambda x: [x[0], map(lambda y: 0 if y <= minn else y, x[1])], data)
			data = map(lambda x: [x[0], map(lambda y: maxx if y > maxx else y, x[1])], data)
		return data

	def evaluate(self):
		errors = []

		for t in self.test_set:
			res = self.compute_result(t[1])
			er = abs(res - t[0])
			# print "computed:", self.compute_result(t[1]), "\nactual", t[0], "\nerror:", er
			# print
			errors.append(er)

		print "calculated", len(errors), "data points"
		print "max error:", max(errors)
		print "min error:", min(errors)
		print "median error:", np.median(errors)
		print "avg error:", np.mean(errors)

		return errors

	def calibrate(self, x, orientation_adc):
		v = []
		for d in self.data:
			if d[0] == orientation_adc:
				if not v:
					v = [[i] for i in d[1]]
				else:
					for i in range(len(self.data[0][1])):
						v[i].append(d[1][i])

		v = map(lambda x: np.median(x), v)

		d = self.distance(x, v)
		prevd = float("inf")
		c = 0

		while d < prevd:
			c += 1
			v = map(lambda x: x + 1, v)
			prevd = d
			d = self.distance(x, v)

		best_d = d
		best_c = c

		d = self.distance(x, v)
		prevd = float("inf")
		c = 0

		while d < prevd:
			c -= 1
			v = map(lambda x: x - 1, v)
			prevd = d
			d = self.distance(x, v)

		if d < best_d:
			best_d = d
			best_c = c

		self.calibration_factor = -c

	def plot_data_std(self, data_file):
		data = self.clean_data(self.read_data(data_file))
		v = {}
		for d in data:
			if d[0] in v:
				for i in range(len(data[0][1])):
					v[d[0]][i].append(d[1][i])
			else:
				v[d[0]] = []
				for i in range(len(data[0][1])):
					v[d[0]].append([d[1][i]])

		for k in v.keys():
			for i in range(len(v[k])):
				v[k][i] = np.std(v[k][i])

		colors = iter(cm.rainbow(np.linspace(0, 1, len(data[0][1]))))
		for i in range(len(data[0][1])):
			plt.scatter(v.keys(), [v[k][i] for k in v.keys()], label='$x{i}$'.format(i=i), color=next(colors))
		plt.ylabel('standard deviation')
		plt.xlabel('output')
		plt.legend(loc=2)
		plt.show()



if __name__ == "__main__":
	# Sensor setup
	ir1 = 3
	dms = 4

	sensors = {"ir1": ir1, "dms": dms}

	top_motor = 9
	bot_motor = 1

	motors = {"top_motor": top_motor, "bot_motor":bot_motor}

	data_files = {"data1": "data1.csv", "data2": "data2.csv"}

	train_test_sizes = {"test_size": 0.2, "train_size": 0.8}

	context = {"sensors": sensors, "motors": motors, "data_files": data_files, "max_n": 9, "train_test_sizes": train_test_sizes}


	l = Asn3Learner(context)

	l.calibrate([1279,1679,1689,1937,1876,1954,1880,1891,1577],510)
	print l.calibration_factor


	errors = l.evaluate()
	

	# l.plot_data_std("data1.csv")

	# print l.compute_result([1777,1845,1874,1911,1830,1693,1498,1816,1642])
