a = 60

for i in range(0, 360, 1):
	if (i + a) > 180:
		print((i + a) - 360)
	else:
		print(i + a)