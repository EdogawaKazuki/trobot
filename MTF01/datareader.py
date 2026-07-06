import matplotlib.pyplot as plt

logfile = "data.log"
f = open(logfile, "r")

lines = f.readlines()
data_list = []
col_names = lines[0].split()
print(col_names)
x_col = 1
y_col = 2

for line in lines[1:]:
    data = line.split(',')
    data_list.append(data)
    print(data)

x_data = [float(data[x_col]) for data in data_list]
y_data = [float(data[y_col]) for data in data_list]

plt.plot(x_data, y_data)
plt.show()

f.close()
