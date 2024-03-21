import matplotlib.pyplot as plt
import csv

data_set_1 = []
data_set_2 = [] 

with open('encoder_data_01.txt') as file:
        csv_reader = csv.reader(file)
        for row in csv_reader:
            data1 = row[0]
            data2 = row[1]
            data_set_1.append(float(data1))
            data_set_2.append(float(data2))


print (data_set_1) 
print (data_set_2) 

#Creating 1 output plot window which will have all the plots:
fig = plt.figure('Motor Encoder Analysis')


plot1 = fig.add_subplot(121)
plot1.set_title("Rear Right Motor Encoder Analysis", fontsize = 10)
plot1.set_xlabel("GPIO Input Reading")
plot1.set_ylabel("Encoder State")
plot1.plot(data_set_1)

plot2 = fig.add_subplot(122)
plot2.set_title("Front Left Motor Encoder Analysis", fontsize = 10)
plot2.set_xlabel("GPIO Input Reading")
plot2.set_ylabel("Encoder State")
plot2.plot(data_set_2)

plt.show()