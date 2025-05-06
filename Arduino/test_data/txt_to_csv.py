import numpy as np

file_name = 'step_response.txt'

data = np.loadtxt(file_name, skiprows=1, delimiter=' ')

print(data)

file_name = file_name.split('.')[0] + '.csv'

np.savetxt(file_name, data, delimiter=',', header='Time,Value', comments='')
  