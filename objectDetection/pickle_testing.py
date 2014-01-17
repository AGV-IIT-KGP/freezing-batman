import pickle

data1 = {"Harsh": "12MA20017",
         "Krishna": "12MA20052"}

output = open('data.pkl', 'wb')

selfref_list = [1, 2, 3]
selfref_list.append(selfref_list)

pickle.dump(data1, output)
pickle.dump(selfref_list, output, -1)
