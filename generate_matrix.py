import random

f = open("matrix.txt", 'w' )


grid_size = (5, 5)
for x in range(grid_size[0]):
    for y in range(grid_size[1]):
        mid_point = random.randrange(1, 101)
        lower_point = random.randrange(0, mid_point)
        upper_point = random.randrange(mid_point, 101)
        f.write("o0 " + str(x) + " " +  str(y) + " " +  str(lower_point) + " " +  str(mid_point-lower_point) + " " +  str(upper_point-mid_point) + " " +  str(100-upper_point) + " 0\n")

for x in range(grid_size[0]):
    for y in range(grid_size[1]):
        mid_point = random.randrange(1, 101)
        lower_point = random.randrange(0, mid_point)
        upper_point = random.randrange(mid_point, 101)
        f.write("o1 " + str(x) + " " +  str(y) + " " +  str(lower_point) + " " +  str(mid_point-lower_point) + " " +  str(upper_point-mid_point) + " " +  str(100-upper_point) + " 0\n")

for x in range(grid_size[0]):
    for y in range(grid_size[1]):
        mid_point = random.randrange(1, 101)
        lower_point = random.randrange(0, mid_point)
        upper_point = random.randrange(mid_point, 101)
        f.write("a0 " + str(x) + " " +  str(y) + " " +  str(lower_point) + " " +  str(mid_point-lower_point) + " " +  str(upper_point-mid_point) + " " +  str(100-upper_point) + " 0\n")

'''
for x in range(10):
    for y in range(10):
        mid_point = random.randrange(1, 101)
        lower_point = random.randrange(0, mid_point)
        upper_point = random.randrange(mid_point, 101)
        f.write("a1 " + str(x) + " " +  str(y) + " " +  str(lower_point) + " " +  str(mid_point-lower_point) + " " +  str(upper_point-mid_point) + " " +  str(100-upper_point) + " 0\n")
'''

f.close()


